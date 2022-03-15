#include "svo/gtsam_backend_interface.hpp"

#include <svo/common/conversions.h>
#include <svo/common/frame.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/global.h>
#include <fstream>

namespace svo
{
  GtsamBackendInterface::GtsamBackendInterface(
      const GtsamBackendInterfaceOptions &options,
      const GtsamBackendOptions &backend_options,
      const MotionDetectorOptions &motion_detector_options,
      const CameraBundlePtr &camera_bundle)
      : options_(options), backend_options_(backend_options)
  {
    type_ = BundleAdjustmentType::kGtsam;

    // Setup modules
    if (options_.use_zero_motion_detection)
    {
      motion_detector_.reset(new MotionDetector(motion_detector_options));
    }
    if (options_.use_outlier_rejection)
    {
      outlier_rejection_.reset(
          new OutlierRejection(options_.outlier_rejection_px_threshold));
    }

    backend_.addCamParams(camera_bundle);

    LOG(FATAL) << "Constructor for GtsamBackendInterface not implemented yet.";
  }

  GtsamBackendInterface::~GtsamBackendInterface()
  {
    if (thread_ != nullptr)
    {
      quitThread();
    }
  }

  void GtsamBackendInterface::loadMapFromBundleAdjustment(const FrameBundlePtr &new_frames,
                                                          const FrameBundlePtr &last_frames,
                                                          const MapPtr &map,
                                                          bool &have_motion_prior)
  {
    if (stop_thread_)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_backend_);

    // Setup motion detector
    if (motion_detector_)
    {
      motion_detector_->setFrames(last_frames, new_frames);
    }

    // get the imu measurements up to the timestamp of the frame and save them in
    // a member variable
    bool success = getImuMeasurements(new_frames->getMinTimestampSeconds());

    // this adds the latest imu measurements and preintegrates them
    backend_.addImuMeasurements(latest_imu_meas_);

    // add projection factors using frames
    // TODO: should this go here or in the bundleAdjustment/optimizationLoop function?
    // TODO: should it take in both new_frames and last_frames, or just new_frames?
    // backend_.addLandmarkObservations(new_frames, last_frames);

    // gets the latest estimate from the backend and sets the frame T_W_B.
    // should also update speed and bias of the frame using latest estimate from backend.
    updateBundleStateWithBackend(new_frames);

    // after this call we should have motion prior available
    // TODO: how is this used? is the motion prior from the framebundle variables?
    // TODO: when do we *not* have motion prior available? some of the above calls might fail,
    //       this should be handled.
    have_motion_prior = true;

    // Skip updating the frame variables if no update is available
    if (last_updated_nframe_ == last_optimized_nframe_.load())
    {
      VLOG(3) << "No map update available.";
      return;
    }

    // Update active keyframes with latest estimate
    int n_frames_updated = 0;
    for (const FramePtr &keyframe : active_keyframes_)
    {
      // should do the same as updateBundleStateWithBackend, but just on one single
      // frame instead of the bundle. there's a subtle difference. see ceres backend.
      // second argument is if it should update speed and bias or not.
      // TODO: is this needed? might be for speed reasons? might just leave it out
      updateFrameStateWithBackend(keyframe, false);
      n_frames_updated++;
    }

    // Update last frame bundle
    for (const FramePtr &last_frame : *last_frames)
    {
      // ceres backend only updates if last_frame is *not* keyframe, why?
      // TODO: should gtsam backend also only update when not keyframe?
      // need better understanding of what last_frames actually are.
      updateFrameStateWithBackend(last_frame, true);
    }

    if (outlier_rejection_ && last_frames)
    {
      size_t n_deleted_edges = 0;
      size_t n_deleted_corners = 0;
      std::vector<int> deleted_points;
      for (FramePtr &frame : *last_frames)
      {
        outlier_rejection_->removeOutliers(
            *frame, n_deleted_edges, n_deleted_corners, deleted_points);
      }
      // remove landmark variables from the factor graph based on the id
      // from outlier rejection
      backend_.removePointsByPointIds(deleted_points);
      VLOG(6) << "Outlier rejection: removed " << n_deleted_edges
              << " edgelets and " << n_deleted_corners << " corners.";
    }

    gtsam_backend::SpeedAndBias speed_and_bias;
    // gets the latest speed and bias estimate for last_frames (likely from the preintegration)
    bool success = backend_.getSpeedAndBias(last_frames->getBundleId(), speed_and_bias);
    imu_handler_->setAccelerometerBias(speed_and_bias.tail<3>());
    imu_handler_->setGyroscopeBias(speed_and_bias.segment<3>(3));

    // shift state
    last_updated_nframe_ = last_optimized_nframe_.load();
  }

  void GtsamBackendInterface::bundleAdjustment(const FrameBundlePtr &frame_bundle)
  {
    if (stop_thread_)
      return;

    // if imu measurements could not be added.
    if (last_added_nframe_imu_ == last_added_nframe_images_)
      return;

    std::lock_guard<std::mutex> lock(mutex_backend_);

    // Checking for zero motion
    bool velocity_prior_added = false;
    if (motion_detector_)
    {
      double sigma = 0;
      if (!motion_detector_->isImageMoving(sigma))
      {
        ++no_motion_counter_;

        if (no_motion_counter_ > options_.backend_zero_motion_check_n_frames)
        {
          image_motion_detector_stationary_ = true;
          VLOG(5) << "Image is not moving: adding zero velocity prior.";
          // add velocity prior to keyframe with id getBundleId. In this case
          // add zero velocity. sigma should be uncertainty for factor
          if (!backend_.addVelocityPrior(frame_bundle->getBundleId(), gtsam::Z_3x1, sigma))
          {
            LOG(ERROR) << "Failed to add a zero velocity prior!";
          }
          else
          {
            velocity_prior_added = true;
          }
        }
      }
      else
      {
        image_motion_detector_stationary_ = false;
        no_motion_counter_ = 0;
      }
    }

    // only use imu-based motion detection when the images are not good
    if (!image_motion_detector_stationary_ && imu_motion_detector_stationary_)
    {
      VLOG(5) << "IMU determined stationary, adding prior at time "
              << frame_bundle->at(0)->getTimestampSec() << std::endl;
      if (!backend_.addVelocityPrior(frame_bundle->getBundleId(), gtsam::Z_3x1, 0.005))
      {
        LOG(ERROR) << "Failed to add a zero velocity prior!";
      }
      else
      {
        velocity_prior_added = true;
      }
    }

    for (const FramePtr &frame : *frame_bundle)
    {
      if (frame->isKeyframe())
      {
        active_keyframes_.push_back(frame);
        // adds landmarks not already in the factor graph to the FC
        // also adds observations (reprojection factors) for the landmarks
        // seen in this frame
        addLandmarksAndObservationsToBackend(frame);
        // backend_.addLandmarks(frame);
        // backend_.addObservations(frame);
      }
      else
      {
        // TODO: what to do with the remaining observations? the ceres backend
        //       seem to also add observations from the non-keyframe frames, but
        //       how can this be done in the factor graph when one observations =
        //       one factor?
      }
    }

    last_added_nframe_images_ = frame_bundle->getBundleId();
    last_added_frame_stamp_ns_ = frame_bundle->getMinTimestampNanoseconds();

    wait_condition_.notify_one();
  }

  void GtsamBackendInterface::reset()
  {
    LOG(ERROR) << "GtsamBackendInterface::reset() is not implemented.";
  }

  void GtsamBackendInterface::startThread()
  {
    CHECK(thread_ == nullptr) << "Tried to start thread that is already running!";
    stop_thread_ = false;
    thread_.reset(new std::thread(&GtsamBackendInterface::optimizationLoop, this));
  }

  void GtsamBackendInterface::quitThread()
  {
    VLOG(1) << "Interrupting and stopping optimization thread.";
    stop_thread_ = true;
    if (thread_ != nullptr)
    {
      wait_condition_.notify_all();
      thread_->join();
      thread_.reset();
    }
    VLOG(1) << "Thread stopped and joined.";
  }

  void GtsamBackendInterface::getAllActiveKeyframes(std::vector<FramePtr> *keyframes)
  {
    CHECK_NOTNULL(keyframes);
    keyframes->clear();
    keyframes->insert(keyframes->begin(), active_keyframes_.begin(),
                      active_keyframes_.end());
  }

  void GtsamBackendInterface::setImuHandler(const std::shared_ptr<ImuHandler> imu_handler)
  {
    imu_handler_ = imu_handler;

    const ImuCalibration &calib = imu_handler_->imu_calib_;
    // TODO: which direction should gravity_magnitude be? + or - coeff?
    auto p = std::make_shared<gtsam_backend::ImuParameters>(gtsam::Vector3(0, 0, calib.gravity_magnitude));
    p->accelerometerCovariance = gtsam::I_3x3 * calib.acc_noise_density * calib.acc_noise_density;
    p->gyroscopeCovariance = gtsam::I_3x3 * calib.gyro_noise_density * calib.gyro_noise_density;
    p->biasAccCovariance = gtsam::I_3x3 * calib.acc_bias_random_walk_sigma * calib.acc_bias_random_walk_sigma;
    p->biasOmegaCovariance = gtsam::I_3x3 * calib.gyro_bias_random_walk_sigma * calib.gyro_bias_random_walk_sigma;
    p->integrationCovariance = gtsam::I_3x3 * options_.preintegration_sigma * options_.preintegration_sigma;
    p->biasAccOmegaInt = gtsam::I_6x6 * options_.bias_preintegration_sigma * options_.bias_preintegration_sigma;
    p->accel_max = calib.saturation_accel_max;
    p->omega_max = calib.saturation_omega_max;
    p->rate = calib.imu_rate;
    p->delay_imu_cam = calib.delay_imu_cam;

    backend_.addImuParams(p);
  }

  void GtsamBackendInterface::getLatestSpeedBiasPose(Eigen::Matrix<double, 9, 1> *speed_bias,
                                                     Transformation *T_WS,
                                                     double *timestamp) const
  {
  }

  void GtsamBackendInterface::setReinitStartValues(const Eigen::Matrix<double, 9, 1> &sb,
                                                   const Transformation &Tws,
                                                   const double timestamp)
  {
  }

  void GtsamBackendInterface::optimizationLoop()
  {
    VLOG(1) << "Started optimization thread.";
    while (!stop_thread_)
    {
      { // create scope to release lock at each iteration
        std::unique_lock<std::mutex> lock(mutex_backend_);

        wait_condition_.wait(
            lock, [&]
            { return ((last_added_nframe_images_ != last_optimized_nframe_.load())) || stop_thread_; });

        if (stop_thread_)
        {
          return;
        }

        // TODO: Marginalize, but how?

        // Optimize
        // add the new factor graph to the isam2 optimizer
        // have some extra calls to update() to get more accurate estimate
        // TODO: should this also calculate the latest estimates? i.e. call
        //       isam2::calculateEstimate() and update variables for keyframes
        //       landmarks, or should those calculations be done when they are
        //       requested (e.g. with getT_WS, getSpeedAndBias, etc)?
        //       it is possible to calculate the estimates of the variables
        //       most likely to be retrieved and handle the cases when the
        //       value is not already calculated?
        backend_.optimize();

        last_optimized_nframe_.store(last_added_nframe_images_);

        Transformation T_WS;
        backend_.getT_WS(last_optimized_nframe_.load(), T_WS);

        gtsam_backend::SpeedAndBias speed_and_bias;
        backend_.getSpeedAndBias(last_optimized_nframe_, speed_and_bias);

        // Save state for visualization
        last_state_.set_T_W_B(T_WS);
        last_state_.set_W_v_B(speed_and_bias.head<3>());
        last_state_.setGyroBias(speed_and_bias.segment<3>(3));
        last_state_.setAccBias(speed_and_bias.tail<3>());

        // TODO: publish
      } // release backend mutex
    }

    LOG(INFO) << "Optimization thread ended.";
  }

  bool GtsamBackendInterface::getImuMeasurements(double timestamp_ns)
  {
    double timestamp_s = timestamp_ns * common::conversions::kNanoSecondsToSeconds;
    if (!imu_handler_->waitTill(timestamp_s))
    {
      return false;
    }

    // Get measurements, newest is interpolated to exactly match timestamp of
    // frame_bundle
    if (!imu_handler_->getMeasurementsContainingEdges(timestamp_s, latest_imu_meas_, true))
    {
      LOG(ERROR) << "Could not retrieve IMU measurements."
                 << " Last frame was at " << last_added_frame_stamp_ns_
                 << ", current is at " << timestamp_ns;
      return false;
    }

    return true;
  }

  bool GtsamBackendInterface::updateBundleStateWithBackend(const FrameBundlePtr &new_frames)
  {
    BundleId bid = new_frames->getBundleId();

    Transformation T_WS;
    bool success = backend_.getT_WS(bid, T_WS);
    new_frames->set_T_W_B(T_WS);

    gtsam_backend::SpeedAndBias speed_and_bias;
    success = backend_.getSpeedAndBias(bid, speed_and_bias);
    // TODO: why are we rotating by velocity? taken from ceres backend
    new_frames->setIMUState(T_WS.getRotation().rotate(speed_and_bias.block<3, 1>(0, 0)),
                            speed_and_bias.block<3, 1>(3, 0),
                            speed_and_bias.block<3, 1>(6, 0));

    return true;
  }

  bool GtsamBackendInterface::updateFrameStateWithBackend(const FramePtr &frame, bool update_speed_bias)
  {
    BundleId bid = frame->bundleId();

    Transformation T_WS;
    bool success = backend_.getT_WS(bid, T_WS);
    T_WS.getRotation().normalize();
    frame->set_T_w_imu(T_WS);

    if (update_speed_bias)
    {
      gtsam_backend::SpeedAndBias speed_and_bias;
      success = backend_.getSpeedAndBias(bid, speed_and_bias);
      frame->setIMUState(T_WS.getRotation().rotate(speed_and_bias.block<3, 1>(0, 0)),
                         speed_and_bias.block<3, 1>(3, 0),
                         speed_and_bias.block<3, 1>(6, 0));
    }

    return true;
  }

  bool GtsamBackendInterface::addLandmarksAndObservationsToBackend(const FramePtr &frame)
  {
    return false;
  }

} // namespace svo
