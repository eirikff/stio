#include "svo/gtsam_backend_interface.hpp"

#include <svo/common/conversions.h>
#include <svo/common/frame.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/global.h>
#include <fstream>
#include <iomanip>

namespace svo
{
  GtsamBackendInterface::GtsamBackendInterface(
      ros::NodeHandle &nh,
      const GtsamBackendInterfaceOptions &options,
      const GtsamBackendOptions &backend_options,
      const MotionDetectorOptions &motion_detector_options,
      const CameraBundlePtr &camera_bundle)
      : options_(options), backend_options_(backend_options), ext_pose_handler_(nh), publisher_(nullptr)
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
    bool success;

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
    success = getImuMeasurements(new_frames->getMinTimestampSeconds());

    // this adds the latest imu measurements and preintegrates them
    backend_.addImuMeasurements(latest_imu_meas_);

    // this predicts using the recently added imu measurements and associates the
    // prediction with the new_frames' bundle id
    backend_.predictWithImu(new_frames->getBundleId(), new_frames->getMinTimestampSeconds());

    // gets the latest estimate from the backend and sets the frame T_W_B.
    // should also update speed and bias of the frame using latest estimate from backend.
    updateBundleStateWithBackend(new_frames);

    // after this call we should have motion prior available
    // TODO: how is this used? is the motion prior from the framebundle variables?
    // TODO: when do we *not* have motion prior available? some of the above calls might fail,
    //       this should be handled.
    have_motion_prior = true;

    // Skip updating the frame variables if no update is available
    // if (last_updated_nframe_ == last_optimized_bid_.load())
    // {
    //   VLOG(3) << "No map update available.";
    //   return;
    // }
    // TODO: what to do here? is this check necessary?
    VLOG(3) << "UPDATING USING BACKEND";

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

    updateAllActiveLandmarks();

    // Update last frame bundle
    if (last_frames)
    {
      for (const FramePtr &last_frame : *last_frames)
      {
        if (!last_frame->isKeyframe())
          // only update if not keyframe to avoid updating the same frame twice.
          // if it is a keyframe, it would be updated in the previous loop over
          // active_keyframes_ as it stores pointers to frames.
          updateFrameStateWithBackend(last_frame, true);
      }
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

      // remove from active landmarks
      // TODO: INEFFICIENT O(n^2), fix!
      for (int id : deleted_points)
      {
        for (auto it = active_landmarks_.begin(); it != active_landmarks_.end(); it++)
        {
          if ((*it)->id() == id)
          {
            active_landmarks_.erase(it);
            break;
          }
        }
      }
      // remove landmark variables from the factor graph based on the id
      // from outlier rejection
      backend_.removePointsByLandmarkIds(deleted_points);
      VLOG(6) << "Outlier rejection: removed " << n_deleted_edges
              << " edgelets and " << n_deleted_corners << " corners.";
    }

    if (last_frames)
    {
      gtsam_backend::SpeedAndBias speed_and_bias;
      // gets the latest speed and bias estimate for last_frames (likely from the preintegration)
      success = backend_.getSpeedAndBias(last_frames->getBundleId(), speed_and_bias);
      imu_handler_->setAccelerometerBias(speed_and_bias.tail<3>());
      imu_handler_->setGyroscopeBias(speed_and_bias.segment<3>(3));
    }

    // shift state
    // last_updated_nframe_ = last_optimized_bid_.load();
  }

  void GtsamBackendInterface::bundleAdjustment(const FrameBundlePtr &frame_bundle)
  {
    if (stop_thread_)
      return;

    // double ts = frame_bundle->getMinTimestampSeconds();
    // VLOG(1) << "External prior at timestamp " << std::setprecision(17) << ts << ": \n"
    //         << ext_pose_handler_.getPose(ts).second;

    if (!is_frontend_initialized_ && frame_bundle->at(0)->isKeyframe())
    {
      is_frontend_initialized_ = true;
    }

    // if imu measurements could not be added.
    // if (last_added_nframe_imu_ == last_added_nframe_images_)
    // {
    //   VLOG(2) << "GtsamBackendInterface::bundleAdjustment: last_added_nframe_imu_ == last_added_nframe_images_ -> True";
    //   // return;
    // }

    std::lock_guard<std::mutex> lock(mutex_backend_);

    bool is_keyframe = false;
    // TODO: don't care about stereo, so frame_bundle should only have one single
    //       image. could/should this loop be remove?
    for (const FramePtr &frame : *frame_bundle)
    {
      if (frame->isKeyframe())
      {
        is_keyframe = true;
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
        //       Perhaps the remaining observations should not be added so the
        //       graph doesn't grow too large?
      }
    }

    if (is_keyframe)
    {
      backend_.increaseTotalKeyframeCount();
      backend_.addPreintFactor(frame_bundle->getBundleId());

      do_optimize_ = true;
    }

    bool success = false;
    if (!is_frontend_initialized_)
    {
      double timestamp = frame_bundle->getMinTimestampSeconds();
      gtsam::Pose3 prior = ext_pose_handler_.getPose(timestamp).second;

      BundleId bid = frame_bundle->getBundleId();
      success = backend_.addPreintFactor(bid, &prior);

      // if (!success)
      //   return;

      VLOG(3) << "Added IMU preintegration factor for bid " << bid << " before front-end is initialized.";

      // need temporary variable to run function and avoid short-circuit evaluation
      bool tmp = backend_.addExternalPosePrior(bid, prior);
      success = success && tmp;

      VLOG(3) << "Added external prior for bundle id " << bid
              << " at timestamp " << std::setprecision(17) << timestamp << ": \n"
              << std::setfill(' ')
              << "  pos = " << prior.translation().transpose() << "\n"
              << "  rpy = " << prior.rotation().rpy().transpose();
    }
    if (options_.add_external_prior_to_every_keyframe && is_keyframe && is_frontend_initialized_)
    {
      double timestamp = frame_bundle->getMinTimestampSeconds();
      gtsam::Pose3 prior = ext_pose_handler_.getPose(timestamp).second;

      BundleId bid = frame_bundle->getBundleId();
      backend_.addExternalPosePrior(bid, prior);

      VLOG(3) << "Added external prior for bundle id " << bid << " after front-end is initialized "
              << " at timestamp " << std::setprecision(17) << timestamp << ": \n"
              << std::setfill(' ')
              << "  pos = " << prior.translation().transpose() << "\n"
              << "  rpy = " << prior.rotation().rpy().transpose();
    }

    // last_added_nframe_images_ = frame_bundle->getBundleId();
    // last_added_frame_stamp_ns_ = frame_bundle->getMinTimestampNanoseconds();

    if (success)
      do_optimize_ = true;

    wait_condition_.notify_all();
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
    // this gives the same direction as MakeSharedU.
    auto p = std::make_shared<gtsam_backend::ImuParameters>(gtsam::Vector3(0, 0, -calib.gravity_magnitude));
    p->int_param = gtsam::PreintegrationCombinedParams::MakeSharedU(calib.gravity_magnitude);
    p->int_param->accelerometerCovariance = gtsam::I_3x3 * calib.acc_noise_density * calib.acc_noise_density;
    p->int_param->gyroscopeCovariance = gtsam::I_3x3 * calib.gyro_noise_density * calib.gyro_noise_density;
    p->int_param->biasAccCovariance = gtsam::I_3x3 * calib.acc_bias_random_walk_sigma * calib.acc_bias_random_walk_sigma;
    p->int_param->biasOmegaCovariance = gtsam::I_3x3 * calib.gyro_bias_random_walk_sigma * calib.gyro_bias_random_walk_sigma;
    p->int_param->integrationCovariance = gtsam::I_3x3 * options_.preintegration_sigma * options_.preintegration_sigma;
    p->int_param->biasAccOmegaInt = gtsam::I_6x6 * options_.bias_preintegration_sigma * options_.bias_preintegration_sigma;
    p->accel_max = calib.saturation_accel_max;
    p->omega_max = calib.saturation_omega_max;
    p->rate = calib.imu_rate;
    p->delay_imu_cam = calib.delay_imu_cam;

    // TODO: this needs to be a parameter or estimated from data
    // FOR EUROC VICON ROOM (T_imu_vicon = T_sensor_body)
    gtsam::Rot3 rot_IV(0.33638, -0.01749, 0.94156,
                       -0.02078, -0.99972, -0.01114,
                       0.94150, -0.01582, -0.33665);
    gtsam::Point3 trans_IV(0.06901, -0.02781, -0.12395);
    gtsam::Pose3 T_imu_vicon(rot_IV, trans_IV);
    p->int_param->body_P_sensor = T_imu_vicon.inverse();

    backend_.addImuParams(p);
  }

  void GtsamBackendInterface::getLatestSpeedBiasPose(Eigen::Matrix<double, 9, 1> *speed_bias,
                                                     Transformation *T_WS,
                                                     double *timestamp) const
  {
    BundleId bid = backend_.getLastAddedBid();

    Transformation T;
    bool success_T_WS = backend_.getT_WS(bid, T);
    *T_WS = T;

    gtsam_backend::SpeedAndBias speed_and_bias;
    bool success_speed_bias = backend_.getSpeedAndBias(bid, speed_and_bias);
    *speed_bias = speed_and_bias;

    *timestamp = backend_.getTimestampSeconds(bid);
  }

  void GtsamBackendInterface::setReinitStartValues(const Eigen::Matrix<double, 9, 1> &sb,
                                                   const Transformation &Tws,
                                                   const double timestamp)
  {
    LOG(FATAL) << "GtsamBackendInterface::setReinitStartValues: Not implemented.";
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
            { return do_optimize_ || stop_thread_; });

        do_optimize_ = false;
        VLOG(1) << "Optimization loop got notified!";

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
        last_optimized_bid_ = backend_.optimize();

        Transformation T_WS;
        backend_.getT_WS(last_optimized_bid_, T_WS);

        gtsam_backend::SpeedAndBias speed_and_bias;
        backend_.getSpeedAndBias(last_optimized_bid_, speed_and_bias);

        // std::cout << "Bias:\n"
        //           << "  acc = " << speed_and_bias.tail<3>().transpose() << "\n"
        //           << "  gyr = " << speed_and_bias.segment<3>(3).transpose() << std::endl;

        // Save state for visualization
        last_state_.set_T_W_B(T_WS);
        last_state_.set_W_v_B(speed_and_bias.head<3>());
        last_state_.setGyroBias(speed_and_bias.segment<3>(3));
        last_state_.setAccBias(speed_and_bias.tail<3>());

        if (publisher_)
        {
          publisher_->publish(
              last_state_,
              backend_.getOptimizationResults(),
              backend_.getBackendLandmarkKeys(),
              backend_.getTimestampSeconds(last_optimized_bid_),
              last_optimized_bid_);
        }

        // TODO: publish
      } // release backend mutex
    }

    LOG(INFO) << "Optimization thread ended.";
  }

  bool GtsamBackendInterface::getImuMeasurements(double timestamp_s)
  {
    if (!imu_handler_->waitTill(timestamp_s))
    {
      return false;
    }

    // Get measurements, newest is interpolated to exactly match timestamp of
    // frame_bundle
    latest_imu_meas_.clear();
    if (!imu_handler_->getMeasurementsContainingEdges(timestamp_s, latest_imu_meas_, true))
    {
      LOG(ERROR) << "Could not retrieve IMU measurements until timestamp " << timestamp_s;
      return false;
    }

    return true;
  }

  bool GtsamBackendInterface::updateBundleStateWithBackend(const FrameBundlePtr &new_frames)
  {
    BundleId bid = new_frames->getBundleId();

    Transformation T_WS;
    bool success_T_WS = backend_.getT_WS(bid, T_WS);
    if (!success_T_WS)
    {
      LOG(WARNING) << "Could not get T_WS for bundle id " << bid;
    }

    gtsam_backend::SpeedAndBias speed_and_bias;
    bool success_speedbias = backend_.getSpeedAndBias(bid, speed_and_bias);
    if (!success_speedbias)
    {
      LOG(WARNING) << "Could not get speed and bias for bundle id " << bid;
    }

    if (!success_T_WS || !success_speedbias)
      return false;

    new_frames->set_T_W_B(T_WS);
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
    for (size_t i = 0; i < frame->numFeatures(); i++)
    {
      const PointPtr &landmark = frame->landmark_vec_[i];

      if (landmark == nullptr)
      {
        continue;
      }

      if (isMapPoint(frame->type_vec_[i]))
      {
        // map points are likely points used for loop closing and global map
        // and are ignored here
        continue;
      }

      // check if we have enough observations. Might not be the case if seed
      // original frame was already dropped.
      if (landmark->obs_.size() < options_.min_obsv_count)
      {
        VLOG(10) << "Landmark (id = " << landmark->id() << ") with " << landmark->obs_.size()
                 << " have less observations than the minimum of " << options_.min_obsv_count;
        continue;
      }

      int level = frame->level_vec_(i);
      if (!backend_.addProjectionFactors(landmark, level))
      {
        LOG(WARNING) << "Failed to add projection factors.";
        continue;
      }

      active_landmarks_.push_back(landmark);
    }

    return false;
  }

  bool GtsamBackendInterface::isInitializedWithExternalPrior(double time_threshold) const
  {
    // TODO: return true when the backend has processed e.g. 5 seconds of IMU and
    //       external priors as that's the heuristic we'll use to determine if
    //       the backend is initialized.
    double t = backend_.getTimeSinceFirstExternalPrior();
    if (t >= time_threshold)
    {
      static int logging_count = 0;
      VLOG_IF(1, logging_count++ < 25) << "Time since first external prior is " << t << " sec and has exceeded time threshold "
                                       << time_threshold << " sec and is considered initialized.";
      return true;
    }
    VLOG(10) << "Time since first external prior is: " << t << " sec. Remaining to threshold: " << time_threshold - t << std::endl;
    return false;
  }

  bool GtsamBackendInterface::updateAllActiveLandmarks()
  {
    for (PointPtr &landmark : active_landmarks_)
    {
      backend_.updateLandmarkPosition(landmark);
      VLOG(20) << "Updated landmark position for landmark with id = " << landmark->id();
    }

    VLOG(5) << "Updated " << active_landmarks_.size() << " active landmarks using backend estimate.";

    return true;
  }

} // namespace svo
