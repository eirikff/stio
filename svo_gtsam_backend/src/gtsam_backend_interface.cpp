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
      const GtsamBackendOptions &optimizer_options,
      const MotionDetectorOptions &motion_detector_options,
      const CameraBundlePtr &camera_bundle)
      : options_(options), backend_options_(optimizer_options)
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

    LOG(FATAL) << "Constructor for GtsamBackendInterface not implemented yet.";
  }

  void GtsamBackendInterface::loadMapFromBundleAdjustment(const FrameBundlePtr &new_frames,
                                                          const FrameBundlePtr &last_frames,
                                                          const MapPtr &map,
                                                          bool &have_motion_prior)
  {
  }

  void GtsamBackendInterface::bundleAdjustment(const FrameBundlePtr &frame_bundle)
  {
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
    }
  }
}
