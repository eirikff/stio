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
      : options_(options), optimizer_options_(optimizer_options)
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

    // TODO: get paraeters from imu_handler and add them to the backend.
    // backend_.addImu(imu_parameters);
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
