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

    test_.compose(test_);

    LOG(INFO) << "Initialized GtsamBackendInterface";
  }
}
