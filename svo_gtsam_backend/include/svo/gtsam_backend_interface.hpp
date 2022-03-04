#pragma once

#include <thread>
#include <condition_variable>
#include <math.h>

#include <svo/abstract_bundle_adjustment.h>
#include <svo/vio_common/backend_types.hpp>

#include <gtsam/geometry/Pose3.h>

namespace svo
{
  // fwd
  struct MotionDetectorOptions;
  class MotionDetector;
  class OutlierRejection;
  class ImuHandler;

  struct GtsamBackendOptions
  {
  };

  struct GtsamBackendInterfaceOptions
  {
  };

  class GtsamBackendInterface : public AbstractBundleAdjustment
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GtsamBackendInterface>;

    GtsamBackendInterfaceOptions options_;
    GtsamBackendOptions optimizer_options_;

    gtsam::Pose3 test_;

    GtsamBackendInterface(const GtsamBackendInterfaceOptions &options,
                          const GtsamBackendOptions &optimizer_options,
                          const MotionDetectorOptions &motion_detector_options,
                          const CameraBundlePtr &camera_bundle);
  };
} // namespace svo
