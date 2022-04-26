#pragma once

#include <memory>
#include <ros/ros.h>
#include <svo/vio_common/backend_types.hpp>
#include <gtsam/nonlinear/Values.h>

namespace svo
{
  class GtsamBackendPublisher
  {
  public: // members
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GtsamBackendPublisher>;
    const std::string kWorldFrame = "world";

  public: // functions
    GtsamBackendPublisher(const ros::NodeHandle &nh);

    void publish(const ViNodeState &state, const gtsam::Values &results, const std::vector<gtsam::Key> lm_keys, const double timestamp_s, const BundleId bid) const
    {
      publishImuPose(state, timestamp_s, bid);
      publishBackendLandmarks(results, lm_keys, timestamp_s, bid);
    }

  private: // members
    ros::NodeHandle nh_;

    ros::Publisher pub_imu_pose_viz_;
    ros::Publisher pub_landmarks_;

  private: // functions
    void publishImuPose(const ViNodeState &state, const double timestamp_s, const BundleId bid) const;
    void publishBackendLandmarks(const gtsam::Values &results, const std::vector<gtsam::Key> lm_keys, const double timestamp_s, const BundleId bid) const;
  };
} // namespace svo
