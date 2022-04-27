#include "svo/gtsam_backend_publisher.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace
{
  template <typename T>
  void normalizeVector(const std::vector<T> &in, std::vector<float> *out)
  {
    auto res = std::minmax_element(in.begin(), in.end());
    const T min = *res.first;
    const T max = *res.second;
    const float dist = static_cast<float>(max - min);

    out->resize(in.size());
    for (size_t i = 0; i < in.size(); i++)
    {
      (*out)[i] = (in[i] - min) / dist;
    }
  }
}

namespace svo
{
  GtsamBackendPublisher::GtsamBackendPublisher(const ros::NodeHandle &nh)
      : nh_(nh)
  {
    pub_imu_pose_viz_ = nh_.advertise<geometry_msgs::PoseStamped>("backend_pose_imu_viz", 10);
    pub_landmarks_ = nh_.advertise<sensor_msgs::PointCloud2>("backend_points", 10);
  }

  void GtsamBackendPublisher::publishImuPose(const ViNodeState &state, const double timestamp_s, const BundleId bid) const
  {
    if (pub_imu_pose_viz_.getNumSubscribers() == 0)
    {
      return;
    }
    VLOG(100) << "Publish IMU Pose for bid = " << bid << " at timestamp " << timestamp_s;

    Eigen::Quaterniond q = state.get_T_W_B().getRotation().toImplementation();
    Eigen::Vector3d p = state.get_T_W_B().getPosition();

    geometry_msgs::PoseStampedPtr msg_pose(new geometry_msgs::PoseStamped);
    msg_pose->header.seq = bid;
    msg_pose->header.stamp.fromSec(timestamp_s);
    msg_pose->header.frame_id = kWorldFrame;
    msg_pose->pose.position.x = p[0];
    msg_pose->pose.position.y = p[1];
    msg_pose->pose.position.z = p[2];
    msg_pose->pose.orientation.x = q.x();
    msg_pose->pose.orientation.y = q.y();
    msg_pose->pose.orientation.z = q.z();
    msg_pose->pose.orientation.w = q.w();

    pub_imu_pose_viz_.publish(msg_pose);
  }

  void GtsamBackendPublisher::publishBackendLandmarks(const gtsam::Values &results, const std::vector<gtsam::Key> lm_keys, const double timestamp_s, const BundleId bid) const
  {
    if (pub_landmarks_.getNumSubscribers() == 0)
    {
      return;
    }
    VLOG(100) << "Publish Backend Landmarks for bid " << bid << " at timestmap " << timestamp_s;

    std::vector<gtsam::Point3> landmarks;
    std::vector<int> landmark_ids;
    for (const gtsam::Key &k : lm_keys)
    {
      try
      {
        gtsam::Point3 p = results.at<gtsam::Point3>(k);
        landmarks.push_back(p);

        gtsam::Symbol s(k);
        landmark_ids.push_back(s.index());
      }
      catch (const gtsam::ValuesKeyDoesNotExist &e)
      {
        VLOG(8) << "Failed to access landmark with key " << gtsam::key_formatter(gtsam::DefaultKeyFormatter) << e.key() << " in publishBackendLandmarks.";
      }
    }

    size_t n_pts = landmarks.size();
    if (n_pts < 5)
    {
      return;
    }

    std::vector<float> intensities;
    normalizeVector(landmark_ids, &intensities);

    // point clound to publish
    pcl::PointCloud<pcl::PointXYZI> pc;
    pc.reserve(n_pts);
    for (size_t i = 0; i < landmarks.size(); i++)
    {
      const auto p = landmarks[i];
      pcl::PointXYZI pt;
      pt.intensity = intensities[i];
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();

      pc.push_back(pt);
    }

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc, pc2);
    pc2.header.seq = bid;
    pc2.header.frame_id = kWorldFrame;
    pc2.header.stamp.fromSec(timestamp_s);

    pub_landmarks_.publish(pc2);
  }
} // namespace svo
