#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/geometry/Point3.h>

namespace svo
{

  // MsgType is ROS message type. Currently supported types are:
  // - geometry_msgs/TransformStamped
  // - geometry_msgs/PointStamped
  // - nav_msgs/Odometry
  template <typename MsgType>
  class ExternalPoseHandler
  {
  public: // functions
    ExternalPoseHandler() = default;

    /**
     * @brief Get the position as timestamp @arg timestamp, interpolating if possible
     * if @arg timestamp is in-between two messages, or returning the latest position
     * if @arg timestamp is newer than the latest position.
     *
     * @param timestamp Timestamp for when to get the external prior.
     * @return std::pair<double, Pose3> first = timestamp of returned position, second = returned position.
     */
    std::pair<double, gtsam::Pose3> getPose(double timestamp) const
    {
      CHECK(!messages_.empty()) << "No pose messages has been added.";

      auto it = messages_.upper_bound(timestamp);

      if (it == messages_.end())
      {
        // no element with timestamp greater than or equal to @arg timestamp, so
        // return latest element
        return *(--it);
      }

      if (messages_.size() == 1)
      {
        // only a single element, return it as it's the best guess
        return *it;
      }

      auto after = *it;
      auto before = *(--it);

      const gtsam::Pose3 &p_before = before.second;
      const gtsam::Pose3 &p_after = after.second;
      const double &t_before = before.first;
      const double &t_after = after.first;
      double dt = t_after - t_before;
      double t = (timestamp - t_before) / dt;
      CHECK(t >= 0 && t <= 1) << "Interpolation parameter t = " << t << " is not in range [0, 1].";
      gtsam::Pose3 interp = gtsam::interpolate(p_before, p_after, t);
      return {timestamp, interp};
    }

    inline void addPose(const boost::shared_ptr<const MsgType> &msg)
    {
      gtsam::Pose3 p = getPose(msg);
      double ts = msg->header.stamp.toSec();

      messages_.insert({ts, p});
    }

  public:    // member
  protected: // functions
    inline gtsam::Pose3 getPose(const geometry_msgs::PointStamped::ConstPtr msg) const
    {
      // TODO: how to best handle no orientation information in prior? set rotation to identity
      //       or indicate some other way (e.g. nullptr, etc.)?
      auto p = msg->point;
      return gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(p.x, p.y, p.z));
    }

    inline gtsam::Pose3 getPose(const nav_msgs::Odometry::ConstPtr msg) const
    {
      auto &pos = msg->pose.pose.position;
      auto &ori = msg->pose.pose.orientation;

      return gtsam::Pose3(gtsam::Quaternion(ori.w, ori.x, ori.y, ori.z), gtsam::Point3(pos.x, pos.y, pos.z));
    }

    inline gtsam::Pose3 getPose(const geometry_msgs::TransformStamped::ConstPtr msg) const
    {
      auto &pos = msg->transform.translation;
      auto &ori = msg->transform.rotation;

      return gtsam::Pose3(gtsam::Quaternion(ori.w, ori.x, ori.y, ori.z), gtsam::Point3(pos.x, pos.y, pos.z));
    }

  protected:                                  // members
    std::map<double, gtsam::Pose3> messages_; // key is timestamp in seconds
  };

} // namespace svo