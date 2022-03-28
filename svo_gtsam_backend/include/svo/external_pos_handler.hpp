#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/geometry/Point3.h>

namespace svo
{

  // MsgType is ROS message type, e.g. geometry_msgs/PointStamped or nav_msgs/Odometry
  template <typename MsgType>
  class ExternalPositionHandler
  {
  public: // typedefs
    using Point3 = gtsam::Point3;

  public: // functions
    ExternalPositionHandler() = default;

    /**
     * @brief Get the position as timestamp @arg timestamp, interpolating if possible
     * if @arg timestamp is in-between two messages, or returning the latest position
     * if @arg timestamp is newer than the latest position.
     *
     * @param timestamp Timestamp for when to get the external prior.
     * @return std::pair<double, Point3> first = timestamp of returned position, second = returned position.
     */
    std::pair<double, Point3> getPosition(double timestamp) const
    {
      CHECK(!messages_.empty()) << "No position messages has been added.";

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

      const Point3 &p_before = before.second;
      const Point3 &p_after = after.second;
      const double &t_before = before.first;
      const double &t_after = after.first;
      double dt = t_after - t_before;
      double t = (timestamp - t_before) / dt;
      CHECK(t >= 0 && t <= 1) << "Interpolation parameter t = " << t << " is not in range [0, 1].";
      Point3 interp = gtsam::interpolate(p_before, p_after, t);
      return {timestamp, interp};
    }

    inline void addPosition(const boost::shared_ptr<const MsgType> &msg)
    {
      Point3 p = getPosition(msg);
      double ts = msg->header.stamp.toSec();

      messages_.insert({ts, p});
    }

  public:    // member
  protected: // functions
    inline Point3 getPosition(const geometry_msgs::PointStamped::ConstPtr msg) const
    {
      auto p = msg->point;
      return gtsam::Point3(p.x, p.y, p.z);
    }
    inline Point3 getPosition(const nav_msgs::Odometry::ConstPtr msg) const
    {
      auto p = msg->pose.pose.position;
      return gtsam::Point3(p.x, p.y, p.z);
    }

  protected:                            // members
    std::map<double, Point3> messages_; // key is timestamp in seconds
  };

} // namespace svo