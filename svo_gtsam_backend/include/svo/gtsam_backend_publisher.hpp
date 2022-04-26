#pragma once

#include <memory>
#include <ros/ros.h>
#include <svo/vio_common/backend_types.hpp>

namespace svo
{
  class GtsamBackendPublisher
  {
  public: // members
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GtsamBackendPublisher>;

  public: // functions
    GtsamBackendPublisher(const ros::NodeHandle &nh)
        : nh_(nh)
    {
    }

    void publish() const {}

  private: // members
    ros::NodeHandle nh_;

  private: // functions
  };
}
