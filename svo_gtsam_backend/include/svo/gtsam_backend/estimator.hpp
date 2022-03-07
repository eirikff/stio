#pragma once

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuFactor.h>

#include <svo/vio_common/backend_types.hpp>

namespace svo
{
  class Estimator
  {
  public: // members
    // for reinitialization
    bool is_reinit_ = false;
    Eigen::Matrix<double, 9, 1> reinit_speed_bias_;
    Transformation reinit_T_WS_;
    double reinit_timestamp_start_;

  public: // functions
    Estimator()
    {
    }

  protected: // members
    gtsam::ISAM2 isam_;
    gtsam::NonlinearFactorGraph graph_;

  protected: // functions
  };
} // namespace svo
