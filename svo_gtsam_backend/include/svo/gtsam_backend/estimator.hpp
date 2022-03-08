#pragma once

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3DS2.h>

#include <svo/global.h>
#include <svo/vio_common/backend_types.hpp>

namespace svo
{
  namespace gtsam_backend
  {
    // TODO: move definition to it's own types file?
    // PreintegratedCombinedMeasurements work even if the Combined Factor is not used.
    struct ImuParameters : public gtsam::PreintegratedCombinedMeasurements::Params
    {
      using shared_ptr = std::shared_ptr<ImuParameters>;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ImuParameters(const gtsam::Vector3 &n_gravity)
          : gtsam::PreintegratedCombinedMeasurements::Params(n_gravity) {}

      double accel_max = 150; // maximum acceleration in [m/s^2]
      double omega_max = 50;  // maximum angular velocity   in [rad/s]

      double rate = 200;        // imu rate in [Hz]
      double delay_imu_cam = 0; // Camera-IMU delay: delay_imu_cam = cam_timestamp - imu_timestamp [s]
    };

    struct CamParameters
    {
      using shared_ptr = std::shared_ptr<CamParameters>;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // TODO: this calibration definition assumes radial-tangental distortion. change to also support equidistant/fisheye
      gtsam::Cal3DS2::shared_ptr K;
      gtsam::Pose3 T_C_B; // transformation between camera and body frame
    };

  } // namespace gtsam_backend

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

    inline void addImuParams(const gtsam_backend::ImuParameters::shared_ptr params)
    {
      imu_params_ = params;
    }

    void addCamParams(const CameraBundlePtr &camera_bundle);

  protected: // members
    std::shared_ptr<gtsam::ISAM2> isam_;
    std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
    gtsam_backend::ImuParameters::shared_ptr imu_params_;
    gtsam_backend::CamParameters::shared_ptr cam_params_;

  protected: // functions
  };
} // namespace svo
