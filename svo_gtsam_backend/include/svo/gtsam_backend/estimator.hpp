#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/inference/Symbol.h>

#include <svo/global.h>
#include <svo/vio_common/backend_types.hpp>
#include <svo/common/imu_calibration.h>

namespace svo
{
  namespace gtsam_backend
  {
    // TODO: move definition to it's own types file?
    // PreintegratedCombinedMeasurements work even if the Combined Factor is not used.
    struct ImuParameters
    {
      using shared_ptr = std::shared_ptr<ImuParameters>;
      using IntegrationParams = gtsam::PreintegratedCombinedMeasurements::Params;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ImuParameters(const gtsam::Vector3 &n_gravity)
      {
        int_param = boost::make_shared<IntegrationParams>(n_gravity);
      }

      boost::shared_ptr<IntegrationParams> int_param;

      double accel_max = 150; // maximum acceleration in [m/s^2]
      double omega_max = 50;  // maximum angular velocity   in [rad/s]

      double rate = 200;        // imu rate in [Hz]
      double delay_imu_cam = 0; // Camera-IMU delay: delay_imu_cam = cam_timestamp - imu_timestamp [s]
    };

    struct CamParameters
    {
      using shared_ptr = std::shared_ptr<CamParameters>;
      using Calibration = gtsam::Cal3DS2;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // TODO: this calibration definition assumes radial-tangental distortion. change to also support equidistant/fisheye
      Calibration::shared_ptr K;
      gtsam::Pose3 T_C_B; // transformation between camera and body frame
    };

    // [velocity, gyro biases, accel biases]
    using SpeedAndBias = Eigen::Matrix<double, 9, 1>;

    using gtsam::symbol_shorthand::B; // bias (accelerometer + gyroscope)
    using gtsam::symbol_shorthand::L; // landmark
    using gtsam::symbol_shorthand::V; // velocity
    using gtsam::symbol_shorthand::X; // pose

    class Estimator
    {
    public: // members
      // for reinitialization
      bool is_reinit_ = false;
      Eigen::Matrix<double, 9, 1> reinit_speed_bias_;
      Transformation reinit_T_WS_;
      double reinit_timestamp_start_;

    public: // functions
      Estimator();

      void addImuParams(const gtsam_backend::ImuParameters::shared_ptr params);

      void addCamParams(const CameraBundlePtr &camera_bundle);

      /**
       * @brief Add the imu measurements to the preintegrator.
       *
       * @param meas The IMU measurements to add.
       * @return true on success.
       * @return false on failure.
       */
      bool addImuMeasurements(const ImuMeasurements &meas);

      /**
       * @brief Add new landmark to the factor graph if it don't already exists.
       *
       * @param point Point to add.
       * @return true on success.
       * @return false on failure.
       */
      bool addLandmark(const PointPtr &point);

      /**
       * @brief Add new landmark observation, i.e. reprojection factors between
       * frame node and landmark node.
       *
       * @param frame Frame with landmark oberservation to add.
       * @param kp_idx Index of keypoint to add.
       * @return true on success.
       * @return false on failure. (e.g. if the frame id or landmark id is not a node)
       */
      bool addObservation(const FramePtr &frame, size_t kp_idx);

      /**
       * @brief Removes landmark observation from factor graph.
       *
       * @param pt_ids Ids of points to remove.
       * @return true on success.
       * @return false on failure.
       */
      bool removePointsByPointIds(const std::vector<int> &pt_ids);

      /**
       * @brief Get the speed and bias estimates from the backend.
       *
       * @param bid Bundle id to get value for.
       * @param speed_and_bias Speed and bias estimate output.
       * @return true on success.
       * @return false on failure.
       */
      bool getSpeedAndBias(const BundleId &kf_id, SpeedAndBias &speed_and_bias) const;

      /**
       * @brief Get the pose T_WS (world to sensor/imu) at node for frame with
       * bundle id bid.
       *
       * @param bid Bundle id to get value for.
       * @param T_WS Output variable for result.
       * @return true on success.
       * @return false on failure.
       */
      bool getT_WS(const BundleId &bid, Transformation &T_WS) const;

      /**
       * @brief Add velocity prior to node of keyframe with id kf_id.
       *
       * @param kf_id Id of keyframe.
       * @param value Prior value.
       * @param sigma Standard deviation/uncertainty of prior.
       * @return true on success.
       * @return false on failure.
       */
      bool addVelocityPrior(const BundleId &kf_id, const Eigen::Vector3d &value, double sigma);

      /**
       * @brief
       *
       * @return BundleId The bundle id of the last optimized factor
       */
      BundleId optimize();

      /**
       * @brief Checks if a landmark with id is in the iSAM2 estimator.
       *
       * @param id Id of landmark/point to check.
       * @return true if it is in the estimator.
       * @return false if it is not in the estimator.
       */
      bool isLandmarkInEstimator(const int id) const;

      /**
       * @brief
       *
       * @param bid
       * @return true
       * @return false
       */
      bool addPreintFactor(const BundleId &bid, const gtsam::Pose3 *const prior = nullptr);

      /**
       * @brief Predicts using the IMU preintegration and associate the prediction
       * with the bundle id.
       *
       * @param bid Bundle ID to associate prediciton with.
       * @param timestamp_s Timestamp in seconds of bundle with bundle id bid.
       * @return gtsam::NavState Prediction.
       */
      gtsam::NavState predictWithImu(const BundleId &bid, double timestamp_s);

      /**
       * @brief Get the number of frames processed by the backend. A frame has been
       * processed if the IMU measurements up to that frame are preintegrated and
       * a prediction for it's bundle id is calculated.
       *
       * @return int
       */
      inline int getNumFrames() const { return predictions_.keys().size() / 3; }

      inline BundleId getLastAddedBid() const { return last_predict_bid_; }

      inline double getTimestampSeconds(BundleId bid) const { return bid_timestamp_s_map_.at(bid); }

      inline void increaseTotalKeyframeCount() { total_keyframes_count_++; }

      bool addExternalPosePrior(BundleId bid, gtsam::Pose3 prior);

    protected: // members
      gtsam::NonlinearFactorGraph graph_;
      gtsam_backend::ImuParameters::shared_ptr imu_params_;
      gtsam_backend::CamParameters::shared_ptr cam_params_;
      std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;
      gtsam::Values initial_values_; // initial values to give to optimzer
      gtsam::Values result_;         // results from last optimization
      gtsam::Values predictions_;    // predictions for every frame using imu preintegration

      std::map<BundleId, double> bid_timestamp_s_map_;
      size_t total_keyframes_count_ = 0;

      // for preintegration factor
      BundleId last_preint_factor_bid_ = -1;
      BundleId last_predict_bid_;
      gtsam::NavState last_optim_state_;
      gtsam::imuBias::ConstantBias last_optim_bias_;

    protected: // functions
    };
  } // namespace gtsam_backend

} // namespace svo
