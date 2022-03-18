#include "svo/gtsam_backend/estimator.hpp"
#include <svo/global.h>
#include <svo/common/imu_calibration.h>
#include <svo/common/point.h>
#include <svo/common/frame.h>

#include <iostream>

namespace svo
{
  namespace gtsam_backend
  {
    Estimator::Estimator(double smoother_lag)
        : prev_kf_bundle_id_(0),
          smoother_lag_(smoother_lag)
    {
      gtsam::ISAM2Params isam_params;
      // isam_ = std::make_shared<gtsam::ISAM2>(isam_params);
      ifl_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(smoother_lag_, isam_params);

      graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
    }

    void Estimator::addInitialPrior()
    {
      // Add origin prior to first pose
      // TODO: load these values from config
      // TODO: load prior noise sigmas from config
      auto pose_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished() // rad, rad, rad, m, m, m
      );
      auto vel_prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
      auto bias_prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

      graph_->addPrior(X(0), gtsam::Pose3(), pose_prior_noise);
      graph_->addPrior(V(0), gtsam::Vector3(), vel_prior_noise);
      graph_->addPrior(B(0), gtsam::imuBias::ConstantBias(), bias_prior_noise);

      initial_estimate_.insert(X(0), gtsam::Pose3());
      initial_estimate_.insert(V(0), gtsam::Vector3());
      initial_estimate_.insert(B(0), gtsam::imuBias::ConstantBias());

      ifl_->update(*graph_, initial_estimate_);
      latest_results_ = ifl_->calculateEstimate();

      graph_->resize(0);
      initial_estimate_.clear();

      VLOG(3) << "Added initial priors to GTSAM backend.";
    }

    void Estimator::addImuParams(const gtsam_backend::ImuParameters::shared_ptr params)
    {
      imu_params_ = params;

      preint_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params->int_param);
    }

    void Estimator::addCamParams(const CameraBundlePtr &camera_bundle)
    {
      if (camera_bundle->getNumCameras() != 1)
      {
        LOG(FATAL) << "GTSAM Backend only supports mono camera setup.";
      }
      if (camera_bundle->getCamera(0).getType() != vk::cameras::Camera::Type::kPinhole)
      {
        LOG(FATAL) << "GTSAM Backend only supports pinhole camera type.";
      }

      // TODO: add check for distortion type and change calibration accordingly
      //      The check might include parsing the printParameters string OR modifying Vikit to include
      //      distortion type as a member variable.
      LOG(WARNING) << "GTSAM Backend assumes radial-tangential distortion as of now. This must be fixed to accomondate equidistant as well.";

      std::stringstream ss;
      camera_bundle->getCamera(0).printParameters(ss);
      std::cout << "output from printParameters:\n\n"
                << ss.str() << "\n\n"
                << std::endl;

      auto K = camera_bundle->getCamera(0).getIntrinsicParameters();
      auto d = camera_bundle->getCamera(0).getDistortionParameters();
      double fx = K(0), fy = K(1), cx = K(2), cy = K(3);
      double k1 = d(0), k2 = d(1), p1 = d(2), p2 = d(3);

      auto T_C_B = camera_bundle->get_T_C_B(0);
      auto quat = T_C_B.getEigenQuaternion();
      auto trans = T_C_B.getPosition();

      cam_params_ = std::make_shared<gtsam_backend::CamParameters>();
      cam_params_->K.reset(new gtsam::Cal3DS2(fx, fy, 0, cx, cy, k1, k2, p1, p2));
      cam_params_->T_C_B = gtsam::Pose3(gtsam::Rot3(quat), trans);

      LOG(INFO) << "Camera parameters loaded to gtsam_backend::Estimator.";
    }

    bool Estimator::addImuMeasurements(const ImuMeasurements &meas)
    {
      static double prev_timestamp = 0;
      for (const ImuMeasurement &m : meas)
      {
        // set the previous timestamp only for the first measurement
        if (prev_timestamp == 0)
        {
          prev_timestamp = m.timestamp_;
          VLOG(6) << "addImuMeasurements: Set previous timestamp to " << prev_timestamp;
        }

        double dt = m.timestamp_ - prev_timestamp;
        prev_timestamp = m.timestamp_;
        VLOG(8) << "addImuMeasurements: dt = " << dt;

        preint_->integrateMeasurement(m.linear_acceleration_, m.angular_velocity_, dt);
      }

      VLOG(6) << "addImuMeasurements: Added " << meas.size() << " measurements to preintegrator.";

      return true;
    }

    bool Estimator::addLandmark(const PointPtr &point)
    {
      int id = point->id();

      initial_estimate_.insert(L(id), point->pos());

      return true;
    }

    bool Estimator::addObservation(const FramePtr &frame, size_t kp_idx)
    {
      PointPtr &p = frame->landmark_vec_[kp_idx];

      gtsam::Point2 meas = frame->px_vec_.col(kp_idx);
      //  TODO: make noise sigma value parameter/option
      auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1);

      graph_->emplace_shared<
          gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, CamParameters::Calibration>>(
          meas, noise, X(frame->bundleId()), L(p->id()), cam_params_->K, cam_params_->T_C_B);

      return true;
    }

    bool Estimator::removePointsByPointIds(const std::vector<int> &pt_ids)
    {

      return false;
    }

    bool Estimator::getSpeedAndBias(const BundleId &kf_id, SpeedAndBias &speed_and_bias) const
    {
      // TODO: the kf_id is not guaranteed to be a keyframe and thus not guaranteed to
      //       be in the factor graph. is it possible to use the imu measurements to
      //       interpolate the speed and bias between two keyframes? could alternatively be
      //       just a simple linear interpolation

      // TODO: measure speed of this and consider if it should be cached somewhere.
      gtsam::Vector3 speed = ifl_->calculateEstimate<gtsam::Vector3>(V(kf_id));
      gtsam::Vector6 bias = ifl_->calculateEstimate<gtsam::Vector6>(B(kf_id));
      speed_and_bias.head<3>(0) = speed;
      speed_and_bias.head<6>(3) = bias;

      return true;
    }

    bool Estimator::getT_WS(const BundleId &kf_id, Transformation &T_WS) const
    {
      // TODO: the kf_id is not guaranteed to be a keyframe and thus not guaranteed to
      //       be in the factor graph. is it possible to use the imu measurements to
      //       interpolate the pose between two keyframes? could alternatively be
      //       just a simple linear interpolation

      // TODO: measure speed of this and consider if it should be cached somewhere.
      gtsam::Pose3 pose = ifl_->calculateEstimate<gtsam::Pose3>(X(kf_id));
      T_WS = Transformation(pose.matrix());

      return true;
    }

    bool Estimator::addVelocityPrior(const BundleId &kf_id, const Eigen::Vector3d &value, double sigma)
    {
      auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma);
      graph_->addPrior(V(kf_id), value, noise_model);

      return true;
    }

    bool Estimator::optimize()
    {

      ifl_->update(*graph_, initial_estimate_, key_timestamp_map_, remove_factors_);
      ifl_->update();

      latest_results_ = ifl_->calculateEstimate();

      graph_->resize(0);
      initial_estimate_.clear();
      remove_factors_.clear();
      preint_->resetIntegrationAndSetBias(prev_bias);

      prev_state = gtsam::NavState(latest_results_.at<gtsam::Pose3>(X(prev_kf_bundle_id_)),
                                   latest_results_.at<gtsam::Vector3>(V(prev_kf_bundle_id_)));
      prev_bias = latest_results_.at<gtsam::imuBias::ConstantBias>(B(prev_kf_bundle_id_));

      return true;
    }

    bool Estimator::isLandmarkInEstimator(const int id) const
    {
      try
      {
        ifl_->getFactors().at(L(id));
      }
      catch (const std::out_of_range &e)
      {
        return false;
      }

      return true;
    }

    bool Estimator::addPreintFactor(const BundleId &kf_id)
    {
      gtsam::CombinedImuFactor imu_factor(
          X(prev_kf_bundle_id_), V(prev_kf_bundle_id_),
          X(kf_id), V(kf_id),
          B(prev_kf_bundle_id_), B(kf_id),
          *preint_);
      graph_->add(imu_factor);
      prev_kf_bundle_id_ = kf_id;

      gtsam::NavState prop_state = preint_->predict(prev_state, prev_bias);
      initial_estimate_.insert(X(kf_id), prop_state.pose());
      initial_estimate_.insert(V(kf_id), prop_state.velocity());
      initial_estimate_.insert(B(kf_id), prev_bias);

      return true;
    }

  } // namespace gtsam_backend
} // namespace svo
