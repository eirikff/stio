#include "svo/gtsam_backend/estimator.hpp"
#include <svo/global.h>
#include <svo/common/imu_calibration.h>
#include <svo/common/point.h>
#include <svo/common/frame.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include <iostream>

namespace svo
{
  namespace gtsam_backend
  {
    Estimator::Estimator()
        : last_preint_factor_bid_(-1)
    {
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
      // if (total_keyframes_count_ == 0)
      // {
      //   VLOG(5) << "Not adding IMU measurements to backend to avoid initial drift.";
      //   return false;
      // }

      // The meas arg is a deque with the newest measurements in front and the oldest
      // in the back. The IMU handler deletes old measurements when they are extracted,
      // but it seems to keep the two first measurements so that in the next call
      // the measurements are all new measurements plus the two from last call. To not
      // add double, we iterate backwards and choose the second element as the previous
      // timestamp, then add all the *actually new* measurements to the preintegrator.
      auto first_it = meas.crbegin() + 1;
      double prev_timestamp = first_it->timestamp_;
      // Iterate backwards because newest measurements are in front of deque
      for (auto it = first_it + 1; it != meas.crend(); it++)
      {
        auto &m = *it;

        double dt = m.timestamp_ - prev_timestamp;
        VLOG(15) << "addImuMeasurements: dt = " << dt;
        prev_timestamp = m.timestamp_;

        preint_->integrateMeasurement(m.linear_acceleration_, m.angular_velocity_, dt);
      }

      return true;
    }

    bool Estimator::addLandmark(const PointPtr &point)
    {
      int id = point->id();

      initial_values_.insert(L(id), point->pos());

      return true;
    }

    bool Estimator::addObservation(const FramePtr &frame, size_t kp_idx)
    {
      PointPtr &p = frame->landmark_vec_[kp_idx];

      gtsam::Point2 meas = frame->px_vec_.col(kp_idx);
      //  TODO: make noise sigma value parameter/option
      auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1);

      graph_.emplace_shared<
          gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, CamParameters::Calibration>>(
          meas, noise, X(frame->bundleId()), L(p->id()), cam_params_->K, cam_params_->T_C_B);

      return true;
    }

    bool Estimator::removePointsByPointIds(const std::vector<int> &pt_ids)
    {

      return false;
    }

    bool Estimator::getSpeedAndBias(const BundleId &bid, SpeedAndBias &speed_and_bias) const
    {
      gtsam::Vector3 speed;
      gtsam::imuBias::ConstantBias bias;
      try
      {
        speed = result_.at<gtsam::Vector3>(V(bid));
        bias = result_.at<gtsam::imuBias::ConstantBias>(B(bid));

        VLOG(6) << "Got speed and bias for bid " << bid << " using results_.";
      }
      catch (const gtsam::ValuesKeyDoesNotExist &)
      {
        try
        {
          speed = predictions_.at<gtsam::Vector3>(V(bid));
          bias = predictions_.at<gtsam::imuBias::ConstantBias>(B(bid));

          VLOG(6) << "Got speed and bias for bid " << bid << " using predictions_.";
        }
        catch (const gtsam::ValuesKeyDoesNotExist &)
        {
          LOG(WARNING) << "Failed to get speed and bias for bid " << bid;
          return false;
        }
      }
      speed_and_bias.segment<3>(0) = speed;
      speed_and_bias.segment<3>(3) = bias.gyroscope();
      speed_and_bias.segment<3>(6) = bias.accelerometer();

      VLOG(6) << "speed_and_bias\n"
              << speed_and_bias;

      return true;
    }

    bool Estimator::getT_WS(const BundleId &bid, Transformation &T_WS) const
    {
      gtsam::Pose3 pose;
      try
      {
        pose = result_.at<gtsam::Pose3>(X(bid));

        VLOG(6) << "Got T_WS for bid " << bid << " using results_.";
      }
      catch (const gtsam::ValuesKeyDoesNotExist &)
      {
        try
        {
          pose = predictions_.at<gtsam::Pose3>(X(bid));

          VLOG(6) << "Got T_WS for bid " << bid << " using predictions_.";
        }
        catch (const gtsam::ValuesKeyDoesNotExist &)
        {
          LOG(WARNING) << "Failed to get T_WS for bid " << bid;
          return false;
        }
      }
      T_WS = Transformation(pose.matrix());
      VLOG(6) << "T_WS\n"
              << T_WS;

      return true;
    }

    bool Estimator::addVelocityPrior(const BundleId &kf_id, const Eigen::Vector3d &value, double sigma)
    {
      auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma);
      graph_.addPrior(V(kf_id), value, noise_model);

      return true;
    }

    BundleId Estimator::optimize()
    {
      if (last_preint_factor_bid_ == -1)
        return false;

      gtsam::LevenbergMarquardtParams param;
      param.setVerbosityLM("SUMMARY");
      gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_values_, param);
      result_ = optimizer.optimize();

      last_optim_state_ = gtsam::NavState(result_.at<gtsam::Pose3>(X(last_preint_factor_bid_)),
                                          result_.at<gtsam::Vector3>(V(last_preint_factor_bid_)));
      last_optim_bias_ = result_.at<gtsam::imuBias::ConstantBias>(B(last_preint_factor_bid_));

      preint_->resetIntegrationAndSetBias(last_optim_bias_);

      return last_preint_factor_bid_;
    }

    bool Estimator::isLandmarkInEstimator(const int id) const
    {
      try
      {
        graph_.at(L(id));
      }
      catch (const std::out_of_range &e)
      {
        return false;
      }

      return true;
    }

    bool Estimator::addPreintFactor(const BundleId &bid, const gtsam::Point3 *const pos_prior)
    {
      if (last_preint_factor_bid_ == -1)
      {
        // this will only be run the first time the function is called
        if (pos_prior)
        {
          gtsam::Pose3 prior(gtsam::Rot3::identity(), *pos_prior);
          last_optim_state_ = gtsam::NavState(prior, gtsam::Vector3::Zero());
          initial_values_.insert(X(bid), prior);
        }

        initial_values_.insert(V(bid), gtsam::Vector3(0, 0, 0));
        initial_values_.insert(B(bid), gtsam::imuBias::ConstantBias());
        last_preint_factor_bid_ = bid;
        return false;
      }

      gtsam::CombinedImuFactor imu_factor(
          X(last_preint_factor_bid_), V(last_preint_factor_bid_),
          X(bid), V(bid),
          B(last_preint_factor_bid_), B(bid),
          *preint_);
      graph_.add(imu_factor);
      last_preint_factor_bid_ = bid;

      gtsam::NavState prop_state = preint_->predict(last_optim_state_, last_optim_bias_);
      initial_values_.insert(X(bid), prop_state.pose());
      initial_values_.insert(V(bid), prop_state.velocity());
      initial_values_.insert(B(bid), last_optim_bias_);

      return true;
    }

    gtsam::NavState Estimator::predictWithImu(const BundleId &bid, double timestamp_s)
    {
      gtsam::NavState pred = preint_->predict(last_optim_state_, last_optim_bias_);
      predictions_.insert(X(bid), pred.pose());
      predictions_.insert(V(bid), pred.velocity());
      predictions_.insert(B(bid), last_optim_bias_);

      last_predict_bid_ = bid;
      bid_timestamp_s_map_.insert({bid, timestamp_s});

      return pred;
    }

    bool Estimator::addExternalPositionPrior(BundleId bid, gtsam::Point3 prior)
    {
      static const std::vector<size_t> indices = {3, 4, 5};
      auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
      gtsam::PartialPriorFactor<gtsam::Pose3> prior_factor(X(bid), indices, prior, noise);
      graph_.add<gtsam::PartialPriorFactor<gtsam::Pose3>>(prior_factor);

      // initial_values_.insert_or_assign(X(bid), prior);

      return true;
    }

  } // namespace gtsam_backend
} // namespace svo
