#include "svo/gtsam_backend/estimator.hpp"
#include <svo/global.h>
#include <svo/common/imu_calibration.h>
#include <svo/common/point.h>
#include <svo/common/frame.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include <iostream>
#include <algorithm>

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

      // TODO: this needs to be a parameter or estimated from data
      // FOR EUROC VICON ROOM (T_imu_vicon = T_sensor_body)
      gtsam::Rot3 rot(0.33638, -0.01749, 0.94156,
                      -0.02078, -0.99972, -0.01114,
                      0.94150, -0.01582, -0.33665);
      gtsam::Point3 trans(0.06901, -0.02781, -0.12395);
      gtsam::Pose3 T_imu_vicon(rot, trans);
      params->int_param->body_P_sensor = T_imu_vicon.inverse();

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

      auto T_C_B_vk = camera_bundle->get_T_C_B(0);
      auto T_C_B_gtsam = gtsam::Pose3(gtsam::Rot3(T_C_B_vk.getEigenQuaternion()), T_C_B_vk.getPosition());

      // since body is redefined to be vicon marker and not imu, need to
      // find transform between body (vicon) and camera instead of imu and camera.
      cam_params_ = std::make_shared<gtsam_backend::CamParameters>();
      cam_params_->K.reset(new gtsam::Cal3DS2(fx, fy, 0, cx, cy, k1, k2, p1, p2));
      cam_params_->T_body_sensor = T_C_B_gtsam.inverse();

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

    bool Estimator::addProjectionFactors(const PointPtr &landmark)
    {
      //  TODO: make noise sigma value parameter/option
      auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1); // px

      for (auto &obsv : landmark->obs_)
      {
        BundleId bid = obsv.frame_id; // frame_id = bundle id of frame the obsv is from
        int lm_id = landmark->id();   // id is the point's/landmark's unique id

        if (!isObservationInBackend(bid, lm_id))
        {
          if (auto f = obsv.frame.lock()) // gets shared_ptr of frame and checks if it's not nullptr
          {
            // keypoint_index is the index of the landmark in the frame's landmark vector
            gtsam::Point2 meas = f->px_vec_.col(obsv.keypoint_index_);

            graph_.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, CamParameters::Calibration>>(
                meas, noise, X(bid), L(lm_id), cam_params_->K, false, true, cam_params_->T_body_sensor);

            addObservationToObservationMap(bid, lm_id);

            if (!initial_values_.exists(L(lm_id)))
            {
              initial_values_.insert(L(lm_id), landmark->pos());
            }
          }
          else
          {
            LOG(WARNING) << "Frame with bid = " << bid << " is nullptr in addObservation.";
          }
        }
      }

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

      VLOG(6) << std::setfill(' ') << "\nspeed_and_bias:\n"
              << "  vel = " << speed.transpose() << "\n"
              << "  gyr = " << bias.gyroscope().transpose() << "\n"
              << "  acc = " << bias.accelerometer().transpose() << std::endl;

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

      VLOG(6) << std::setfill(' ') << "\nT_WS:\n"
              << "  pos = " << pose.translation().transpose() << "\n"
              << "  rpy = " << pose.rotation().rpy().transpose() << std::endl;

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
      // graph_.print("FACTOR GRAPH: ");
      // initial_values_.print("INITIAL VALUES: ");

      if (last_preint_factor_bid_ == -1)
        return false;

      gtsam::LevenbergMarquardtParams param;
      param.setVerbosityLM("SUMMARY");
      gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_values_, param);
      result_ = optimizer.optimize();
      std::cout << std::endl;
      initial_values_ = result_;

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

    bool Estimator::addPreintFactor(const BundleId &bid, const gtsam::Pose3 *const prior)
    {
      if (last_preint_factor_bid_ == -1)
      {
        // this will only be run the first time the function is called

        gtsam::Pose3 pose_prior;
        // TODO: velocity prior and bias prior values should come from parameters
        gtsam::Vector3 velocity_prior(0, 0, 0);
        gtsam::imuBias::ConstantBias bias_prior(gtsam::Vector6::Zero());

        if (prior)
        {
          pose_prior = *prior;
        }

        last_optim_state_ = gtsam::NavState(pose_prior, velocity_prior);
        initial_values_.insert(X(bid), pose_prior);
        initial_values_.insert(V(bid), velocity_prior);
        initial_values_.insert(B(bid), bias_prior);

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

    bool Estimator::addExternalPosePrior(BundleId bid, gtsam::Pose3 prior)
    {
      if (!use_external_prior_)
      {
        VLOG(5) << "In addExternalPosePrior: use_external_prior is set to false, not adding prior.";
        return false;
      }

      // when we only have position available as priors
      // TODO: figure out how/if to handle this
      // static const std::vector<size_t> indices = {3, 4, 5};
      // auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-6);
      // gtsam::PartialPriorFactor<gtsam::Pose3> prior_factor(X(bid), indices, prior, noise);
      // graph_.add<gtsam::PartialPriorFactor<gtsam::Pose3>>(prior_factor);

      // when we have position and orientation available as priors
      auto noise = gtsam::noiseModel::Isotropic::Sigmas(
          (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3).finished() // rad, rad, rad, m, m, m
      );
      graph_.addPrior(X(bid), prior, noise);

      static bool first_ever_prior = true; // TODO: this could rather be handled by using one of the counters
      if (first_ever_prior)
      {
        first_ever_prior = false;

        // TODO: these values, both priors and sigmas, should come from parameters
        gtsam::Vector3 velocity_prior(0, 0, 0);
        gtsam::imuBias::ConstantBias bias_prior(gtsam::Vector6::Zero());
        auto velocity_prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
        auto bias_prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

        graph_.addPrior(V(bid), velocity_prior, velocity_prior_noise);
        graph_.addPrior(B(bid), bias_prior, bias_prior_noise);

        // since this sets the priors for x0 we need to reset the preintegration so successive
        // calls calculates the preintegration between x0 and x1, not "x-1" and x1 (i.e. from before
        // the very first variable)
        preint_->resetIntegrationAndSetBias(bias_prior);

        VLOG(2) << "Added initial zero velocity and bias priors for bid " << bid << " to constrain problem.";
      }

      return true;
    }

    bool Estimator::isObservationInBackend(BundleId bid, int landmark_id) const
    {
      try
      {
        auto &obsv = landmark_obsv_states_.at(L(landmark_id));

        if (std::find(obsv.cbegin(), obsv.cend(), X(bid)) != obsv.cend())
        {
          return true;
        }

        return false;
      }
      catch (const std::out_of_range &)
      {
        return false;
      }

      return false;
    }

  } // namespace gtsam_backend
} // namespace svo
