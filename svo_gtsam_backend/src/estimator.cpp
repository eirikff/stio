#include "svo/gtsam_backend/estimator.hpp"
#include <svo/global.h>

#include <iostream>

namespace svo
{

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
} // namespace svo
