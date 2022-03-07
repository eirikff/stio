#pragma once

#include <thread>
#include <condition_variable>
#include <deque>
#include <math.h>
#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>

#include <svo/abstract_bundle_adjustment.h>
#include <svo/vio_common/backend_types.hpp>
#include <svo/motion_detector.hpp>
#include <svo/outlier_rejection.hpp>

#include "svo/gtsam_backend/estimator.hpp"

namespace svo
{
  // fwd
  struct MotionDetectorOptions;
  class MotionDetector;
  class OutlierRejection;
  class ImuHandler;

  struct GtsamBackendInterfaceOptions
  {
    /// (!) use the zero motion detection?
    bool use_zero_motion_detection = true;

    /// (!) After how many non moving frames should we introduce zero motion prior
    size_t backend_zero_motion_check_n_frames = 5;

    /// (!) use the outlier rejection module
    bool use_outlier_rejection = true;

    /// (!) This parameter is the reprojection error threshold after
    /// optimization. If the distance between a feature and the projected pixel
    /// position of the corresponding 3D point is further than this threshold
    /// appart (on the zero'th level pyramid), then the feature is removed from
    /// the frame. With a good camera and image resolution of 640x480 a threshold
    /// of 2.0 is typically ok. If you use shitty cameras (rolling shutter),
    /// higher resolution cameras, cameras with imperfect calibration etc. you
    /// might increase this threshold. We made the experice that with GoPro
    /// cameras, we had to increase this threshold.
    double outlier_rejection_px_threshold = 2.0;
  };

  struct GtsamBackendOptions
  {
  };

  class GtsamBackendInterface : public AbstractBundleAdjustment
  {
  public: // members
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GtsamBackendInterface>;

    GtsamBackendInterfaceOptions options_;
    GtsamBackendOptions optimizer_options_;

  public: // functions
    GtsamBackendInterface(const GtsamBackendInterfaceOptions &options,
                          const GtsamBackendOptions &optimizer_options,
                          const MotionDetectorOptions &motion_detector_options,
                          const CameraBundlePtr &camera_bundle);

    /**
     * @brief This is called BEFORE frame is processed in frontend
     * @param new_frames Pose prior is written into transformation
     * @param last_frames Optimized pose is written into transformation
     * @param map All keyframe poses and landmark positions are updated
     * @param have_motion_prior flags if we succesfully obtained pose prior
     *        for new frames
     */
    void loadMapFromBundleAdjustment(const FrameBundlePtr &new_frames,
                                     const FrameBundlePtr &last_frames,
                                     const MapPtr &map,
                                     bool &have_motion_prior) override;

    /**
     * @brief This is called AFTER frame_bundle was processed by the frontend.
     *        All landmark observations of the frame bundle are added to backend.
     *        If this is a keyframe new landmarks are introduced, and the frame
     *        bundle is set to a keyframe in the backend.
     * @param[in] frame_bundle_vec Frame bundles that were already processed by
     *            the frontend.
     */
    void bundleAdjustment(const FrameBundlePtr &frame_bundle) override;

    /**
     * @brief Reset the backend: NOT IMPLEMENTED
     */
    void reset() override;

    /**
     * @brief Start the optimization thread.
     */
    void startThread() override;

    /**
     * @brief Stop and join optimization thread.
     */
    void quitThread() override;

    // set correction transformation to be applied
    inline void setCorrectionInWorld(const Transformation & /*w_T_correction*/) override
    {
      LOG(WARNING) << "Functionality related to loop closnig not supported in GTSAM backend.";
    }

    void getAllActiveKeyframes(std::vector<FramePtr> *keyframes) override;

    /**
     * @brief Set the IMU in backend
     * @param imu_handler Shared ImuHandler with frontend
     */
    void setImuHandler(const std::shared_ptr<ImuHandler> imu_handler);

    /**
     * @brief Get the pose and speed bias of the IMU as per the latest IMU frame
     * @param[in] None
     * @param[out] The speed bias and pose of the latest imu frame
     */
    void getLatestSpeedBiasPose(Eigen::Matrix<double, 9, 1> *speed_bias,
                                Transformation *T_WS,
                                double *timestamp) const override;

    void setReinitStartValues(const Eigen::Matrix<double, 9, 1> &sb,
                              const Transformation &Tws,
                              const double timestamp) override;

    /**
     * @brief getNumFrames returns the number of frames in backend
     * @return
     */
    inline int getNumFrames() const override
    {
      // return static_cast<int>(backend_.numFrames());
      LOG(FATAL) << "Not implemented yet.";
    }

    inline bool isFixedToGlobalMap() const override
    {
      LOG(WARNING) << "Functionality related to global map not supported in GTSAM backend.";
      // returning false is safe because then nothing special is done.
      return false;
    }

    inline BundleId lastOptimizedBundleId() const override
    {
      return last_optimized_nframe_.load();
    }

    inline void getLastState(ViNodeState *state) const override
    {
      *state = last_state_;
    }

  protected: // members
    // modules
    Estimator backend_;
    std::shared_ptr<ImuHandler> imu_handler_;
    std::unique_ptr<MotionDetector> motion_detector_;
    size_t no_motion_counter_;
    std::unique_ptr<OutlierRejection> outlier_rejection_;

    // Threading
    mutable std::condition_variable wait_condition_;
    mutable std::mutex mutex_backend_;
    std::unique_ptr<std::thread> thread_;
    std::atomic_bool stop_thread_{false};

    // state
    // bundle id for which the IMU messages are added
    BundleId last_added_nframe_imu_ = -1;
    // bundle id for which the images and observations are added (after frontend)
    BundleId last_added_nframe_images_ = -1;
    // book keeping for the time
    int64_t last_added_frame_stamp_ns_ = 0;
    // the bundle id for which the optimized states have been
    // updated in the fronend
    BundleId last_updated_nframe_ = -1;

    // the bundle id for which the backend has finished optimization
    std::atomic<BundleId> last_optimized_nframe_{-1};

    // variables for handling optimization choices
    bool skip_optimization_once_ = false;

    /// keyframes that are contained in backend. Oldest keyframe in front
    std::deque<FramePtr> active_keyframes_;

    // visualization
    ViNodeState last_state_;
    // CeresBackendPublisher::Ptr publisher_;

    // fixation
    bool image_motion_detector_stationary_ = false;
    bool imu_motion_detector_stationary_ = false;

  protected: // functions
    /**
     * @brief Loop of optimization thread
     */
    void optimizationLoop();
  };
} // namespace svo
