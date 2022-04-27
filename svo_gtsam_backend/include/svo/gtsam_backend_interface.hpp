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
#include "svo/external_pose_handler.hpp"
#include "svo/gtsam_backend_publisher.hpp"

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

    // TODO: what does these sigma parameters *actually* mean?
    /// Sigma for the uncertainty in the integration.
    double preintegration_sigma = 1e-4;
    /// Sigma for the bias used for pre-integration.
    double bias_preintegration_sigma = 1e-4;

    /// Minimum number of observations a landmark must have to be added to the
    /// backend.
    size_t min_obsv_count = 2;
  };

  struct GtsamBackendOptions
  {
    // If true, use the PreintegratedCombinedMeasurement factor instead
    // of PreintegraedMeasurement and Randon walk factors.
    bool use_combined_factor = true;
  };

  class GtsamBackendInterface : public AbstractBundleAdjustment
  {
  public: // members
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GtsamBackendInterface>;

    GtsamBackendInterfaceOptions options_;
    GtsamBackendOptions backend_options_;

  public: // functions
    GtsamBackendInterface(
        ros::NodeHandle &nh,
        const GtsamBackendInterfaceOptions &options,
        const GtsamBackendOptions &backend_options,
        const MotionDetectorOptions &motion_detector_options,
        const CameraBundlePtr &camera_bundle);

    ~GtsamBackendInterface();

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

    void makePublisher(const ros::NodeHandle &nh)
    {
      publisher_ = std::make_shared<GtsamBackendPublisher>(nh);
    }

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

    /**
     * @brief Set the start values to use when reinitializing the backend.
     *
     * @param sb Speed and biases
     * @param Tws Transform world imu
     * @param timestamp Timestamp
     */
    void setReinitStartValues(const Eigen::Matrix<double, 9, 1> &sb,
                              const Transformation &Tws,
                              const double timestamp) override;

    /**
     * @brief getNumFrames returns the number of frames in backend. This is not the
     * same as number of keyframes.
     *
     * @return
     */
    inline int getNumFrames() const override
    {
      return backend_.getNumFrames();
    }

    inline bool isFixedToGlobalMap() const override
    {
      LOG(WARNING) << "Functionality related to global map not supported in GTSAM backend.";
      // returning false is safe because then nothing special is done.
      return false;
    }

    inline BundleId lastOptimizedBundleId() const override
    {
      return last_optimized_bid_;
    }

    inline void getLastState(ViNodeState *state) const override
    {
      *state = last_state_;
    }

    inline void setPerformanceMonitor(const std::string & /* trace_dir */) override
    {
      LOG(ERROR) << "GtsamBackendInterface::setPerformanceMonitor is not implemented.";
    }

    inline void startTimer(BundleId /* bundle_id */) override
    {
      LOG(ERROR) << "GtsamBackendInterface::startTimer is not implemented.";
    }

    template <typename MsgType>
    inline void addExternalPoseMessage(const boost::shared_ptr<MsgType> &msg)
    {
      ext_pose_handler_.addPose(msg);
    }

    bool isInitializedWithExternalPrior(double time_threshold = 5.0) const override;

    void setUseExternalPrior(bool b) override
    {
      AbstractBundleAdjustment::setUseExternalPrior(b);
      backend_.setUseExternalPrior(b);
    }

  protected: // members
    // modules
    gtsam_backend::Estimator backend_;
    std::shared_ptr<ImuHandler> imu_handler_;
    std::unique_ptr<MotionDetector> motion_detector_;
    size_t no_motion_counter_;
    std::unique_ptr<OutlierRejection> outlier_rejection_;
    ExternalPoseHandler<geometry_msgs::TransformStamped> ext_pose_handler_; // TODO: make template type depend on parameters

    // Threading
    mutable std::condition_variable wait_condition_;
    mutable std::mutex mutex_backend_;
    std::unique_ptr<std::thread> thread_;
    std::atomic_bool stop_thread_{false};
    bool do_optimize_ = false;

    BundleId last_optimized_bid_ = -1;
    BundleId last_added_factor_bid_ = -1;

    bool is_frontend_initialized_ = false;

    // variables for handling optimization choices
    bool skip_optimization_once_ = false;

    /// keyframes that are contained in backend. Oldest keyframe in front
    std::deque<FramePtr> active_keyframes_;

    ImuMeasurements latest_imu_meas_;

    std::vector<PointPtr> active_landmarks_;

    // visualization
    ViNodeState last_state_;
    GtsamBackendPublisher::Ptr publisher_;

    // fixation
    bool image_motion_detector_stationary_ = false;
    bool imu_motion_detector_stationary_ = false;

  protected: // functions
    /**
     * @brief Loop of optimization thread
     */
    void optimizationLoop();

    /**
     * @brief Get the latest IMU measurements from imu_handler_ up to the timestamp
     * and save them to latest_imu_meas_.
     *
     * @param timestamp_s Timestamp of frame in seconds.
     * @return true On success.
     * @return false On failure.
     */
    bool getImuMeasurements(double timestamp_s);

    /**
     * @brief Get the latest estimate from the backend and set the frame bundle's
     * T_W_B, speed, and bias members.
     *
     * @param new_frames New frames.
     * @return true On success.
     * @return false On failure.
     */
    bool updateBundleStateWithBackend(const FrameBundlePtr &new_frames);

    /**
     * @brief Get the latest estimate from the backend and set the single frame's
     * T_W_B. If update_speed_bias = true, also update the speed and bias estimates.
     *
     * @param frame The frame.
     * @param update_speed_bias Whether or not to also update speed and bias. This
     * is a parameter for speed reasons.
     * @return true On success.
     * @return false On failure.
     */
    bool updateFrameStateWithBackend(const FramePtr &frame, bool update_speed_bias);

    /**
     * @brief Add landmarks that are not already in the factor graph backend. Also
     * add observations of landmarks seen in the frame.
     *
     * @param frame Frame to be processed.
     * @return true On success.
     * @return false On failure.
     */
    bool addLandmarksAndObservationsToBackend(const FramePtr &frame);

    bool updateAllActiveLandmarks();
  };
} // namespace svo
