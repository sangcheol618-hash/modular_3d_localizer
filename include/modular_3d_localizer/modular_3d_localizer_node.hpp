#pragma once

#include <rclcpp/rclcpp.hpp>
#include "modular_3d_localizer/map_loader.hpp"
#include "modular_3d_localizer/point_cloud_preprocessor.hpp"
#include "modular_3d_localizer/scan_processing_controller.hpp"
#include "modular_3d_localizer/local_map_extractor.hpp"
#include "modular_3d_localizer/motion_model/motion_model.hpp"
#include "modular_3d_localizer/motion_model/odometry_motion_model.hpp"
#include "modular_3d_localizer/registration_backend.hpp"
#include "modular_3d_localizer/registration_backend_factory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <atomic>
#include <limits>
#include <mutex>
#include <optional>


namespace modular_3d_localizer
{
    class Modular3DLocalizerNode : public rclcpp::Node
    {
    public:
        Modular3DLocalizerNode();

    private:
        void loadParameters();
        bool initializeMap();
        void setupRosInterfaces();
        void pointCloudCallback(
            const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
        void initialPoseCallback(
            const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg);
        bool lookupBaseToLidarTransform(Eigen::Matrix4f &base_to_lidar);
        bool setCurrentPoseFromBasePose(const geometry_msgs::msg::Pose &base_pose);
        void applyPendingInitialPose();
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
        bool getMotionModelInitialGuess(
            const builtin_interfaces::msg::Time &scan_stamp,
            Eigen::Matrix4f &initial_guess);
        bool updateMapToOdomFromCurrentPose(const builtin_interfaces::msg::Time &stamp);
        void publishOdometryPrediction(const builtin_interfaces::msg::Time &stamp);
        bool isRegistrationJumpWithinLimits(
            const Eigen::Matrix4f &initial_guess,
            const Eigen::Matrix4f &estimated_pose,
            double &translation_jump_meters,
            double &rotation_jump_radians) const;
        void publishDiagnostics(
            const builtin_interfaces::msg::Time &stamp,
            const std::string &reason,
            std::size_t source_point_count = 0,
            std::size_t target_point_count = 0,
            double registration_duration_seconds = 0.0,
            double fitness_score = std::numeric_limits<double>::infinity(),
            double translation_jump_meters = 0.0,
            double rotation_jump_radians = 0.0);
        void publishEstimatedPose(const builtin_interfaces::msg::Time &stamp);
        void publishCandidatePose(
            const Eigen::Matrix4f &candidate_pose_map_lidar,
            const builtin_interfaces::msg::Time &stamp);
        void publishAlignedScan(
            MapLoader::PointCloud::ConstPtr scan_cloud,
            const builtin_interfaces::msg::Time &stamp);
        void publishCandidateAlignedScan(
            MapLoader::PointCloud::ConstPtr scan_cloud,
            const Eigen::Matrix4f &candidate_pose_map_lidar,
            const builtin_interfaces::msg::Time &stamp);
        bool createMapToBasePoseMessage(
            const Eigen::Matrix4f &pose_map_lidar,
            const builtin_interfaces::msg::Time &stamp,
            geometry_msgs::msg::PoseStamped &pose_message);
        void initializeLocalizationBackend();
        void initializeMotionModel();
        std::string map_path_;
        std::string map_topic_;
        std::string map_frame_;
        double map_voxel_leaf_size_{0.0};
        bool auto_adjust_voxel_leaf_size_{false};
        std::string point_cloud_topic_;
        std::string base_frame_;
        std::string lidar_frame_;
        double lidar_voxel_leaf_size_{0.0};
        int scan_queue_depth_{1};
        double max_registration_rate_hz_{0.0};
        std::string localization_backend_;
        std::string initial_pose_topic_;
        std::string estimated_pose_topic_;
        std::string aligned_scan_topic_;
        std::string candidate_pose_topic_;
        std::string candidate_aligned_scan_topic_;
        std::string diagnostics_topic_;
        std::string motion_model_name_;
        std::string odometry_topic_;
        std::string odom_frame_;
        std::string tf_output_mode_;
        GicpRegistrationConfig gicp_config_;
        CoarseToFineGicpRegistrationConfig coarse_to_fine_gicp_config_;
        FastGicpRegistrationConfig fast_gicp_config_;
        bool use_local_map_{true};
        double local_map_radius_{30.0};
        double registration_max_fitness_score_{0.75};
        double registration_max_translation_jump_meters_{2.0};
        double registration_max_rotation_jump_radians_{0.523599};
        double odometry_max_age_seconds_{0.5};
        double odometry_history_duration_seconds_{10.0};
        std::unique_ptr<RegistrationBackend> registration_backend_;
        std::unique_ptr<MotionModel> motion_model_;
        OdometryMotionModel *odometry_motion_model_{nullptr};

        MapLoader map_loader_;
        PointCloudPreprocessor point_cloud_preprocessor_;
        ScanProcessingController scan_processing_controller_;
        LocalMapExtractor local_map_extractor_;


        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_aligned_scan_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr candidate_pose_pub_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        sensor_msgs::msg::PointCloud2::SharedPtr map_msg_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_volatile_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_transient_local_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
        rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
        MapLoader::PointCloud::ConstPtr map_cloud_;

        Eigen::Matrix4f current_pose_map_lidar_ =
        Eigen::Matrix4f::Identity();
        std::optional<Eigen::Matrix4f> map_to_odom_;
        std::mutex pose_mutex_;

        std::atomic<bool> has_valid_pose_{false};
        std::optional<geometry_msgs::msg::Pose> pending_initial_base_pose_;
        std::optional<builtin_interfaces::msg::Time> last_initial_pose_stamp_;


    };

} // namespace modular_3d_localizer
