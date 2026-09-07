#include "modular_3d_localizer/modular_3d_localizer_node.hpp"
#include "modular_3d_localizer/motion_model/motion_model_factory.hpp"

#include <functional>
#include <chrono>
#include <algorithm>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <tf2/exceptions.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace modular_3d_localizer
{

    Modular3DLocalizerNode::Modular3DLocalizerNode()
    : Node("modular_3d_localizer_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Modular 3D Localizer Node has been started.");

        loadParameters();

        if (!initializeMap())
        {
            return;
        }

        initializeLocalizationBackend();
        initializeMotionModel();
        setupRosInterfaces();

    }

    void Modular3DLocalizerNode::loadParameters()
    {
        this->declare_parameter<std::string>("map_path", "");
        this->declare_parameter<std::string>("map_topic", "map");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("map_voxel_leaf_size", 0.2);
        this->declare_parameter<bool>("auto_adjust_voxel_leaf_size", false);
        this->declare_parameter<std::string>("point_cloud_topic", "");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("lidar_frame", "lidar");
        this->declare_parameter<double>("lidar_voxel_leaf_size", 0.2);
        this->declare_parameter<int>("scan_queue_depth", 1);
        this->declare_parameter<double>("max_registration_rate_hz", 0.0);
        this->declare_parameter<std::string>("localization_backend", "fast_gicp");
        this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
        this->declare_parameter<std::string>("estimated_pose_topic", "localization_pose");
        this->declare_parameter<std::string>("aligned_scan_topic", "aligned_scan");
        this->declare_parameter<std::string>("candidate_pose_topic", "candidate_pose");
        this->declare_parameter<std::string>("candidate_aligned_scan_topic", "candidate_aligned_scan");
        this->declare_parameter<std::string>("diagnostics_topic", "localization_diagnostics");
        this->declare_parameter<std::string>("motion_model", "none");
        this->declare_parameter<std::string>("odometry_topic", "");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("tf_output_mode", "map_to_base");
        this->declare_parameter<int>("gicp_max_iterations", 20);
        this->declare_parameter<double>("gicp_max_correspondence_distance", 3.0);
        this->declare_parameter<double>("gicp_transformation_epsilon", 1e-4);
        this->declare_parameter<double>("gicp_euclidean_fitness_epsilon", 1e-3);
        this->declare_parameter<int>("gicp_correspondence_randomness", 10);
        this->declare_parameter<double>("coarse_gicp_voxel_leaf_size", 2.0);
        this->declare_parameter<double>("coarse_gicp_max_correspondence_distance", 5.0);
        this->declare_parameter<int>("coarse_gicp_max_iterations", 8);
        this->declare_parameter<int>("fast_gicp_max_iterations", 20);
        this->declare_parameter<double>("fast_gicp_max_correspondence_distance", 3.0);
        this->declare_parameter<double>("fast_gicp_transformation_epsilon", 1e-4);
        this->declare_parameter<int>("fast_gicp_correspondence_randomness", 10);
        this->declare_parameter<int>("fast_gicp_num_threads", 0);
        this->declare_parameter<bool>("use_local_map", true);
        this->declare_parameter<double>("local_map_radius", 30.0);
        this->declare_parameter<double>("registration_max_fitness_score", 0.75);
        this->declare_parameter<double>("registration_max_translation_jump_meters", 2.0);
        this->declare_parameter<double>("registration_max_rotation_jump_radians", 0.523599);
        this->declare_parameter<double>("odometry_max_age_seconds", 0.5);
        this->declare_parameter<double>("odometry_history_duration_seconds", 10.0);

        map_path_ = this->get_parameter("map_path").as_string();
        map_topic_ = this->get_parameter("map_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        map_voxel_leaf_size_ =
            this->get_parameter("map_voxel_leaf_size").as_double();
        auto_adjust_voxel_leaf_size_ =
            this->get_parameter("auto_adjust_voxel_leaf_size").as_bool();
        point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        lidar_voxel_leaf_size_ = this->get_parameter("lidar_voxel_leaf_size").as_double();
        scan_queue_depth_ = std::max(
            1, static_cast<int>(this->get_parameter("scan_queue_depth").as_int()));
        max_registration_rate_hz_ =
            this->get_parameter("max_registration_rate_hz").as_double();
        scan_processing_controller_.setConfig(
            ScanProcessingConfig{max_registration_rate_hz_});
        localization_backend_ = this->get_parameter("localization_backend").as_string();
        initial_pose_topic_ = this->get_parameter("initial_pose_topic").as_string();
        estimated_pose_topic_ = this->get_parameter("estimated_pose_topic").as_string();
        aligned_scan_topic_ = this->get_parameter("aligned_scan_topic").as_string();
        candidate_pose_topic_ = this->get_parameter("candidate_pose_topic").as_string();
        candidate_aligned_scan_topic_ =
            this->get_parameter("candidate_aligned_scan_topic").as_string();
        diagnostics_topic_ = this->get_parameter("diagnostics_topic").as_string();
        motion_model_name_ = this->get_parameter("motion_model").as_string();
        odometry_topic_ = this->get_parameter("odometry_topic").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        tf_output_mode_ = this->get_parameter("tf_output_mode").as_string();
        gicp_config_.max_iterations = this->get_parameter("gicp_max_iterations").as_int();
        gicp_config_.max_correspondence_distance =
            this->get_parameter("gicp_max_correspondence_distance").as_double();
        gicp_config_.transformation_epsilon =
            this->get_parameter("gicp_transformation_epsilon").as_double();
        gicp_config_.euclidean_fitness_epsilon =
            this->get_parameter("gicp_euclidean_fitness_epsilon").as_double();
        gicp_config_.correspondence_randomness =
            this->get_parameter("gicp_correspondence_randomness").as_int();
        coarse_to_fine_gicp_config_.fine_config = gicp_config_;
        coarse_to_fine_gicp_config_.coarse_voxel_leaf_size =
            this->get_parameter("coarse_gicp_voxel_leaf_size").as_double();
        coarse_to_fine_gicp_config_.coarse_max_correspondence_distance =
            this->get_parameter("coarse_gicp_max_correspondence_distance").as_double();
        coarse_to_fine_gicp_config_.coarse_max_iterations =
            this->get_parameter("coarse_gicp_max_iterations").as_int();
        fast_gicp_config_.max_iterations =
            this->get_parameter("fast_gicp_max_iterations").as_int();
        fast_gicp_config_.max_correspondence_distance =
            this->get_parameter("fast_gicp_max_correspondence_distance").as_double();
        fast_gicp_config_.transformation_epsilon =
            this->get_parameter("fast_gicp_transformation_epsilon").as_double();
        fast_gicp_config_.correspondence_randomness =
            this->get_parameter("fast_gicp_correspondence_randomness").as_int();
        fast_gicp_config_.num_threads =
            this->get_parameter("fast_gicp_num_threads").as_int();
        use_local_map_ = this->get_parameter("use_local_map").as_bool();
        local_map_radius_ = this->get_parameter("local_map_radius").as_double();
        registration_max_fitness_score_ =
            this->get_parameter("registration_max_fitness_score").as_double();
        registration_max_translation_jump_meters_ =
            this->get_parameter("registration_max_translation_jump_meters").as_double();
        registration_max_rotation_jump_radians_ =
            this->get_parameter("registration_max_rotation_jump_radians").as_double();
        odometry_max_age_seconds_ =
            this->get_parameter("odometry_max_age_seconds").as_double();
        odometry_history_duration_seconds_ =
            this->get_parameter("odometry_history_duration_seconds").as_double();
    }

    bool Modular3DLocalizerNode::initializeMap()
    {
        if (!map_loader_.loadMap(map_path_))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to load map from %s",
                map_path_.c_str());

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Successfully loaded map from %s",
            map_path_.c_str());

        const auto map_cloud = map_loader_.getMap();

        RCLCPP_INFO(
            this->get_logger(),
            "Map cloud has %zu points.",
            map_cloud->size());

        PointCloudPreprocessorConfig preprocessor_config;
        preprocessor_config.voxel_leaf_size = map_voxel_leaf_size_;
        preprocessor_config.auto_adjust_voxel_leaf_size =
            auto_adjust_voxel_leaf_size_;
        const auto preprocess_result = point_cloud_preprocessor_.process(
            map_cloud, preprocessor_config);
        map_cloud_ = preprocess_result.cloud;

        if (preprocess_result.has_bounds)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Map bounds: x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f].",
                preprocess_result.minimum_point.x,
                preprocess_result.maximum_point.x,
                preprocess_result.minimum_point.y,
                preprocess_result.maximum_point.y,
                preprocess_result.minimum_point.z,
                preprocess_result.maximum_point.z);
        }

        if (preprocess_result.effective_voxel_leaf_size > map_voxel_leaf_size_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Requested voxel size %.3f m would overflow VoxelGrid indices. "
                "Using %.3f m instead.",
                map_voxel_leaf_size_,
                preprocess_result.effective_voxel_leaf_size);
        }

        if (preprocess_result.effective_voxel_leaf_size > 0.0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Downsampled map with %.3f m voxels: %zu points.",
                preprocess_result.effective_voxel_leaf_size,
                preprocess_result.cloud->size());
        }

        map_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::toROSMsg(*preprocess_result.cloud, *map_msg_);

        return true;
    }

    void Modular3DLocalizerNode::setupRosInterfaces()
    {
        rclcpp::QoS map_qos(1);
        map_qos.transient_local();
        map_qos.reliable();

        map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                map_topic_,
                map_qos);

        map_msg_->header.stamp = this->now();
        map_msg_->header.frame_id = map_frame_;
        map_pub_->publish(*map_msg_);

        estimated_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                estimated_pose_topic_, rclcpp::QoS(10));
        diagnostics_pub_ =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnostics_topic_, rclcpp::QoS(10));
        aligned_scan_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                aligned_scan_topic_, rclcpp::QoS(1).reliable());
        candidate_aligned_scan_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                candidate_aligned_scan_topic_, rclcpp::QoS(1).reliable());
        candidate_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                candidate_pose_topic_, rclcpp::QoS(10));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        initial_pose_volatile_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                initial_pose_topic_, rclcpp::QoS(10).reliable(),
                std::bind(
                    &Modular3DLocalizerNode::initialPoseCallback,
                    this,
                    std::placeholders::_1));
        initial_pose_transient_local_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                initial_pose_topic_, rclcpp::QoS(1).reliable().transient_local(),
                std::bind(
                    &Modular3DLocalizerNode::initialPoseCallback,
                    this,
                    std::placeholders::_1));

        if (motion_model_name_ == "odometry" && !odometry_topic_.empty())
        {
            odometry_callback_group_ = this->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions odometry_subscription_options;
            odometry_subscription_options.callback_group = odometry_callback_group_;
            odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odometry_topic_,
                rclcpp::QoS(50).best_effort(),
                std::bind(
                    &Modular3DLocalizerNode::odometryCallback,
                    this,
                    std::placeholders::_1),
                odometry_subscription_options);
            RCLCPP_INFO(
                this->get_logger(),
                "Using odometry motion model input from %s.", odometry_topic_.c_str());
        }

        if(!point_cloud_topic_.empty())
        {
            auto scan_qos = rclcpp::SensorDataQoS();
            scan_qos.keep_last(scan_queue_depth_);
            point_cloud_sub_ =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    point_cloud_topic_,
                    scan_qos,
                    std::bind(
                        &Modular3DLocalizerNode::pointCloudCallback,
                        this,
                        std::placeholders::_1));
        }

    }

    void Modular3DLocalizerNode::initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg)
    {
        const auto &stamp = initial_pose_msg->header.stamp;
        if (last_initial_pose_stamp_.has_value() &&
            last_initial_pose_stamp_->sec == stamp.sec &&
            last_initial_pose_stamp_->nanosec == stamp.nanosec)
        {
            return;
        }
        last_initial_pose_stamp_ = stamp;

        if (initial_pose_msg->header.frame_id != map_frame_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Ignoring initial pose in frame '%s'; expected map frame '%s'.",
                initial_pose_msg->header.frame_id.c_str(), map_frame_.c_str());
            return;
        }

        if (!setCurrentPoseFromBasePose(initial_pose_msg->pose.pose))
        {
            pending_initial_base_pose_ = initial_pose_msg->pose.pose;
            RCLCPP_INFO(
                this->get_logger(),
                "Received initial pose; waiting for TF %s <- %s before applying it.",
                base_frame_.c_str(), lidar_frame_.c_str());
        }
    }

    void Modular3DLocalizerNode::odometryCallback(
        const nav_msgs::msg::Odometry::SharedPtr odometry_msg)
    {
        if (!odometry_motion_model_)
        {
            return;
        }

        if (!odometry_motion_model_->update(*odometry_msg))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Ignoring invalid odometry motion-model input on %s.",
                odometry_topic_.c_str());
            return;
        }

        if (!has_valid_pose_)
        {
            return;
        }
        bool has_map_to_odom = false;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            has_map_to_odom = map_to_odom_.has_value();
        }
        if (!has_map_to_odom)
        {
            updateMapToOdomFromCurrentPose(odometry_msg->header.stamp);
        }
        publishOdometryPrediction(odometry_msg->header.stamp);
    }

    bool Modular3DLocalizerNode::getMotionModelInitialGuess(
        const builtin_interfaces::msg::Time &scan_stamp,
        Eigen::Matrix4f &initial_guess)
    {
        if (!motion_model_)
        {
            return false;
        }

        Eigen::Matrix4f base_to_lidar;
        if (!lookupBaseToLidarTransform(base_to_lidar))
        {
            return false;
        }

        return motion_model_->predict(
            current_pose_map_lidar_, base_to_lidar, scan_stamp, initial_guess);
    }

    bool Modular3DLocalizerNode::updateMapToOdomFromCurrentPose(
        const builtin_interfaces::msg::Time &stamp)
    {
        if (!odometry_motion_model_)
        {
            return false;
        }
        Eigen::Matrix4f base_to_lidar;
        Eigen::Matrix4f odom_to_base;
        if (!lookupBaseToLidarTransform(base_to_lidar) ||
            !odometry_motion_model_->getPoseAt(stamp, odom_to_base))
        {
            return false;
        }
        Eigen::Matrix4f current_pose_map_lidar;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_map_lidar = current_pose_map_lidar_;
        }
        const Eigen::Matrix4f map_to_base =
            current_pose_map_lidar * base_to_lidar.inverse();
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            map_to_odom_ = map_to_base * odom_to_base.inverse();
        }
        return true;
    }

    void Modular3DLocalizerNode::publishOdometryPrediction(
        const builtin_interfaces::msg::Time &stamp)
    {
        std::optional<Eigen::Matrix4f> map_to_odom;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            map_to_odom = map_to_odom_;
        }
        if (!map_to_odom.has_value() || !odometry_motion_model_)
        {
            return;
        }
        Eigen::Matrix4f odom_to_base;
        Eigen::Matrix4f base_to_lidar;
        if (!odometry_motion_model_->getPoseAt(stamp, odom_to_base) ||
            !lookupBaseToLidarTransform(base_to_lidar))
        {
            return;
        }

        const Eigen::Matrix4f map_to_base = *map_to_odom * odom_to_base;
        const Eigen::Matrix4f map_to_lidar = map_to_base * base_to_lidar;
        geometry_msgs::msg::PoseStamped pose_message;
        if (!createMapToBasePoseMessage(map_to_lidar, stamp, pose_message))
        {
            return;
        }
        estimated_pose_pub_->publish(pose_message);

        geometry_msgs::msg::TransformStamped transform_message;
        transform_message.header.stamp = stamp;
        transform_message.header.frame_id = map_frame_;
        if (tf_output_mode_ == "map_to_odom")
        {
            const Eigen::Quaternionf rotation(map_to_odom->block<3, 3>(0, 0));
            transform_message.child_frame_id = odom_frame_;
            transform_message.transform.translation.x = (*map_to_odom)(0, 3);
            transform_message.transform.translation.y = (*map_to_odom)(1, 3);
            transform_message.transform.translation.z = (*map_to_odom)(2, 3);
            transform_message.transform.rotation.x = rotation.x();
            transform_message.transform.rotation.y = rotation.y();
            transform_message.transform.rotation.z = rotation.z();
            transform_message.transform.rotation.w = rotation.w();
        }
        else
        {
            transform_message.child_frame_id = base_frame_;
            transform_message.transform.translation.x = pose_message.pose.position.x;
            transform_message.transform.translation.y = pose_message.pose.position.y;
            transform_message.transform.translation.z = pose_message.pose.position.z;
            transform_message.transform.rotation = pose_message.pose.orientation;
        }
        tf_broadcaster_->sendTransform(transform_message);
    }

    bool Modular3DLocalizerNode::isRegistrationJumpWithinLimits(
        const Eigen::Matrix4f &initial_guess,
        const Eigen::Matrix4f &estimated_pose,
        double &translation_jump_meters,
        double &rotation_jump_radians) const
    {
        const Eigen::Matrix4f initial_to_estimated =
            initial_guess.inverse() * estimated_pose;
        translation_jump_meters =
            static_cast<double>(initial_to_estimated.block<3, 1>(0, 3).norm());

        Eigen::Quaternionf rotation(initial_to_estimated.block<3, 3>(0, 0));
        if (rotation.norm() == 0.0F)
        {
            rotation_jump_radians = std::numeric_limits<double>::infinity();
            return false;
        }

        rotation.normalize();
        const double scalar = std::clamp(
            std::abs(static_cast<double>(rotation.w())), 0.0, 1.0);
        rotation_jump_radians = 2.0 * std::acos(scalar);

        const bool translation_is_valid =
            registration_max_translation_jump_meters_ <= 0.0 ||
            translation_jump_meters <= registration_max_translation_jump_meters_;
        const bool rotation_is_valid =
            registration_max_rotation_jump_radians_ <= 0.0 ||
            rotation_jump_radians <= registration_max_rotation_jump_radians_;
        return translation_is_valid && rotation_is_valid;
    }

    bool Modular3DLocalizerNode::lookupBaseToLidarTransform(
        Eigen::Matrix4f &base_to_lidar)
    {
        try
        {
            const auto transform = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame_, tf2::TimePointZero);
            const auto &translation = transform.transform.translation;
            const auto &orientation = transform.transform.rotation;

            Eigen::Quaternionf rotation(
                static_cast<float>(orientation.w),
                static_cast<float>(orientation.x),
                static_cast<float>(orientation.y),
                static_cast<float>(orientation.z));
            if (rotation.norm() == 0.0F)
            {
                return false;
            }

            base_to_lidar.setIdentity();
            base_to_lidar.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
            base_to_lidar(0, 3) = static_cast<float>(translation.x);
            base_to_lidar(1, 3) = static_cast<float>(translation.y);
            base_to_lidar(2, 3) = static_cast<float>(translation.z);
            return true;
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Cannot transform from lidar frame '%s' to base frame '%s': %s",
                lidar_frame_.c_str(), base_frame_.c_str(), exception.what());
            return false;
        }
    }

    bool Modular3DLocalizerNode::setCurrentPoseFromBasePose(
        const geometry_msgs::msg::Pose &base_pose)
    {
        const auto &position = base_pose.position;
        const auto &orientation = base_pose.orientation;
        Eigen::Quaternionf rotation(
            static_cast<float>(orientation.w),
            static_cast<float>(orientation.x),
            static_cast<float>(orientation.y),
            static_cast<float>(orientation.z));

        if (rotation.norm() == 0.0F)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring initial base pose with a zero quaternion.");
            return false;
        }

        Eigen::Matrix4f map_to_base = Eigen::Matrix4f::Identity();
        rotation.normalize();
        map_to_base.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        map_to_base(0, 3) = static_cast<float>(position.x);
        map_to_base(1, 3) = static_cast<float>(position.y);
        map_to_base(2, 3) = static_cast<float>(position.z);

        Eigen::Matrix4f base_to_lidar;
        if (!lookupBaseToLidarTransform(base_to_lidar))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Ignoring initial pose until TF %s <- %s is available.",
                base_frame_.c_str(), lidar_frame_.c_str());
            return false;
        }

        current_pose_map_lidar_ = map_to_base * base_to_lidar;
        has_valid_pose_ = true;
        motion_model_->resetReference();
        publishDiagnostics(this->now(), "initial_pose_applied");

        RCLCPP_INFO(
            this->get_logger(),
            "Initialized map-to-lidar pose from map-to-%s initial pose on %s.",
            base_frame_.c_str(), initial_pose_topic_.c_str());
        return true;
    }

    void Modular3DLocalizerNode::publishDiagnostics(
        const builtin_interfaces::msg::Time &stamp,
        const std::string &reason,
        const std::size_t source_point_count,
        const std::size_t target_point_count,
        const double registration_duration_seconds,
        const double fitness_score,
        const double translation_jump_meters,
        const double rotation_jump_radians)
    {
        if (!diagnostics_pub_)
        {
            return;
        }

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = this->get_name() + std::string("/localization");
        status.hardware_id = localization_backend_;
        status.message = reason;
        status.level = has_valid_pose_
            ? diagnostic_msgs::msg::DiagnosticStatus::OK
            : diagnostic_msgs::msg::DiagnosticStatus::WARN;

        const auto add_value = [&status](const std::string &key, const std::string &value)
        {
            diagnostic_msgs::msg::KeyValue item;
            item.key = key;
            item.value = value;
            status.values.push_back(item);
        };

        add_value("has_initial_pose", has_valid_pose_ ? "true" : "false");
        add_value("source_point_count", std::to_string(source_point_count));
        add_value("target_point_count", std::to_string(target_point_count));
        add_value("registration_duration_sec", std::to_string(registration_duration_seconds));
        add_value(
            "fitness_score",
            std::isfinite(fitness_score) ? std::to_string(fitness_score) : "not_available");
        add_value("translation_jump_meters", std::to_string(translation_jump_meters));
        add_value("rotation_jump_radians", std::to_string(rotation_jump_radians));

        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        diagnostics.header.stamp = stamp;
        diagnostics.status.push_back(status);
        diagnostics_pub_->publish(diagnostics);
    }

    void Modular3DLocalizerNode::applyPendingInitialPose()
    {
        if (!pending_initial_base_pose_.has_value())
        {
            return;
        }

        if (setCurrentPoseFromBasePose(*pending_initial_base_pose_))
        {
            pending_initial_base_pose_.reset();
        }
    }

    bool Modular3DLocalizerNode::createMapToBasePoseMessage(
        const Eigen::Matrix4f &pose_map_lidar,
        const builtin_interfaces::msg::Time &stamp,
        geometry_msgs::msg::PoseStamped &pose_message)
    {
        Eigen::Matrix4f base_to_lidar;
        if (!lookupBaseToLidarTransform(base_to_lidar))
        {
            return false;
        }

        const Eigen::Matrix4f map_to_base =
            pose_map_lidar * base_to_lidar.inverse();
        pose_message.header.stamp = stamp;
        pose_message.header.frame_id = map_frame_;
        pose_message.pose.position.x = map_to_base(0, 3);
        pose_message.pose.position.y = map_to_base(1, 3);
        pose_message.pose.position.z = map_to_base(2, 3);

        const Eigen::Quaternionf rotation(map_to_base.block<3, 3>(0, 0));
        pose_message.pose.orientation.x = rotation.x();
        pose_message.pose.orientation.y = rotation.y();
        pose_message.pose.orientation.z = rotation.z();
        pose_message.pose.orientation.w = rotation.w();
        return true;
    }

    void Modular3DLocalizerNode::publishEstimatedPose(
        const builtin_interfaces::msg::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose_message;
        if (!createMapToBasePoseMessage(current_pose_map_lidar_, stamp, pose_message))
        {
            return;
        }

        estimated_pose_pub_->publish(pose_message);

        geometry_msgs::msg::TransformStamped transform_message;
        transform_message.header = pose_message.header;
        if (tf_output_mode_ == "map_to_base")
        {
            transform_message.child_frame_id = base_frame_;
            transform_message.transform.translation.x = pose_message.pose.position.x;
            transform_message.transform.translation.y = pose_message.pose.position.y;
            transform_message.transform.translation.z = pose_message.pose.position.z;
            transform_message.transform.rotation = pose_message.pose.orientation;
        }
        else
        {
            std::optional<Eigen::Matrix4f> map_to_odom;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                map_to_odom = map_to_odom_;
            }
            if (!map_to_odom.has_value())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Cannot publish map-to-odom TF without a time-aligned odometry reference.");
                return;
            }
            const Eigen::Quaternionf map_to_odom_rotation(
                map_to_odom->block<3, 3>(0, 0));
            transform_message.child_frame_id = odom_frame_;
            transform_message.transform.translation.x = (*map_to_odom)(0, 3);
            transform_message.transform.translation.y = (*map_to_odom)(1, 3);
            transform_message.transform.translation.z = (*map_to_odom)(2, 3);
            transform_message.transform.rotation.x = map_to_odom_rotation.x();
            transform_message.transform.rotation.y = map_to_odom_rotation.y();
            transform_message.transform.rotation.z = map_to_odom_rotation.z();
            transform_message.transform.rotation.w = map_to_odom_rotation.w();
        }
        tf_broadcaster_->sendTransform(transform_message);
    }

    void Modular3DLocalizerNode::publishCandidatePose(
        const Eigen::Matrix4f &candidate_pose_map_lidar,
        const builtin_interfaces::msg::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose_message;
        if (createMapToBasePoseMessage(candidate_pose_map_lidar, stamp, pose_message))
        {
            candidate_pose_pub_->publish(pose_message);
        }
    }

    void Modular3DLocalizerNode::publishAlignedScan(
        MapLoader::PointCloud::ConstPtr scan_cloud,
        const builtin_interfaces::msg::Time &stamp)
    {
        MapLoader::PointCloud aligned_cloud;
        pcl::transformPointCloud(*scan_cloud, aligned_cloud, current_pose_map_lidar_);

        sensor_msgs::msg::PointCloud2 aligned_scan_message;
        pcl::toROSMsg(aligned_cloud, aligned_scan_message);
        aligned_scan_message.header.stamp = stamp;
        aligned_scan_message.header.frame_id = map_frame_;
        aligned_scan_pub_->publish(aligned_scan_message);
    }

    void Modular3DLocalizerNode::publishCandidateAlignedScan(
        MapLoader::PointCloud::ConstPtr scan_cloud,
        const Eigen::Matrix4f &candidate_pose_map_lidar,
        const builtin_interfaces::msg::Time &stamp)
    {
        MapLoader::PointCloud candidate_aligned_cloud;
        pcl::transformPointCloud(*scan_cloud, candidate_aligned_cloud, candidate_pose_map_lidar);

        sensor_msgs::msg::PointCloud2 candidate_scan_message;
        pcl::toROSMsg(candidate_aligned_cloud, candidate_scan_message);
        candidate_scan_message.header.stamp = stamp;
        candidate_scan_message.header.frame_id = map_frame_;
        candidate_aligned_scan_pub_->publish(candidate_scan_message);
    }

    void Modular3DLocalizerNode::pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg)
    {
        if (!pointcloud_msg->header.frame_id.empty() &&
            pointcloud_msg->header.frame_id != lidar_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Ignoring point cloud in frame '%s'; expected lidar frame '%s'.",
                pointcloud_msg->header.frame_id.c_str(), lidar_frame_.c_str());
            return;
        }

        applyPendingInitialPose();
        if (!has_valid_pose_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Ignoring point cloud until an initial map-to-%s pose is available.",
                base_frame_.c_str());
            return;
        }

        if (!scan_processing_controller_.shouldProcess(pointcloud_msg->header.stamp))
        {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Skipping scan due to max_registration_rate_hz=%.3f.",
                max_registration_rate_hz_);
            return;
        }
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Received point cloud: width=%u, height=%u, frame=%s",
        //     pointcloud_msg->width,
        //     pointcloud_msg->height,
        //     pointcloud_msg->header.frame_id.c_str());
        auto lidar_cloud = std::make_shared<MapLoader::PointCloud>();
        pcl::fromROSMsg(*pointcloud_msg, *lidar_cloud);
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Received point cloud: width=%u, height=%u, frame=%s, points=%zu",
        //     pointcloud_msg->width,
        //     pointcloud_msg->height,
        //     pointcloud_msg->header.frame_id.c_str(),
        //     lidar_cloud->size());
        PointCloudPreprocessorConfig scan_config;
        scan_config.voxel_leaf_size = lidar_voxel_leaf_size_;
        scan_config.auto_adjust_voxel_leaf_size = false;
        const auto scan_preprocess_result = point_cloud_preprocessor_.process(
            lidar_cloud, scan_config);
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Scan downsampled: %zu -> %zu points",
        //     lidar_cloud->size(),
        //     scan_preprocess_result.cloud->size());
        Eigen::Matrix4f initial_guess = current_pose_map_lidar_;
        if (getMotionModelInitialGuess(pointcloud_msg->header.stamp, initial_guess))
        {
            RCLCPP_DEBUG(
                this->get_logger(), "Using the latest odometry pose prior for the registration guess.");
        }

        if (!registration_backend_)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "No localization backend is configured; ignoring point cloud.");
            return;
        }

        std::size_t target_point_count = map_cloud_->size();
        if (use_local_map_ && has_valid_pose_)
        {
            const auto local_map = local_map_extractor_.extract(
                map_cloud_, initial_guess.block<3, 1>(0, 3),
                static_cast<float>(local_map_radius_));
            if (local_map->empty())
            {
                publishDiagnostics(
                    pointcloud_msg->header.stamp,
                    "local_map_empty",
                    scan_preprocess_result.cloud->size());
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Local map is empty at the predicted pose; skipping tracking registration.");
                return;
            }
            else
            {
                target_point_count = local_map->size();
                registration_backend_->setTarget(local_map);
            }
        }

        const auto registration_start = std::chrono::steady_clock::now();
        const RegistrationResult result = registration_backend_->align(
            scan_preprocess_result.cloud,
            initial_guess);
        const auto registration_duration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - registration_start).count();

        if (result.converged)
        {
            publishCandidateAlignedScan(
                scan_preprocess_result.cloud, result.transform, pointcloud_msg->header.stamp);
            publishCandidatePose(result.transform, pointcloud_msg->header.stamp);
        }

        double translation_jump_meters = 0.0;
        double rotation_jump_radians = 0.0;
        const bool jump_is_valid = result.converged &&
            isRegistrationJumpWithinLimits(
                initial_guess,
                result.transform,
                translation_jump_meters,
                rotation_jump_radians);
        const bool fitness_is_valid =
            registration_max_fitness_score_ <= 0.0 ||
            result.fitness_score <= registration_max_fitness_score_;
        const bool accepted = result.converged && jump_is_valid && fitness_is_valid;

        if (accepted)
        {
            current_pose_map_lidar_ = result.transform;
            has_valid_pose_ = true;
            if (!odometry_motion_model_ ||
                !odometry_motion_model_->resetReferenceAt(pointcloud_msg->header.stamp))
            {
                motion_model_->resetReference();
            }
            updateMapToOdomFromCurrentPose(pointcloud_msg->header.stamp);
            publishAlignedScan(scan_preprocess_result.cloud, pointcloud_msg->header.stamp);
            publishEstimatedPose(pointcloud_msg->header.stamp);
            publishDiagnostics(
                pointcloud_msg->header.stamp,
                "registration_accepted",
                scan_preprocess_result.cloud->size(),
                target_point_count,
                registration_duration,
                result.fitness_score,
                translation_jump_meters,
                rotation_jump_radians);
            RCLCPP_INFO(
                this->get_logger(),
                "Registration accepted: scan=%zu points, target=%zu points, time=%.3f s, "
                "fitness=%.6f, jump=[%.3f m, %.3f rad].",
                scan_preprocess_result.cloud->size(), target_point_count, registration_duration,
                result.fitness_score, translation_jump_meters, rotation_jump_radians);
        }
        else if (result.converged)
        {
            publishDiagnostics(
                pointcloud_msg->header.stamp,
                jump_is_valid ? "registration_rejected_fitness" : "registration_rejected_pose_jump",
                scan_preprocess_result.cloud->size(),
                target_point_count,
                registration_duration,
                result.fitness_score,
                translation_jump_meters,
                rotation_jump_radians);
            RCLCPP_WARN(
                this->get_logger(),
                "Registration rejected: scan=%zu points, target=%zu points, time=%.3f s, "
                "fitness=%.6f (threshold=%.6f), jump=[%.3f m, %.3f rad] "
                "(limits=[%.3f m, %.3f rad]). Keeping the previous pose.",
                scan_preprocess_result.cloud->size(), target_point_count, registration_duration,
                result.fitness_score, registration_max_fitness_score_,
                translation_jump_meters, rotation_jump_radians,
                registration_max_translation_jump_meters_,
                registration_max_rotation_jump_radians_);
        }
        else
        {
            publishDiagnostics(
                pointcloud_msg->header.stamp,
                "registration_not_converged",
                scan_preprocess_result.cloud->size(),
                target_point_count,
                registration_duration,
                result.fitness_score,
                translation_jump_meters,
                rotation_jump_radians);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Registration did not converge: scan=%zu points, target=%zu points, time=%.3f s.",
                scan_preprocess_result.cloud->size(), target_point_count, registration_duration);
        }
    }

    void Modular3DLocalizerNode::initializeLocalizationBackend()
    {
        registration_backend_ = createRegistrationBackend(
            localization_backend_, gicp_config_, coarse_to_fine_gicp_config_,
            fast_gicp_config_);
        if (!registration_backend_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported localization backend: %s",
                localization_backend_.c_str());
            return;
        }

        registration_backend_->setTarget(map_cloud_);
        RCLCPP_INFO(
            this->get_logger(),
            "Using %s as the localization backend.",
            localization_backend_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Scan input policy: queue_depth=%d, max_registration_rate=%.3f Hz.",
            scan_queue_depth_, max_registration_rate_hz_);
        if (localization_backend_ == "gicp")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "GICP config: iterations=%d, max_correspondence_distance=%.3f m, "
                "transformation_epsilon=%.6f, fitness_epsilon=%.6f, randomness=%d.",
                gicp_config_.max_iterations,
                gicp_config_.max_correspondence_distance,
                gicp_config_.transformation_epsilon,
                gicp_config_.euclidean_fitness_epsilon,
                gicp_config_.correspondence_randomness);
        }
        else if (localization_backend_ == "coarse_to_fine_gicp")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Coarse-to-fine GICP config: coarse_leaf=%.3f m, coarse_iterations=%d, "
                "coarse_max_correspondence_distance=%.3f m.",
                coarse_to_fine_gicp_config_.coarse_voxel_leaf_size,
                coarse_to_fine_gicp_config_.coarse_max_iterations,
                coarse_to_fine_gicp_config_.coarse_max_correspondence_distance);
        }
        else if (localization_backend_ == "fast_gicp")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "FastGICP config: iterations=%d, max_correspondence_distance=%.3f m, "
                "transformation_epsilon=%.6f, randomness=%d, threads=%d.",
                fast_gicp_config_.max_iterations,
                fast_gicp_config_.max_correspondence_distance,
                fast_gicp_config_.transformation_epsilon,
                fast_gicp_config_.correspondence_randomness,
                fast_gicp_config_.num_threads);
        }
    }

    void Modular3DLocalizerNode::initializeMotionModel()
    {
        const OdometryMotionModelConfig odometry_config{
            odom_frame_, base_frame_, odometry_max_age_seconds_,
            odometry_history_duration_seconds_};
        motion_model_ = createMotionModel(motion_model_name_, odometry_config);
        if (!motion_model_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported motion model '%s'; falling back to 'none'.",
                motion_model_name_.c_str());
            motion_model_name_ = "none";
            motion_model_ = createMotionModel(motion_model_name_, odometry_config);
        }

        odometry_motion_model_ =
            dynamic_cast<OdometryMotionModel *>(motion_model_.get());
        RCLCPP_INFO(
            this->get_logger(), "Using %s motion model.", motion_model_name_.c_str());

        if (tf_output_mode_ != "map_to_base" && tf_output_mode_ != "map_to_odom")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported tf_output_mode '%s'; falling back to 'map_to_base'.",
                tf_output_mode_.c_str());
            tf_output_mode_ = "map_to_base";
        }
        if (tf_output_mode_ == "map_to_odom" && !odometry_motion_model_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "tf_output_mode=map_to_odom requires motion_model=odometry. "
                "No localization TF will be published until that is configured.");
        }

        if (motion_model_name_ == "odometry" && odometry_topic_.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "The odometry motion model is selected but odometry_topic is empty. "
                "The node will use the last accepted pose until odometry arrives.");
        }
    }
}  // namespace modular_3d_localizer
