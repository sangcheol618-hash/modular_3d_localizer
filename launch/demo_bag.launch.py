from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localizer_launch = PathJoinSubstitution(
        [FindPackageShare("modular_3d_localizer"), "launch", "localizer.launch.py"]
    )

    arguments = [
        DeclareLaunchArgument("map_path", description="Path to a PCD or PLY map file."),
        DeclareLaunchArgument("point_cloud_topic", description="PointCloud2 topic stored in the bag."),
        DeclareLaunchArgument("bag_path", default_value="", description="ROS 2 bag directory or DB3 path."),
        DeclareLaunchArgument("play_bag", default_value="false"),
        DeclareLaunchArgument("bag_start_delay", default_value="10.0"),
        DeclareLaunchArgument("map_voxel_leaf_size", default_value="0.2"),
        DeclareLaunchArgument("lidar_voxel_leaf_size", default_value="0.2"),
        DeclareLaunchArgument("scan_queue_depth", default_value="1"),
        DeclareLaunchArgument("max_registration_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar"),
        DeclareLaunchArgument("publish_base_lidar_tf", default_value="false"),
        DeclareLaunchArgument("base_lidar_x", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_y", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_z", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("localization_backend", default_value="fast_gicp"),
        DeclareLaunchArgument("aligned_scan_topic", default_value="aligned_scan"),
        DeclareLaunchArgument("candidate_pose_topic", default_value="candidate_pose"),
        DeclareLaunchArgument("candidate_aligned_scan_topic", default_value="candidate_aligned_scan"),
        DeclareLaunchArgument("diagnostics_topic", default_value="localization_diagnostics"),
        DeclareLaunchArgument("motion_model", default_value="none"),
        DeclareLaunchArgument("odometry_topic", default_value=""),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("odometry_max_age_seconds", default_value="0.5"),
        DeclareLaunchArgument("tf_output_mode", default_value="map_to_base"),
        DeclareLaunchArgument("gicp_max_iterations", default_value="20"),
        DeclareLaunchArgument("gicp_max_correspondence_distance", default_value="3.0"),
        DeclareLaunchArgument("gicp_transformation_epsilon", default_value="0.0001"),
        DeclareLaunchArgument("gicp_euclidean_fitness_epsilon", default_value="0.001"),
        DeclareLaunchArgument("gicp_correspondence_randomness", default_value="10"),
        DeclareLaunchArgument("use_local_map", default_value="true"),
        DeclareLaunchArgument("local_map_radius", default_value="30.0"),
        DeclareLaunchArgument("registration_max_fitness_score", default_value="0.75"),
        DeclareLaunchArgument("registration_max_translation_jump_meters", default_value="2.0"),
        DeclareLaunchArgument("registration_max_rotation_jump_radians", default_value="0.523599"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch),
        launch_arguments={
            "map_path": LaunchConfiguration("map_path"),
            "point_cloud_topic": LaunchConfiguration("point_cloud_topic"),
            "map_voxel_leaf_size": LaunchConfiguration("map_voxel_leaf_size"),
            "lidar_voxel_leaf_size": LaunchConfiguration("lidar_voxel_leaf_size"),
            "scan_queue_depth": LaunchConfiguration("scan_queue_depth"),
            "max_registration_rate_hz": LaunchConfiguration("max_registration_rate_hz"),
            "base_frame": LaunchConfiguration("base_frame"),
            "lidar_frame": LaunchConfiguration("lidar_frame"),
            "publish_base_lidar_tf": LaunchConfiguration("publish_base_lidar_tf"),
            "base_lidar_x": LaunchConfiguration("base_lidar_x"),
            "base_lidar_y": LaunchConfiguration("base_lidar_y"),
            "base_lidar_z": LaunchConfiguration("base_lidar_z"),
            "base_lidar_roll": LaunchConfiguration("base_lidar_roll"),
            "base_lidar_pitch": LaunchConfiguration("base_lidar_pitch"),
            "base_lidar_yaw": LaunchConfiguration("base_lidar_yaw"),
            "localization_backend": LaunchConfiguration("localization_backend"),
            "aligned_scan_topic": LaunchConfiguration("aligned_scan_topic"),
            "candidate_pose_topic": LaunchConfiguration("candidate_pose_topic"),
            "candidate_aligned_scan_topic": LaunchConfiguration("candidate_aligned_scan_topic"),
            "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
            "motion_model": LaunchConfiguration("motion_model"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "odometry_max_age_seconds": LaunchConfiguration("odometry_max_age_seconds"),
            "tf_output_mode": LaunchConfiguration("tf_output_mode"),
            "gicp_max_iterations": LaunchConfiguration("gicp_max_iterations"),
            "gicp_max_correspondence_distance": LaunchConfiguration("gicp_max_correspondence_distance"),
            "gicp_transformation_epsilon": LaunchConfiguration("gicp_transformation_epsilon"),
            "gicp_euclidean_fitness_epsilon": LaunchConfiguration("gicp_euclidean_fitness_epsilon"),
            "gicp_correspondence_randomness": LaunchConfiguration("gicp_correspondence_randomness"),
            "use_local_map": LaunchConfiguration("use_local_map"),
            "local_map_radius": LaunchConfiguration("local_map_radius"),
            "registration_max_fitness_score": LaunchConfiguration("registration_max_fitness_score"),
            "registration_max_translation_jump_meters": LaunchConfiguration("registration_max_translation_jump_meters"),
            "registration_max_rotation_jump_radians": LaunchConfiguration("registration_max_rotation_jump_radians"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    bag_playback = TimerAction(
        period=LaunchConfiguration("bag_start_delay"),
        actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path")],
                condition=IfCondition(LaunchConfiguration("play_bag")),
                output="screen",
            )
        ],
    )

    return LaunchDescription(arguments + [localizer, bag_playback])
