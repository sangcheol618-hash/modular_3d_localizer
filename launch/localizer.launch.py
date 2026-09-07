from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Empty means "keep the params_file value". This lets a profile be reused
# while callers override just a map path or a topic from the command line.
NODE_PARAMETERS = [
    ("map_path", str), ("map_topic", str), ("map_frame", str),
    ("map_voxel_leaf_size", float), ("auto_adjust_voxel_leaf_size", bool),
    ("point_cloud_topic", str), ("base_frame", str), ("lidar_frame", str),
    ("lidar_voxel_leaf_size", float), ("scan_queue_depth", int),
    ("max_registration_rate_hz", float), ("localization_backend", str),
    ("initial_pose_topic", str), ("estimated_pose_topic", str),
    ("aligned_scan_topic", str), ("candidate_pose_topic", str),
    ("candidate_aligned_scan_topic", str), ("diagnostics_topic", str),
    ("motion_model", str), ("odometry_topic", str), ("odom_frame", str),
    ("odometry_max_age_seconds", float), ("odometry_history_duration_seconds", float),
    ("tf_output_mode", str), ("gicp_max_iterations", int),
    ("gicp_max_correspondence_distance", float),
    ("gicp_transformation_epsilon", float),
    ("gicp_euclidean_fitness_epsilon", float),
    ("gicp_correspondence_randomness", int),
    ("coarse_gicp_voxel_leaf_size", float),
    ("coarse_gicp_max_correspondence_distance", float),
    ("coarse_gicp_max_iterations", int), ("fast_gicp_max_iterations", int),
    ("fast_gicp_max_correspondence_distance", float),
    ("fast_gicp_transformation_epsilon", float),
    ("fast_gicp_correspondence_randomness", int), ("fast_gicp_num_threads", int),
    ("use_local_map", bool),
    ("local_map_radius", float), ("registration_max_fitness_score", float),
    ("registration_max_translation_jump_meters", float),
    ("registration_max_rotation_jump_radians", float),
]


def parse_launch_parameter(value, value_type):
    if value_type is bool:
        normalized = value.strip().lower()
        if normalized in ("true", "1"):
            return True
        if normalized in ("false", "0"):
            return False
        raise RuntimeError(
            f"Expected a boolean launch value, got '{value}'. Use true or false.")
    return value_type(value)


def create_localizer_node(context):
    overrides = {}
    for name, value_type in NODE_PARAMETERS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = ParameterValue(
                parse_launch_parameter(value, value_type), value_type=value_type)
    return [Node(
        package="modular_3d_localizer",
        executable="modular_3d_localizer_node",
        name="modular_3d_localizer_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("modular_3d_localizer"), "config", "defaults.yaml"]),
            LaunchConfiguration("params_file"), overrides],
    )]


def generate_launch_description():
    package_share = FindPackageShare("modular_3d_localizer")
    rviz_config = PathJoinSubstitution([package_share, "rviz", "localization.rviz"])
    default_params = PathJoinSubstitution([package_share, "config", "defaults.yaml"])

    arguments = [DeclareLaunchArgument("params_file", default_value=default_params)]
    arguments.extend(
        DeclareLaunchArgument(name, default_value="") for name, _ in NODE_PARAMETERS)
    arguments.extend([
        DeclareLaunchArgument("publish_base_lidar_tf", default_value="false"),
        DeclareLaunchArgument("base_lidar_x", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_y", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_z", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("base_lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ])

    static_base_to_lidar_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=[
            "--x", LaunchConfiguration("base_lidar_x"),
            "--y", LaunchConfiguration("base_lidar_y"),
            "--z", LaunchConfiguration("base_lidar_z"),
            "--roll", LaunchConfiguration("base_lidar_roll"),
            "--pitch", LaunchConfiguration("base_lidar_pitch"),
            "--yaw", LaunchConfiguration("base_lidar_yaw"),
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("lidar_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_base_lidar_tf")), output="screen")
    rviz = Node(
        package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")), output="screen")
    return LaunchDescription(arguments + [
        static_base_to_lidar_tf, OpaqueFunction(function=create_localizer_node), rviz])
