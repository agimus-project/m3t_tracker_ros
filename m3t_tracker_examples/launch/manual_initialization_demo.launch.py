from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    distance_from_cam = LaunchConfiguration("distance_from_cam")
    object_rpy = LaunchConfiguration("object_rpy")
    mesh_file = LaunchConfiguration("mesh_file")
    camera_device = LaunchConfiguration("camera_device")

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("m3t_tracker_examples"),
            "rviz",
            "m3t_tracker_example.rviz",
        ]
    )

    # Start ROS node for image publishing
    image_publisher_node = Node(
        package="image_publisher",
        executable="image_publisher_node",
        namespace="color",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "publish_rate": 23.0,
                "frame_id": "webcam",
                "filename": camera_device,
                # Camera info is ignored by the node on startup.
                # Waiting for https://github.com/ros-perception/image_pipeline/issues/965
                "camera_info_url": "package://m3t_tracker_examples/config/camera_info.yaml",
            }
        ],
        remappings=[
            # Remapped topics have to match the names from
            ("image_raw", "image"),
        ],
    )

    # Node initializing pose of the tracked object
    dummy_detection_publisher = Node(
        package="m3t_tracker_examples",
        executable="dummy_detection_publisher",
        name="dummy_detection_publisher_node",
        parameters=[
            {
                "frame_id": "webcam",
                "distance_from_cam": distance_from_cam,
                "object_rpy": object_rpy,
                "mesh_file": mesh_file,
            }
        ],
    )

    # Start RViz2 ROS node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config_path,
        ],
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=[
            "--roll",
            "-1.57",
            "--yaw",
            "-1.57",
            "--z",
            "0.3",
            "--frame-id",
            "world",
            "--child-frame-id",
            "webcam",
        ],
    )

    return [
        image_publisher_node,
        dummy_detection_publisher,
        rviz_node,
        static_transform_publisher_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "distance_from_cam",
            default_value="0.5",
            description="Initial distance from the camera at which object will be expected.",
        ),
        DeclareLaunchArgument(
            "object_rpy",
            default_value="[0.0, 0.0, 0.0]",
            description="Initial rotation of the object.",
        ),
        DeclareLaunchArgument(
            "mesh_file",
            default_value="",
            description="Path to a file containing the object you want to track.",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video0",
            description="Path to video device used during the demo.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
