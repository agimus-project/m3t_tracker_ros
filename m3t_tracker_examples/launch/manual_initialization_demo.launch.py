import pathlib

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    object_distance_from_camera = LaunchConfiguration("object_distance_from_camera")
    object_rpy = LaunchConfiguration("object_rpy")
    mesh_file = LaunchConfiguration("mesh_file")
    mesh_scale = LaunchConfiguration("mesh_scale")
    camera_device = LaunchConfiguration("camera_device")
    camera_fov = LaunchConfiguration("camera_fov")

    # Create temporary path for generated objects
    tmp_data_path = pathlib.Path("/tmp/m3t_tracker_ros_data")
    tmp_data_path.mkdir(parents=True, mode=0o700, exist_ok=True)

    # Extract path to the input file
    input_file = pathlib.Path(mesh_file.perform(context)).absolute()
    input_folder = input_file.parent
    object_class_id = input_file.stem

    # Script converting input meshes to format used my M3T
    prepare_sparse_views = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            " run m3t_tracker_ros prepare_sparse_views ",
            "--input-path ",
            input_folder.as_posix(),
            " --output-path ",
            tmp_data_path.as_posix(),
            " --mesh-format ",
            input_file.suffix[1:],
            " --mesh-scale ",
            mesh_scale,
            " --filter-objects ",
            object_class_id,
        ],
        name="prepare_sparse_views",
        output="both",
        shell=True,
    )

    # Node initializing pose of the tracked object
    dummy_detection_publisher = Node(
        package="m3t_tracker_examples",
        executable="dummy_detection_publisher",
        name="dummy_detection_publisher_node",
        parameters=[
            {
                "class_id": object_class_id,
                "frame_id": "webcam",
                "object_distance_from_camera": object_distance_from_camera,
                "object_rpy": object_rpy,
                # Use converted mesh file
                "mesh_file": tmp_data_path.as_posix() + "/" + object_class_id + ".obj",
            }
        ],
    )

    # Start ROS node for M3T tracker
    m3t_tracker = Node(
        package="m3t_tracker_ros",
        executable="real_time_tracker_node",
        output="screen",
        parameters=[
            {
                "dataset_path": tmp_data_path.as_posix(),
                "tracked_objects": [object_class_id],
                "region_modality.model_occlusion": True,
                "region_modality.learning_rate_f": 0.5,
                "region_modality.learning_rate_b": 0.5,
                "optimizer.tikhonov_parameter_rotation": 500.0,
                "optimizer.tikhonov_parameter_translation": 20000.0,
                # "use_texture_modality": True
            }
        ],
    )

    # Start ROS node for image publishing
    image_publisher_node = Node(
        package="image_publisher",
        executable="image_publisher_node",
        namespace="color",
        output="screen",
        parameters=[
            {
                "publish_rate": 23.0,
                "frame_id": "webcam",
                "filename": camera_device,
                "field_of_view": camera_fov,
            }
        ],
    )

    # Start mesh publisher
    happypose_marker_publisher = Node(
        package="happypose_marker_publisher",
        executable="marker_publisher",
        output="screen",
        namespace="m3t_tracker",
        parameters=[
            {
                "filename_format": "${class_id}.obj",
                "marker_lifetime": 1.0 / 23.0 + 0.01,
                "mesh.use_vision_info_uri": False,
                "mesh.uri": "file://" + tmp_data_path.as_posix(),
                "mesh.scale": 1.0,
                "mesh.color_overwrite": [0.5, 1.0, 0.5, 1.0],
            }
        ],
    )

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("m3t_tracker_examples"),
            "rviz",
            "m3t_tracker_example.rviz",
        ]
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
        # m3t_tracker
        prepare_sparse_views,
        # Once sparse view exits cleanly start the pipeline
        RegisterEventHandler(
            OnProcessExit(
                target_action=prepare_sparse_views,
                on_exit=[
                    dummy_detection_publisher,
                    image_publisher_node,
                    rviz_node,
                    static_transform_publisher_node,
                    m3t_tracker,
                    happypose_marker_publisher,
                ],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "object_distance_from_camera",
            default_value="0.5",
            description="Initial distance from the camera at which object will be expected.",
        ),
        DeclareLaunchArgument(
            "object_rpy",
            default_value="[0.0, 0.0, 0.0]",
            description="Initial rotation [deg] of the object.",
        ),
        DeclareLaunchArgument(
            "mesh_file",
            description="Path to a file containing the object you want to track.",
        ),
        DeclareLaunchArgument(
            "mesh_scale",
            default_value="0.001",
            description="Path to a file containing the object you want to track.",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video0",
            description="Path to video device used during the demo.",
        ),
        DeclareLaunchArgument(
            "camera_fov",
            default_value="50.0",
            description="Field of view [deg] of used camera.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
