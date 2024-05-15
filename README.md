# m3t_tracker_ros
M3T base 3D tracker for real time object tracking with detections from HappyPose, Based on [m3t_tracker](https://github.com/agimus-project/pym3t).

## Build instructions

:warning: Conda installation is not supported

Currently, there is no automated build for m3t_tracker library itself built into the ROS node.

```bash
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO
# parameter --symlink-install is optional
colcon build --symlink-install
```

## ROS API

### Publishers

- **m3t_tracker/detections** [vision_msgs/msg/Detection2DArray]

    Array containing poses from **reference/detections** topic refined by M3T tracker.

- **m3t_tracker/vision_info** [vision_msgs/msg/VisionInfo]

    Relied information from **reference/vision_info** topic.

### Subscribers

- image_color** [sensor_msgs/msg/Image]

    Color video stream from a given camera.

<!-- TODO fix message type -->
- image_depth** [sensor_msgs/msg/Image]

    Depth video stream from a given camera.

- camera_info** [sensor_msgs/msg/CameraInfo]

    Topic used to obtain intrinsic matrix of a given camera.

- **reference/detections** [vision_msgs/msg/Detection2DArray]

    Detections to be used to initialize tracking.

- **reference/vision_info** [vision_msgs/msg/VisionInfo]

    Information about the used pose estimator and URL with object database location.

### Service Servers

- **/m3t_tracker_ros/set_paramters** [rcl_interfaces/srv/SetParameters]

    Allows to dynamically change ROS parameters. For more information. For more information, refer to the [documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

### Parameters

Parameters are generated with [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library). Currently, no automatic documentation generation is set up. Refer to [m3t_tracker_ros_parameters.yaml](./m3t_tracker_ros/m3t_tracker_ros/m3t_tracker_ros_parameters.yaml) for more information.

Note that some of the parameters are possible to tune in the runtime. Refer to the code generation YAML file to see which of them are available.

For examples on how to configure the node, see the examples for [single-view](./m3t_tracker_examples/config/cosypose_params.yaml) and [multi-view](./m3t_tracker_examples/config/cosypose_params_multiview.yaml).
