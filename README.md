# m3t_tracker_ros
M3T base 3D tracker for real time object tracking with detections from HappyPose, Based on [m3t_tracker](https://github.com/agimus-project/pym3t).

## Build instructions

:warning: Conda installation is not supported

Currently, there is no automated build for m3t_tracker library itself built into the ROS node.

:warning: As a prerequirement user has to build the [pym3t](https://github.com/agimus-project/pym3t) from source!

```bash
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO
# parameter --symlink-install is optional
colcon build --symlink-install
```

## Launch

This demo tracks given object based on the video input from your webcam.

To launch the demo you need two terminal windows. In the first window run:
```bash
ros2 launch m3t_tracker_examples manual_initialization_demo.launch.py mesh_file:=<global path to mesh file>
```
This will start M3T tracker, subscribe to your webcam and open RViz window.

In the second terminal run:
```bash
ros2 run m3t_tracker_examples keyboard_monitor
```
This node will await your input from the keyboard. Image preview in RViz will overlay an red version of your tracked object over the video stream. Move the object you want to track to align it with its preview in RViz and press **space bar** to start tracking. The red object will disappear and now, normal version of this object will be show up in the RViz, now tracking it in the space.

## Data Preprocessing

Tracker ROS node expects the meshes to be preprocessed before launch. This is done by converting them to **Wavefront** (**.obj**) file format and creating binary files with sparse views for Region Model (file format **.m3t_rmb**) and optionally Depth Model (file format **.m3t_dmb**).

To preprocess the data you can use following script:
```bash
ros2 run m3t_tracker_ros prepare_sparse_views \
    --input-path <path to folder containing meshes to convert> \
    --output-path <path to folder where converted meshes will be saved> \
    --use-depth # optional flag
```
To see all options use `ros2 run m3t_tracker_ros prepare_sparse_views --help`.
Default values of this script match configuration of the meshes used by [HappyPose](https://github.com/agimus-project/happypose).


## ROS API

### Publishers

- **m3t_tracker/detections** [vision_msgs/msg/Detection2DArray]

    Array containing poses from **reference/detections** topic refined by M3T tracker.

- **m3t_tracker/vision_info** [vision_msgs/msg/VisionInfo]

    Relied information from **reference/vision_info** topic.

### Subscribers

- **color/image** [sensor_msgs/msg/Image]

    Color video stream from a given camera.

- **depth/image** [sensor_msgs/msg/Image]

    Depth video stream from a given camera.

- **color/camera_info** [sensor_msgs/msg/CameraInfo]

    Topic used to obtain intrinsic matrix of the color image sensor.

- **depth/camera_info** [sensor_msgs/msg/CameraInfo]

    Topic used to obtain intrinsic matrix of the depth image sensor.

- **reference/detections** [vision_msgs/msg/Detection2DArray]

    Detections to be used to initialize tracking.

- **reference/vision_info** [vision_msgs/msg/VisionInfo]

    Information about the used pose estimator and URL with object database location.

### Service Servers

- **~/set_paramters** [rcl_interfaces/srv/SetParameters]

    Allows to dynamically change ROS parameters. For more information. For more information, refer to the [documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

### Parameters

Parameters are generated with [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library). Currently, no automatic documentation generation is set up. Refer to [m3t_tracker_ros_parameters.yaml](./m3t_tracker_ros/m3t_tracker_ros_parameters.yaml) for more information.

Note that some of the parameters are possible to tune in the runtime. Refer to the code generation YAML file to see which of them are available.
