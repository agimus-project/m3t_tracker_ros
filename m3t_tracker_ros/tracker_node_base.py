import abc
import numpy as np
import numpy.typing as npt
import pathlib
from typing import Union

import pym3t

from transforms3d.quaternion import quat2mat

from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, VisionInfo

# Automatically generated file
from m3t_tracker_ros.m3t_tracker_ros_parameters import m3t_tracker_ros  # noqa: E402


class TrackerNodeBase(Node):
    """Base class for M3T tracker nodes. Provides unified functions and time
    synchronization to apply tracking of incoming images."""

    __metaclass__ = abc.ABCMeta

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        try:
            self._param_listener = m3t_tracker_ros.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # Initialize M3T tracker
        self._dummy_color_camera = pym3t.DummyColorCamera()
        self._dummy_color_camera.camera2world_pose = np.eye(4)
        if self._params.use_depth:
            self._dummy_depth_camera = pym3t.DummyDepthCamera()

        self._tracker = pym3t.Tracker("tracker", synchronize_cameras=False)
        self.bodies, self.links, self.object_files = self.create_bodies(
            self.obj_model_dir, self.accepted_objs, self.cfg.geometry_unit_in_meter
        )

        # Transform buffers
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        # Color subscribers
        filter_subscribers = [
            Subscriber(self, Image, "color/image_raw"),
            Subscriber(self, CameraInfo, "color/camera_info"),
        ]

        # Depth subscribers
        if self._params.use_depth:
            filter_subscribers.extend(
                [
                    Subscriber(self, Image, "depth/image_raw"),
                    Subscriber(self, CameraInfo, "depth/camera_info"),
                ]
            )

        # Create time approximate time synchronization
        image_approx_time_sync = ApproximateTimeSynchronizer(
            filter_subscribers, queue_size=5, slop=self._params.time_sync_slop
        )
        # Register callback depending on the configuration
        image_approx_time_sync.registerCallback(
            self._on_image_data_cb
            if self._params.use_depth
            else self._on_image_data_no_depth_cb
        )

        # Detections subscribers
        detection_approx_time_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "reference/detections"),
                Subscriber(self, VisionInfo, "reference/vision_info"),
            ],
            queue_size=5,
            slop=0.01,
        )
        # Register callback depending on the configuration
        detection_approx_time_sync.registerCallback(self._on_detection_cb)

        # Publishers
        self._detection_pub = self.create_publisher(
            Detection2DArray, "m3t_tracker/detections"
        )
        self._vision_info_pub = self.create_publisher(
            VisionInfo, "m3t_tracker/vision_info"
        )

    def _update_poses(
        self,
        initial_poses: Detection2DArray,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float64],
        depth_image: Union[None, npt.NDArray[np.float64]],
        depth_camera_k: Union[None, npt.NDArray[np.float64]],
    ) -> Detection2DArray:
        """Inputs list of detections and update their poses with
        M3T tracker based on subscribed images.

        :param initial_poses: List poses to refine.
        :type initial_poses: Detection2DArray
        :param color_image: Color image to use when updating.
        :type color_image: Image
        :param color_camera_info:
        :type color_camera_info: CameraInfo
        :param depth_image: _description_
        :type depth_image: Union[None, Image]
        :param depth_camera_info: _description_
        :type depth_camera_info: Union[None, CameraInfo]
        :return: _description_
        :rtype: Detection2DArray
        """
        initial_poses
        color_image
        color_camera_k
        depth_image
        depth_camera_k
        pass

    def _on_image_data_no_depth_cb(
        self, color_image: Image, camera_info: CameraInfo
    ) -> None:
        """Proxy callback function for ``self._on_update`` to use when depth images are
        not subscribed. Passes the color image and camera info for the color image, while
        replacing all the depth info with None vales.

        :param color_image: Received color image.
        :type color_image: sensor_msgs.msg.Image
        :param camera_info: Received info of the color camera.
        :type camera_info: sensor_msgs.msg.CameraInfo
        """
        self._on_update_no_depth(color_image, camera_info, None, None)

    def _on_image_data_cb(
        self,
        color_image: Image,
        camera_info: CameraInfo,
        depth_image: Union[None, Image],
        depth_info: Union[None, CameraInfo],
    ) -> None:
        """Callback triggered every time batch of synchronized image messages is received.
        Checks correctness of received messages, extracts required data fields and
        performs frame conversions.

        :param color_image: Received color image.
        :type color_image: sensor_msgs.msg.Image
        :param camera_info: Received info of the color camera.
        :type camera_info: sensor_msgs.msg.CameraInfo
        :param depth_image: Received depth image.
            If depth is not subscribed returns None.
        :type depth_image: Union[None, sensor_msgs.msg.Image]
        :param depth_info: Received info of the depth camera.
            If depth is not subscribed returns None.
        :type depth_info: Union[None, sensor_msgs.msg.CameraInfo]
        """
        if color_image.header.frame_id != camera_info.header.frame_id:
            self.get_logger().error(
                "Frame IDs for color image and it's camera info don't match!",
                throttle_duration_sec=5.0,
            )
            return

        self._dummy_color_camera.intrinsics = self._camera_info_to_intrinsics(
            depth_info
        )

        if depth_image is not None and depth_info is not None:
            if depth_image.header.frame_id != depth_info.header.frame_id:
                self.get_logger().error(
                    "Frame IDs for depth image and it's camera info don't match!",
                    throttle_duration_sec=5.0,
                )
                return

            image_ts = Time.from_msg(color_image.header.stamp)
            if not self._buffer.can_transform(
                depth_image.header.frame_id, color_image.header.frame_id, image_ts
            ):
                self.get_logger().error(
                    f"No transformation between frames '{depth_image.header.frame_id}' "
                    f"and '{color_image.header.frame_id}'!",
                    throttle_duration_sec=5.0,
                )
                return

            depth_to_color = self._buffer.lookup_transform(
                depth_image.header.frame_id, color_image.header.frame_id, image_ts
            ).transform
            # Convert geometry_msgs/Transform into the transformation matrix
            R = quat2mat(
                [
                    depth_to_color.rotation.w,
                    depth_to_color.rotation.x,
                    depth_to_color.rotation.y,
                    depth_to_color.rotation.z,
                ]
            )
            V = np.array(
                [
                    depth_to_color.translation.x,
                    depth_to_color.translation.y,
                    depth_to_color.translation.z,
                ]
            ).reshape((3, 1))
            # Set this pose to be the relative pose between the cameras
            self._dummy_depth_camera.camera2world_pose = np.dot(R, V)
            self._dummy_depth_camera.intrinsics = self._camera_info_to_intrinsics(
                depth_info
            )

    @abc.abstractmethod
    def _detection_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback triggered every time detection and vision info messages arrive.

        :param detections: Received detections array.
        :type detections: Detection2DArray
        :param detections: Received vision info message.
        :type detections: VisionInfo
        """

    def _camera_info_to_intrinsics(self, camera_info: CameraInfo) -> pym3t.Intrinsics:
        """Converts ROS camera info message into pym3t Intrinsics object.

        :param camera_info: ROS message with camera data fields.
        :type camera_info: sensor_msgs.msg.CameraInfo
        :return: M3T object holding intrinsics parameters of the camera.
        :rtype: pym3t.Intrinsics
        """
        return pym3t.Intrinsics(
            fu=camera_info.k[0],
            fv=camera_info.k[4],
            ppu=camera_info.k[2],
            ppv=camera_info.k[5],
            width=camera_info.width,
            height=camera_info.height,
        )

    def _load_models(self) -> None:
        # Basic checks on the path are performed in ``custom_validation.py``
        dataset_path = pathlib.Path(self._params.dataset_path)
        available_objects = list(dataset_path.glob("*.obj"))
        valid_object_paths = {}
        for object_name in self._params.tracked_objects:
            try:
                object_file = next(
                    file_name
                    for file_name in available_objects
                    if object_name + ".obj" in file_name.as_posix()
                )
                valid_object_paths.update({object_name: object_file})
                self._load_mesh_model(object_file)
            except StopIteration:
                self.get_logger().warn(
                    f"No mesh found matching object name '{object}' "
                    "found. Object creation will be skipped!"
                )
                continue
        if len(valid_object_paths) == 0:
            e = RuntimeError(
                "None of the provided objects could be found in the "
                f"folder '{dataset_path.as_posix()}'. Unable to start the node, exiting!"
            )
            self.get_logger().error(str(e))
            raise e

        # bodies = {
        #     obj_name: pym3t.Body(
        #         name=obj_name,
        #         geometry_path=model_path.as_posix(),
        #         geometry_unit_in_meter=self._params.geometry_unit_in_meter,
        #         geometry_counterclockwise=True,
        #         geometry_enable_culling=True,
        #         geometry2body_pose=np.eye(4),
        #     )
        #     for obj_name, model_path in valid_object_paths.items()
        # }

        # links = {
        #     body_name: pym3t.Link(body_name + "_link", body)
        #     for body_name, body in bodies.items()
        # }
        # TODO initialize bodies
