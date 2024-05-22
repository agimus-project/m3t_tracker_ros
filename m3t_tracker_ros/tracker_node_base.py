import abc
import numpy as np
import numpy.typing as npt
from typing import Union

from rclpy.node import Node
from rclpy.time import Time

from cv_bridge import CvBridge

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from message_filters import ApproximateTimeSynchronizer, Subscriber

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, VisionInfo

from m3t_tracker_ros.cached_tracker import CachedTracker
from m3t_tracker_ros.utils import (
    camera_info_to_intrinsics,
    transform_msg_to_matrix,
)

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

        # M3T tracker
        self._tracker = CachedTracker(self._params)

        # Image type converter
        self._cvb = CvBridge()

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

    @abc.abstractmethod
    def _image_data_cb(
        self,
        camera_header: Header,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float64],
        depth_image: Union[None, npt.NDArray[np.float16]],
        depth_camera_k: Union[None, npt.NDArray[np.float64]],
        depth2color_pose: Union[npt.NDArray[np.float32], None],
    ) -> Detection2DArray:
        """Inputs list of detections and update their poses with
        M3T tracker based on subscribed images.

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

        intrinsics_rgb = camera_info_to_intrinsics(depth_info)
        encoding = "passthrough" if color_image.encoding == "rgb8" else "rgb8"
        image_rgb = self._cvb.imgmsg_to_cv2(color_image, encoding)

        image_ts = Time.from_msg(color_image.header.stamp)
        if depth_image is not None and depth_info is not None:
            if depth_image.header.frame_id != depth_info.header.frame_id:
                self.get_logger().error(
                    "Frame IDs for depth image and it's camera info don't match!",
                    throttle_duration_sec=5.0,
                )
                return

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
            # Set this pose to be the relative pose between the cameras
            transform_depth = transform_msg_to_matrix(depth_to_color)
            intrinsics_depth = camera_info_to_intrinsics(depth_info)

            # https://ros.org/reps/rep-0118.html
            # Depth images are published as sensor_msgs/Image encoded as 32-bit float.
            # Each pixel is a depth (along the camera Z axis) in meters.
            # ROS 2 topic echo shows, clearly 16UC1, with depth scale 16
            # ICG expects a CV_16UC1.
            encoding = "passthrough" if depth_image.encoding == "16UC1" else "16UC1"
            image_depth = self._cvb.imgmsg_to_cv2(depth_image, encoding)
        else:
            transform_depth = None
            intrinsics_depth = None
            image_depth = None

        self._image_data_cb(
            color_image.header,
            image_rgb,
            intrinsics_rgb,
            image_depth,
            intrinsics_depth,
            transform_depth,
        )

    @abc.abstractmethod
    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback triggered every time detection and vision info messages arrive.

        :param detections: Received detections array.
        :type detections: Detection2DArray
        :param detections: Received vision info message.
        :type detections: VisionInfo
        """
        pass

    def _refresh_parameters(self) -> None:
        """Checks if parameter change occurred and if needed reloads the tracker.

        :raises RuntimeError: Raised when tracker setup failed.
        """
        if self._param_listener.is_old(self._params):
            # Update parameters
            self._param_listener.refresh_dynamic_parameters()
            self._params = self._param_listener.get_params()

            # Reinitialize the tracker
            self._tracker.update_params(self._params)

    def _transform_detections_to_camera(
        self,
        detections: Detection2DArray,
        camera_header: Header,
    ) -> Detection2DArray:
        # If camera frame is used as a stationary frame its
        # motion will not be taken into account
        stationary_frame = (
            self._params.camera_motion_stationary_frame_id
            if self._params.compensate_camera_motion
            else camera_header.frame_id
        )

        camera_stamp = Time.from_msg(camera_header.stamp)
        for i in len(range(detections)):
            # Transform poses of the objects to account for a moving camera
            # Additionally change frame in which those objects are represented
            detection_header = detections[i].results[0].header
            self._buffer.lookup_transform_full(
                stationary_frame,
                camera_stamp,
                detection_header.frame_id,
                Time.from_msg(detection_header.stamp),
                stationary_frame,
            )
            # Update their poses
            detections[i].results[0].pose.pose = do_transform_pose(
                detections[i].results[0].results[0].pose.pose
            )
            detections[i].header = camera_header
        detections.header = camera_header
        return detections
