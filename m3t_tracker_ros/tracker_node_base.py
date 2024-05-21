import abc
import numpy as np
import numpy.typing as npt
from typing import Union


from rclpy.node import Node
from rclpy.time import Time

from cv_bridge import CvBridge

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, VisionInfo

from m3t_tracker_ros.utils import (
    camera_info_to_intrinsics,
    params_to_dict,
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

        intrinsics_rgb = camera_info_to_intrinsics(depth_info)
        encoding = "passthrough" if color_image.encoding == "rgb8" else "rgb8"
        image_rgb = self._cvb.imgmsg_to_cv2(color_image, encoding)

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
        image_rgb
        intrinsics_rgb

        intrinsics_depth
        image_depth
        transform_depth

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

    def _refresh_parameters(self) -> None:
        """Checks if parameter change occurred and if needed reloads the tracker.

        :raises RuntimeError: Raised when tracker setup failed.
        """
        if self._param_listener.is_old(self._params):
            # Update parameters
            self._param_listener.refresh_dynamic_parameters()
            self._params = self._param_listener.get_params()

            # Convert them to a dictionary
            self._params_dict = params_to_dict(self._params)

            # Reinitialize the tracker
            self._tracker = self._initialize_tracker(self._objects_with_valid_paths)
            if not self.tracker.SetUp():
                e = RuntimeError("Failed to initialize tracker!")
                self.get_logger().error(str(e))
                raise e
