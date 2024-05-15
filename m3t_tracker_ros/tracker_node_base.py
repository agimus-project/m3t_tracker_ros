import abc
import numpy as np
import numpy.typing as npt
from typing import Union


from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

from vision_msgs.msg import Detection2DArray

# Automatically generated file
from m3t_tracker_ros.m3t_tracker_ros_parameters import m3t_tracker_ros  # noqa: E402


class MinimalSubscriber(Node):
    __metaclass__ = abc.ABCMeta

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        try:
            self._param_listener = m3t_tracker_ros.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # Color subscribers
        self._color_image_sub = self.create_subscription(
            Image, "color/image_raw", self._color_image_cb, 10
        )
        self._color_camera_info_sub = self.create_subscription(
            CameraInfo, "color/camera_info", self._color_camera_info_cb, 10
        )

        # Depth subscribers
        if self._params.use_depth:
            self._depth_image_sub = self.create_subscription(
                Image, "depth/image_raw", self._depth_image_cb, 10
            )
            self._color_camera_info_sub = self.create_subscription(
                CameraInfo, "depth/camera_info", self._depth_camera_info_cb, 10
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

    @abc.abstractmethod
    def _color_image_cb(self, color_image: Image) -> None:
        """Called on every color image received by the node.

        :param color_image: Received color image
        :type color_image: sensor_msgs.msg.Image
        """
        pass

    @abc.abstractmethod
    def _color_camera_info_cb(self, camera_info: CameraInfo) -> None:
        """Called on every color camera info message received.

        :param color_image: Received color image
        :type color_image: sensor_msgs.msg.CameraInfo
        """
        pass

    @abc.abstractmethod
    def _depth_image_cb(self, color_depth: Image) -> None:
        """Called on every depth image received by the node.

        :param color_image: Received depth image
        :type color_image: sensor_msgs.msg.Image
        """
        pass

    @abc.abstractmethod
    def _depth_camera_info_cb(self, camera_info: CameraInfo) -> None:
        """Called on every depth camera info message received.

        :param color_image: Received info of a depth camera
        :type color_image: sensor_msgs.msg.CameraInfo
        """
        pass
