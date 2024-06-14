from typing import Union
import numpy as np
import numpy.typing as npt

import rclpy

from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, VisionInfo

from m3t_tracker_ros.tracker_node_base import TrackerNodeBase


class RealTimeTrackerNode(TrackerNodeBase):
    """Specialized M3T tracker ROS node class for real time tracking.
    Refines poses of detected objects every time it receives a new image.
    """

    def __init__(self, *args, **kwargs) -> None:
        """Initializes base tracker node and creates
        variables used for real time tracking."""
        super().__init__(node_name="m3t_rt_tracker_node", **kwargs)

        self._last_vision_info = VisionInfo()
        self._tracked_objects = None
        self._new_estimates_arrived = False

        self.get_logger().info("Node started.")

    def _image_data_cb(
        self,
        camera_header: Header,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float64],
        depth_image: Union[None, npt.NDArray[np.float16]],
        depth_camera_k: Union[None, npt.NDArray[np.float64]],
        depth2color_pose: Union[None, npt.NDArray[np.float32]],
    ) -> None:
        """Callback triggered when new, synchronized set of images arrived.
        Refines poses of known objects with the incoming images.

        :param camera_header: Header with color image time stamp and its frame id.
        :type camera_header: std_msgs.msg.Header
        :param color_image: OpenCV style RBG8 color image.
        :type color_image: npt.NDArray[np.uint8]
        :param color_camera_k: Matrix with intrinsic parameters of the color camera.
        :type color_camera_k: npt.NDArray[np.float64]
        :param depth_image: OpenCV style CV_16UC1 depth image. None if not used.
        :type depth_image: Union[None, npt.NDArray[np.float16]]
        :param depth_camera_k:  Matrix with intrinsic parameters of the depth camera.
            None if not used.
        :type depth_camera_k: Union[None, npt.NDArray[np.float64]]
        :param depth2color_pose: Pose between depth camera and color camera.
            None if not used.
        :type depth2color_pose: Union[None, npt.NDArray[np.float32]]
        """

        self._refresh_parameters()

        # Skip if no detections can be refined
        if self._tracked_objects is None:
            return

        if not self._check_image_time_ok(self._tracked_objects, camera_header):
            # Reset tracked objects to prevent the log from showing all the time
            self._tracked_objects = None
            self.get_logger().warn(
                "Time difference between detections and the incoming "
                "image is to big for it to recover!"
            )
            return

        try:
            self._tracked_objects = self._perform_tracking_step(
                camera_header,
                color_image,
                color_camera_k,
                depth_image,
                depth_camera_k,
                depth2color_pose,
                self._tracked_objects,
                self._new_estimates_arrived,
            )
        except Exception as err:
            self.get_logger().warn(str(err))
            return
        self._new_estimates_arrived = False

        header = Header(
            frame_id=camera_header.frame_id, stamp=self.get_clock().now().to_msg()
        )
        # Objects are now considered to be relevant at the time
        # of the color image and in the color camera frame
        self._tracked_objects.header = header

        self._last_vision_info.header = header
        # Append info to the method that the poses are now refined with the tracker
        self._last_vision_info.method += "-M3T-rt-refined"

        self._detection_pub.publish(self._tracked_objects)
        self._vision_info_pub.publish(self._last_vision_info)

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback triggered every time synchronized detection and vision info
        messages arrive. Changes list of currently tracked objects and indicates the
        change occurred.

        :param detections: Received detections array.
        :type detections: vision_msgs.msg.Detection2DArray
        :param detections: Received vision info message.
        :type detections: vision_msgs.msg.VisionInfo
        """
        self._last_vision_info = vision_info
        self._tracked_objects = detections
        self._new_estimates_arrived = True


def main() -> None:
    """Creates the ROS node object and spins it."""
    rclpy.init()
    real_time_tracker_node = RealTimeTrackerNode()
    try:
        rclpy.spin(real_time_tracker_node)
    except KeyboardInterrupt:
        pass
    real_time_tracker_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
