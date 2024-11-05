import numpy as np
import numpy.typing as npt
from typing import Union

import rclpy
from rclpy.time import Time

from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, VisionInfo

from m3t_tracker_ros.image_time_buffer import ImageQueueData, ImageTimeBuffer
from m3t_tracker_ros.tracker_node_base import TrackerNodeBase


class TimeCatchupNode(TrackerNodeBase):
    """Specialized M3T tracker ROS node class for time compensation for slow 6D pose
    estimation pipelines with pose refinement. Stores incoming camera images waiting for
    new detections to be refined.
    """

    def __init__(self, **kwargs):
        """Initializes base tracker node and creates
        buffer with a counter to store incoming images."""
        super().__init__(node_name="m3t_time_catchup_node", **kwargs)

        self._image_buffer = ImageTimeBuffer(self, self._params.image_timeout)

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
        Adds the images to the buffer for later processing.

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

        self._image_buffer.append(
            ImageQueueData(
                Time.from_msg(camera_header.stamp),
                camera_header.frame_id,
                color_image,
                color_camera_k,
                depth_image,
                depth_camera_k,
                depth2color_pose,
            )
        )

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback triggered every time synchronized detection and vision info
        messages arrive. Loops over saved images and refines their poses to compensate
        for already passed time.

        :param detections: Received detections array.
        :type detections: vision_msgs.msg.Detection2DArray
        :param detections: Received vision info message.
        :type detections: vision_msgs.msg.VisionInfo
        """

        self._refresh_parameters()

        # No images to refine. Do not compensate in the time.
        if len(self._image_buffer) == 0:
            return

        if self._check_image_time_too_new(detections, self._image_buffer[0].stamp):
            self.get_logger().warn(
                "Time difference between detections and first image in the buffer "
                "is too big. Tracker won't be able to start tracking!"
            )
            return

        if self._check_image_time_too_old(detections, self._image_buffer[-1].stamp):
            self.get_logger().warn(
                "Time difference between detections and last image in the buffer "
                "is too big. All of the images are too old!"
            )
            return

        tracked_objects = detections

        update_detections = True
        start_time = Time.from_msg(tracked_objects.header.stamp)
        # Loop over saved images
        for im_data in self._image_buffer.loop_from_point(start_time):
            try:
                tracked_objects = self._perform_tracking_step(
                    im_data.stamp,
                    im_data.frame_id,
                    im_data.color_image,
                    im_data.color_camera_k,
                    im_data.depth_image,
                    im_data.depth_camera_k,
                    im_data.depth2color_pose,
                    tracked_objects,
                    update_detections,
                )
                update_detections = False
            except RuntimeError:
                pass

        # Reset the buffer counter
        self._buffer_cnt = 0

        header = Header(
            frame_id=im_data.frame_id,
            stamp=self.get_clock().now().to_msg(),
        )

        # Objects are now considered to be relevant at the time
        # of the color image and in the color camera frame
        tracked_objects.header = header

        vision_info.header = header
        # Append info to the method that the poses are now refined with the tracker
        vision_info.method += "-M3T-time-compensated"

        self._detection_pub.publish(tracked_objects)
        self._vision_info_pub.publish(vision_info)


def main() -> None:
    """Creates the ROS node object and spins it."""
    rclpy.init()
    time_catchup_node = TimeCatchupNode()
    try:
        rclpy.spin(time_catchup_node)
    except KeyboardInterrupt:
        pass
    time_catchup_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
