from dataclasses import dataclass
from typing import Union
import numpy as np
import numpy.typing as npt

import rclpy

from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, VisionInfo

from m3t_tracker_ros.tracker_node_base import TrackerNodeBase


@dataclass
class ImageQueueData:
    """Class storing all data relevant to the tracked images.
    Contains time stamp, color image and optional depth image.
    """

    camera_header: Header
    color_image: npt.NDArray[np.uint8]
    color_camera_k: npt.NDArray[np.float64]
    depth_image: Union[None, npt.NDArray[np.float16]]
    depth_camera_k: Union[None, npt.NDArray[np.float64]]
    depth2color_pose: Union[None, npt.NDArray[np.float32]]


class TimeDelayCompensationNode(TrackerNodeBase):
    """Specialized M3T tracker ROS node class for time compensation for slow 6D pose
    estimation pipelines with pose refinement. Stores incoming camera images waiting for
    new detections to be refined.
    """

    def __init__(self, **kwargs):
        """Initializes base tracker node and creates
        buffer with a counter to store incoming images."""
        super().__init__(name="m3t_time_delay_tracker_node", **kwargs)

        # Initial queue size
        self._image_buffer = [ImageQueueData()] * 30
        self._buffer_cnt = 0

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
        im_data = ImageQueueData(
            camera_header,
            color_image,
            color_camera_k,
            depth_image,
            depth_camera_k,
            depth2color_pose,
        )
        # If expansion of the list is not needed, just use existing memory
        if self._buffer_cnt < len(self._image_buffer):
            self._image_buffer[self._buffer_cnt] = im_data
        # Expand it only if needed
        else:
            self._image_buffer.append(self._buffer_cnt)
        self._buffer_cnt += 1

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
        # No images to refine. Do not compensate in the time.
        if self._buffer_cnt == 0:
            return

        if self._check_image_time_too_new(
            self._tracked_objects, self._image_buffer[0].camera_header
        ):
            self.get_logger().warn(
                "Time difference between detections and first image in the buffer "
                "is too big. Tracker won't be able to start tracking!"
            )
            return

        if self._check_image_time_too_old(
            self._tracked_objects, self._image_buffer[self._buffer_cnt].camera_header
        ):
            self.get_logger().warn(
                "Time difference between detections and last image in the buffer "
                "is too big. All of the images are too old!"
            )
            return

        tracked_objects = detections
        skipped_images = 0
        # Loop over saved images
        for i in range(self._buffer_cnt):
            im_data = self._image_buffer[i]

            # Check
            if not self._check_image_time_ok(
                self._tracked_objects, im_data.camera_header
            ):
                skipped_images += 1
                continue

            try:
                tracked_objects = self._perform_tracking_step(
                    im_data.camera_header,
                    im_data.color_image,
                    im_data.color_camera_k,
                    im_data.depth_image,
                    im_data.depth_camera_k,
                    im_data.depth2color_pose,
                    tracked_objects,
                    i == 0,
                )
            except RuntimeError:
                pass

        if skipped_images == self._buffer_cnt:
            self.get_logger().warn(
                "Time difference between detections and all images in the buffer "
                "was too big for the tracker to start tracking!"
            )
            return

        # Store histograms detections with known id
        self._tracker.update_tracked_objects(self._tracked_objects)

        # Reset the buffer counter
        self._buffer_cnt = 0

        header = Header(
            frame_id=im_data.camera_header.frame_id,
            stamp=self.get_clock().now().to_msg(),
        )

        # Objects are now considered to be relevant at the time
        # of the color image and in the color camera frame
        tracked_objects.header = header

        vision_info.header = header
        # Append info to the method that the poses are now refined with the tracker
        vision_info.method += "-M3T-time-compensated"

        self._detection_pub.publish(tracked_objects)
        self._vision_info_pub(vision_info)


def main() -> None:
    """Creates the ROS node object and spins it."""
    rclpy.init()
    time_delay_compensation_node = TimeDelayCompensationNode()
    try:
        rclpy.spin(time_delay_compensation_node)
    except KeyboardInterrupt:
        pass
    time_delay_compensation_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
