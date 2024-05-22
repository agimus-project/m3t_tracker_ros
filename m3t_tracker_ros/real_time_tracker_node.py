from typing import Union
import numpy as np
import numpy.typing as npt

import rclpy

from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, VisionInfo

from m3t_tracker_ros.utils import get_tracked_objects, update_detection_poses

from m3t_tracker_ros.tracker_node_base import TrackerNodeBase


class RealTimeTrackerNode(TrackerNodeBase):
    def __init__(self, **kwargs):
        super().__init__(name="real_time_tracker_node", **kwargs)

        self._last_vision_info = VisionInfo()
        self._tracked_objects = Detection2DArray()
        self._refined_objects = Detection2DArray()
        self._new_estimates_arrived = False

    def _image_data_cb(
        self,
        camera_header: Header,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float64],
        depth_image: Union[None, npt.NDArray[np.float16]],
        depth_camera_k: Union[None, npt.NDArray[np.float64]],
        depth2color_pose: Union[npt.NDArray[np.float32], None],
    ) -> None:
        header = Header(
            frame_id=camera_header.frame_id, stamp=self.get_clock().now().to_msg()
        )
        # Objects are now considered to be relevant at the time
        # of the color image and in the color camera frame
        self._tracked_objects.header = header

        self._last_vision_info.header = header
        # Append info to the method that the poses are now refined with the tracker
        self._last_vision_info.method += "-M3T-rt-refined"

        # No tracked objects mean empty message to be published
        if len(self._tracked_objects.detections) == 0:
            # Relay empty messages
            self._detection_pub.publish(self._tracked_objects)
            self._vision_info_pub(self._last_vision_info)
            return

        if self._new_estimates_arrived or self._params.compensate_camera_motion:
            # Convert detections to given camera's frame
            # and optionally apply motion model
            self._tracked_objects = self._transform_detections_to_camera(
                self._tracked_objects, camera_header
            )

            # Update list of tracked objects
            self._tracker.update_tracked_objects(
                get_tracked_objects(self._tracked_objects, self._dataset_name)
            )

        self._new_estimates_arrived = False
        # Perform tracking step
        tracking_results = self._tracker.track_image(
            color_image, color_camera_k, depth_image, depth_camera_k, depth2color_pose
        )

        # Update poses of the tracked objects
        self._tracked_objects = update_detection_poses(
            self._tracked_objects, tracking_results, camera_header
        )

        self._detection_pub.publish(self._tracked_objects)
        self._vision_info_pub(self._last_vision_info)

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        self._last_vision_info = vision_info
        self._tracked_objects = detections
        self._new_estimates_arrived = True


def main() -> None:
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
