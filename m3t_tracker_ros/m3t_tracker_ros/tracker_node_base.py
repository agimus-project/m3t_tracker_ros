import abc
import numpy as np
import numpy.typing as npt
from typing import List, Union

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Time, CONVERSION_CONSTANT

from cv_bridge import CvBridge

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from message_filters import ApproximateTimeSynchronizer, Subscriber

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, VisionInfo

from m3t_tracker_ros.specialized_tracker import SpecializedTracker
from m3t_tracker_ros.utils import (
    get_tracked_objects,
    transform_msg_to_matrix,
    update_detection_poses,
    params_to_dict,
)

# Automatically generated file
from m3t_tracker_ros.m3t_tracker_ros_parameters import m3t_tracker_ros  # noqa: E402


class TrackerNodeBase(Node):
    """Base class for M3T tracker nodes. Provides unified functions and time
    synchronization to apply tracking of incoming images."""

    __metaclass__ = abc.ABCMeta

    def __init__(self, spin_tf_in_thread: bool = True, **kwargs) -> None:
        """Initializes ROS node object, creates synchronized subscribers and publisher.

        :raises Exception: Initialization of the generate_parameter_library object failed.
        """
        super().__init__(**kwargs)

        # Expect the callbacks to work synchronous even with multi-threaded executor
        self.group = MutuallyExclusiveCallbackGroup()

        try:
            self._param_listener = m3t_tracker_ros.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # M3T tracker
        try:
            self._tracker = SpecializedTracker(params_to_dict(self._params))
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # Image type converter
        self._cvb = CvBridge()

        # Transform buffers
        self._buffer = Buffer()
        self._listener = TransformListener(
            self._buffer, self, spin_thread=spin_tf_in_thread
        )

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
        detection_approx_time_sync.registerCallback(self._detection_data_cb)

        # Publishers
        self._detection_pub = self.create_publisher(
            Detection2DArray, "m3t_tracker/detections", 10
        )
        self._vision_info_pub = self.create_publisher(
            VisionInfo, "m3t_tracker/vision_info", 10
        )

    @abc.abstractmethod
    def _image_data_cb(
        self,
        camera_header: Header,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float64],
        depth_image: Union[None, npt.NDArray[np.uint16]],
        depth_camera_k: Union[None, npt.NDArray[np.float64]],
        depth2color_pose: Union[None, npt.NDArray[np.float32]],
    ) -> Detection2DArray:
        """Callback triggered when new synchronized set of images arrived.

        :param camera_header: Header with color image time stamp and its frame id.
        :type camera_header: std_msgs.msg.Header
        :param color_image: OpenCV style RBG8 color image.
        :type color_image: npt.NDArray[np.uint8]
        :param color_camera_k: Matrix with intrinsic parameters of the color camera.
        :type color_camera_k: npt.NDArray[np.float64]
        :param depth_image: OpenCV style CV_16UC1 depth image. None if not used.
        :type depth_image: Union[None, npt.NDArray[np.uint16]]
        :param depth_camera_k:  Matrix with intrinsic parameters of the depth camera.
            None if not used.
        :type depth_camera_k: Union[None, npt.NDArray[np.float64]]
        :param depth2color_pose: Pose between depth camera and color camera.
            None if not used.
        :type depth2color_pose: Union[None, npt.NDArray[np.float32]]
        """

    def _on_image_data_no_depth_cb(
        self, color_image: Image, camera_info: CameraInfo
    ) -> None:
        """Proxy callback function for ``self._on_image_data_cb`` to use when depth
        images are not subscribed. Passes the color image and camera info for the color
        image, while replacing all the depth info with None vales.

        :param color_image: Received color image.
        :type color_image: sensor_msgs.msg.Image
        :param camera_info: Received info of the color camera.
        :type camera_info: sensor_msgs.msg.CameraInfo
        """
        self._on_image_data_cb(color_image, camera_info, None, None)

    def _on_image_data_cb(
        self,
        color_image: Image,
        camera_info: CameraInfo,
        depth_image: Union[None, Image],
        depth_info: Union[None, CameraInfo],
    ) -> None:
        """Callback triggered every time batch of synchronized image messages is received.
        Checks correctness of received messages, extracts required data fields and
        performs frame and image type conversions.

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

        intrinsics_rgb = camera_info.k
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
                self.get_logger().warn(
                    f"Failed to transform from frame '{depth_image.header.frame_id}' "
                    f"to '{color_image.header.frame_id}'!",
                    throttle_duration_sec=5.0,
                )
                return

            depth_to_color = self._buffer.lookup_transform(
                depth_image.header.frame_id, color_image.header.frame_id, image_ts
            ).transform
            # Set this pose to be the relative pose between the cameras
            transform_depth = transform_msg_to_matrix(depth_to_color)
            intrinsics_depth = depth_info.k

            # https://ros.org/reps/rep-0118.html
            # Depth images are published as sensor_msgs/Image encoded as 32-bit float.
            # Each pixel is a depth (along the camera Z axis) in meters.
            # ROS 2 topic echo shows, clearly 16UC1, with depth scale 16
            # ICG expects a CV_16UC1.
            encoding = "passthrough" if depth_image.encoding == "16UC1" else "16UC1"
            image_depth = self._cvb.imgmsg_to_cv2(depth_image, encoding)
            # pym3t type caster does not accept non-recasted uint16 when using ``passthrough``
            if encoding == "passthrough":
                image_depth = image_depth.astype(np.uint16)
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
        """Callback triggered every time synchronized detection
        and vision info messages arrive.

        :param detections: Received detections array.
        :type detections: vision_msgs.msg.Detection2DArray
        :param detections: Received vision info message.
        :type detections: vision_msgs.msg.VisionInfo
        """
        pass

    def _refresh_parameters(self) -> None:
        """Checks if parameter change occurred and if needed reloads the tracker."""
        if self._param_listener.is_old(self._params):
            # Update parameters
            self._param_listener.refresh_dynamic_parameters()
            self._params = self._param_listener.get_params()

            # Reinitialize the tracker
            self._tracker.update_params(params_to_dict(self._params))

    def _perform_tracking_step(
        self,
        camera_header: Header,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float64],
        depth_image: Union[None, npt.NDArray[np.uint16]],
        depth_camera_k: Union[None, npt.NDArray[np.float64]],
        depth2color_pose: Union[None, npt.NDArray[np.float32]],
        tracked_objects: Detection2DArray,
        new_detection: bool = False,
    ) -> Detection2DArray:
        """Performs tracking step. Performs basic check of correctness of the passed
        detections. If ``update_tracked_objects`` is set to ``True`` updates poses of the
        tracked objects inside of the CashedTracker class. If configured, updates poses
        of the objects with changes in the kinematic model of the robot.

        :param camera_header: Header associated with color camera used to obtain its
            frame id and timestamp for spatial conversions.
        :type camera_header: std_msgs.msg.Header
        :param color_image: OpenCV style RBG8 color image.
        :type color_image: npt.NDArray[np.uint8]
        :param color_camera_k: Matrix with intrinsic parameters of the color camera.
        :type color_camera_k: npt.NDArray[np.float64]
        :param depth_image: OpenCV style CV_16UC1 depth image. None if not used.
        :type depth_image: Union[None, npt.NDArray[np.uint16]]
        :param depth_camera_k:  Matrix with intrinsic parameters of the depth camera.
            None if not used.
        :type depth_camera_k: Union[None, npt.NDArray[np.float64]]
        :param depth2color_pose: Pose between depth camera and color camera.
            None if not used.
        :type depth2color_pose: Union[None, npt.NDArray[np.float32]]
        :param tracked_objects: Array of known objects to refine their poses with the tracker.
        :type tracked_objects: vision_msgs.msg.Detection2DArray
        :param new_detection: Flag indicating change of tracked objects. Passed to cashed
            tracker to update its internal structures.
        :type new_detection: bool, optional
        :raises RuntimeError: Passed objects to track are an empty array.
        :return: Array of tracked objects with refined poses and updated timestamps.
        :rtype: Detection2DArray
        """
        if len(tracked_objects.detections) == 0:
            raise RuntimeError("No objects to track")

        obj_counter = {obj: 0 for obj in self._params.tracked_objects}

        def _filter_and_log(idx: int, detection: Detection2D) -> bool:
            class_id = detection.results[0].hypothesis.class_id
            if class_id not in self._params.tracked_objects:
                self.get_logger().warn(
                    f"Unknown class id '{class_id}' for detection at index '{idx}'. "
                    "Detection discarded!",
                    throttle_duration_sec=5.0,
                )
                return False

            obj_counter[class_id] += 1
            if obj_counter[class_id] > self._params.get_entry(class_id).max_instances:
                self.get_logger().warn(
                    f"Class '{class_id}' reached its maximum number of instances to be "
                    f"tracked at the same time. Discarding detection at index '{idx}'.",
                    throttle_duration_sec=5.0,
                )
                return False

            if detection.id == "":
                self.get_logger().warn(
                    f"Detection of a class '{class_id}' "
                    f"at index '{idx}' was discarded due to empty 'id' filed.",
                    throttle_duration_sec=5.0,
                )
                return False
            return True

        # Filter only the objects with the tracks
        tracked_objects.detections = [
            obj
            for i, obj in enumerate(tracked_objects.detections)
            if _filter_and_log(i, obj)
        ]

        if len(tracked_objects.detections) == 0:
            raise RuntimeError("All new detections were discarded!")

        if new_detection or self._params.compensate_camera_motion:
            # If camera frame is used as a stationary frame its
            # motion will not be taken into account
            stationary_frame = (
                self._params.camera_motion_stationary_frame_id
                if self._params.compensate_camera_motion
                else camera_header.frame_id
            )

            camera_stamp = Time.from_msg(camera_header.stamp)

            def _transform_frame(detection: Detection2D) -> Detection2D:
                # Transform poses of the objects to account for a moving camera
                # Additionally change frame in which those objects are represented
                # Update their poses
                detection_header = detection.header
                try:
                    transform = self._buffer.lookup_transform_full(
                        camera_header.frame_id,
                        camera_stamp,
                        detection_header.frame_id,
                        Time.from_msg(detection_header.stamp),
                        stationary_frame,
                    )
                    detection.results[0].pose.pose = do_transform_pose(
                        detection.results[0].pose.pose, transform
                    )
                except Exception as err:
                    self.get_logger().warn(
                        "Failed to compensate motion for object with "
                        f"id '{detection.id}'. Error: '{str(err)}'"
                    )
                return detection

            tracked_objects.detections = [
                _transform_frame(detection) for detection in tracked_objects.detections
            ]

            tracked_objects.header.frame_id = camera_header.frame_id
            tracked_objects.header.stamp = camera_header.stamp

            self._tracker.update_tracked_objects(get_tracked_objects(tracked_objects))

        # Perform tracking step
        tracking_results = self._tracker.track_image(
            color_image,
            color_camera_k,
            depth_image,
            depth_camera_k,
            depth2color_pose,
        )

        return update_detection_poses(tracked_objects, tracking_results, camera_header)

    def _check_image_time_ok(self, *args) -> bool:
        """Checks timestamps of all detections to see if they are within time window
        for the tracker to properly recover and track objects.

        :param *args: See ``self._detection_time_delta``.
        :return: Indication if the tracker will be able to recover and track the objects.
        :rtype: bool
        """
        delta = self._params.detection_to_image_time_slop * CONVERSION_CONSTANT
        return any(abs(diff) < delta for diff in self._detection_time_delta(*args))

    def _check_image_time_too_old(self, *args) -> bool:
        """Checks timestamps of all detections to see if the image is not too old for
        the tracker to properly recover and track them.

        :param *args: See ``self._detection_time_delta``.
        :return: True if image is too old, false if it is ok or newer.
        :rtype: bool
        """
        delta = self._params.detection_to_image_time_slop * CONVERSION_CONSTANT
        return any(diff > delta for diff in self._detection_time_delta(*args))

    def _check_image_time_too_new(self, *args) -> bool:
        """Checks timestamps of all detections to see if the image is not too new for
        the tracker to properly recover and track them.

        :param *args: See ``self._detection_time_delta``.
        :return: True if image is too new, false if it is ok or older.
        :rtype: bool
        """
        delta = self._params.detection_to_image_time_slop * CONVERSION_CONSTANT
        return any(diff > delta for diff in self._detection_time_delta(*args))

    def _detection_time_delta(
        self, detections: Detection2DArray, image_header: Header
    ) -> List[int]:
        """Computes time differences between image time stamp and all
        detections in the array.

        :param detections: Array of detections to check for timestamp.
        :type detections: vision_msgs.msg.Detection2DArray
        :param image_header: Image header used to obtain the timestamp.
        :type image_header: std_msgs.msg.Header
        :return: List of time differences [nanoseconds] between detection timestamps
            and image time stamp.
        :rtype: List[int]
        """
        image_stamp = Time.from_msg(image_header.stamp)
        return [
            (Time.from_msg(det.header.stamp) - image_stamp).nanoseconds
            for det in detections.detections
        ]
