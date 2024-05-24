import numbers
import numpy as np
import numpy.typing as npt
import pinocchio as pin
from typing import Annotated, Any, List, Literal


from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Transform, Quaternion, Vector3
from vision_msgs.msg import Detection2D, Detection2DArray

from m3t_tracker_ros.cached_tracker import TrackedObject

# Automatically generated file
from m3t_tracker_ros.m3t_tracker_ros_parameters import m3t_tracker_ros  # noqa: E402


def params_to_dict(params: m3t_tracker_ros.Params) -> dict:
    """Converts an object created by generate_parameter_library to a Python dictionary.
    Parameters are converted from 'my_params.foo.bar' to 'my_params["foo"]["bar"]'.

    :param params: Object created by generate_parameter_library with ROS parameters.
    :type params: m3t_tracker_ros.Params
    :return: ROS parameters converted to a dictionary.
    :rtype: dict
    """
    out = {}

    def to_dict_internal(instance: Any, name: str, base_dict: dict) -> None:
        if isinstance(instance, (str, numbers.Number, list)):
            base_dict.update({name: instance})
        else:
            if name != "":
                base_dict.update({name: {}})
            data = [
                attr
                for attr in dir(instance)
                if (
                    not callable(getattr(instance, attr))
                    and not attr.startswith("__")
                    and attr != "stamp_"
                )
            ]
            for attr in data:
                to_dict_internal(
                    getattr(instance, attr),
                    attr,
                    base_dict[name] if name != "" else base_dict,
                )

    to_dict_internal(params, "", out)
    return out


def transform_msg_to_matrix(
    transform: Transform,
) -> Annotated[npt.NDArray[np.float64], Literal[4, 4]]:
    """Converts ROS Transform message into a 4x4 transformation matrix.

    :param transform: Transform message to convert into the matrix.
    :type transform: geometry_msgs.msg.Transform
    :return: Transformation matrix based on the ROS message.
    :rtype: Annotated[npt.NDArray[np.float64], Literal[4, 4]]
    """
    return pin.XYZQUATToSE3(
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ).np


def pose_msg_to_matrix(
    pose: Pose,
) -> Annotated[npt.NDArray[np.float64], Literal[4, 4]]:
    """Converts ROS Transform message into a 4x4 transformation matrix.

    :param pose: Pose message to convert into the matrix.
    :type pose: geometry_msgs.msg.Pose
    :return: Transformation matrix based on the ROS message.
    :rtype: Annotated[npt.NDArray[np.float64], Literal[4, 4]]
    """
    return pin.XYZQUATToSE3(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ).np


def get_tracked_objects(detections: Detection2DArray) -> List[TrackedObject]:
    """Converts Detection2DArray message into a list of TrackedObject

    :param detections: ROS message to convert.
    :type detections: vision_msgs.msg.Detection2DArray
    :return: List of objects to pass to the cashed tracker.
    :rtype: List[m3t_tracker_ros.cached_tracker.TrackedObject]
    """

    def get_tracked_object(detection: Detection2D) -> TrackedObject:
        TrackedObject(
            id=detection.id,
            class_id=detection.results[0].hypothesis.class_id,
            body2camera_pose=pose_msg_to_matrix(detection.results[0].pose.pose),
        )

    return [get_tracked_object(detection) for detection in detections.detections]


def matrix_to_pose(matrix: npt.NDArray[np.float64]) -> Pose:
    """Converts 4x4 transformation matrix into ROS Pose message.

    :param matrix: 4x4 transformation matrix.
    :type matrix: Annotated[npt.NDArray[np.float64], Literal[4, 4]]
    :return: Corresponding to the transformation ROS Pose message.
    :rtype: Pose
    """
    pose_vec = pin.SE3ToXYZQUAT(pin.SE3(matrix))
    return Pose(
        position=Vector3(**dict(zip("xyz", pose_vec[:3]))),
        orientation=Quaternion(**dict(zip("xyzw", pose_vec[3:]))),
    )


def update_detection_poses(
    orig_detection_arr: Detection2DArray,
    tracked_objects: List[TrackedObject],
    new_header: Header,
) -> Detection2DArray:
    """Updates Detection2DArray ROS message detections with new, refined poses from
    TrackedObject list and new header.

    :param orig_detection_arr: Original, input detections array message.
    :type orig_detection_arr: vision_msgs.msg.Detection2DArray
    :param tracked_objects: List of refined poses from cashed tracker.
    :type tracked_objects: List[m3t_tracker_ros.cached_tracker.TrackedObject]
    :param new_header: New header to emplace in the detections.
    :type new_header: std_msgs.msg.Header
    :return: Detection array message with updated fields.
    :rtype: vision_msgs.msg.Detection2DArray
    """
    for i in range(len(orig_detection_arr.detections)):
        orig_detection_arr.detections[i].header = new_header
        orig_detection_arr.detections[i].results[0].pose.pose = matrix_to_pose(
            tracked_objects[i].body2camera_pose
        )

    return orig_detection_arr
