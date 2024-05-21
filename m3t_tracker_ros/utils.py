import numbers
import numpy as np
import numpy.typing as npt
from transforms3d.quaternion import quat2mat
from typing import Annotated, Any, Literal

import pym3t

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Transform

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


def camera_info_to_intrinsics(camera_info: CameraInfo) -> pym3t.Intrinsics:
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


def transform_msg_to_matrix(
    transform: Transform,
) -> Annotated[npt.NDArray[np.float64], Literal[4, 4]]:
    """Converts ROS Transform message into a 4x4 transformation matrix.

    :param transform: Transform message to convert into the matrix.
    :type transform: Transform
    :return: Transformation matrix based on the ROS message.
    :rtype: Annotated[npt.NDArray[np.float64], Literal[4, 4]]
    """
    R = quat2mat(
        [
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
        ]
    )
    V = np.array(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
    ).reshape((3, 1))
    return np.dot(R, V)
