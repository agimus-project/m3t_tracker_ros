import numbers
import numpy as np
import numpy.typing as npt
import pathlib
from transforms3d.quaternion import quat2mat
from typing import Annotated, Any, Dict, List, Literal, Tuple, Union

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


def update_object_config(
    modality: Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer],
    params_dict: dict,
) -> Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer]:
    """Directly converts code-generated ROS parameters converted into dictionary
    into modality or optimizer data structure configuration.

    :param modality: Region or Depth modality or Optimizer configuration structure.
    :type modality: Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer]
    :param params_dict: Dictionary with ROS parameters extracted for a given modality
        type or optimizer configuration.
    :type params_dict: dict
    :return: Update modality config
    :rtype: Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer]
    """
    for key, val in params_dict.items():
        # Check if the binded object has a parameter
        if hasattr(modality, key):
            # Set new parameter value
            setattr(modality, key, val)


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


def check_dataset_path(
    dataset_path: pathlib.Path, tracked_objects: List[str], use_depth: bool
) -> Dict[str : Tuple[pathlib.Path]]:
    """Assigns paths to files required by tracked objects. hecks if files with
    extensions ``.obj`` and ``.m3t_rmb`` exists for objects specified with the parameter
    ``tracked_objects`` exists. If depth is expected objects with extension ``.m3t_dmb``
    are also checked.

    :param dataset_path: Path to the dataset of objects. With assumption the folder was
        already pre checked in parameter validation.
    :type dataset_path: pathlib.Path
    :param tracked_objects: List of object names to load to track.
    :type tracked_objects: List[str]
    :param use_depth: Whether to expect depth data.
    :type use_depth: bool

    :raises RuntimeError: Failed to math object name with filenames in the directory.
    :return: Dictionary with following structure:
        ``tracked_object_name``: {
            ``obj``: pathlib.Path,
            ``m3t_rmb``: pathlib.Path,
            ``m3t_dmb``: Union[None, pathlib.Path],
        }
        The value for ``m3t_dmb`` is set to none if ``use_depth`` is equal to False
    :rtype: Dict[str : Tuple[pathlib.Path]]
    """

    # Filter only files from the directory
    files_in_dir = [x for x in dataset_path if x.is_file()]
    objects_with_valid_paths = {}
    for object_name in tracked_objects:
        try:
            # ``.obj``: 3D mesh file
            # ``.m3t_rmb``: Precomputed region modality binary file
            # ``.m3t_dmb``: Precomputed depth modality binary file
            file_extensions = ["obj", "m3t_rmb"]
            if use_depth:
                file_extensions.append("m3t_dmb")
            # Find all files matching object name and file extension
            object_files = {
                extension: next(
                    file_name
                    for file_name in files_in_dir
                    if object_name + "." + extension in file_name.as_posix()
                )
                for extension in file_extensions
            }
            objects_with_valid_paths.update({object_name: object_files})
        except StopIteration:
            continue

    if len(objects_with_valid_paths) == 0:
        raise RuntimeError(
            "None of the provided objects could be found in the "
            f"folder '{dataset_path.as_posix()}'. Unable to start the node, exiting!"
        )

    return objects_with_valid_paths
