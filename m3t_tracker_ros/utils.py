import numbers
from typing import Any, Union

import pym3t

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


def update_modality_config(
    modality: Union[pym3t.RegionModality, pym3t.DepthModality], params_dict: dict
) -> None:
    """Directly converts code-generated ROS parameters converted into dictionary
    into modality data structure configuration.

    :param modality: Region or Depth modality configuration structure.
    :type modality: Union[pym3t.RegionModality, pym3t.DepthModality]
    :param params_dict: Dictionary with ROS parameters extracted for a given modality type.
    :type params_dict: dict
    """
    for key, val in params_dict.items():
        # Check if the binded object has a parameter
        if hasattr(modality, key):
            # Set new parameter value
            setattr(modality, key, val)
