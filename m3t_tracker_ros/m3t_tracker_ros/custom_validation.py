import pathlib
from rclpy.parameter import Parameter


def check_object_model_path(param: Parameter) -> str:
    """Checks correctness of the provided path with object models. Checks if the path
    exists and has at lest one object with extension ``.obj`` that could be loaded.

    :param param: ROS parameter with a string containing path to the object models dataset.
    :type param: rclpy.parameter.Parameter
    :return: Error explanation. If empty string, everything is correct.
    :rtype: str
    """
    dataset_path = pathlib.Path(param.value)
    if not dataset_path.is_dir():
        return "Provided path does not exist!"

    if len(list(dataset_path.glob("*.obj"))) == 0:
        return "No files with extension '.obj' found in the provided directory!"
    return


def filename_format_correct(param: Parameter) -> str:
    """Checks if filename format string contains substring ``${class_id}`` sub string
    and ends with ``.${file_fmt}``.

    :param param: ROS parameter with a string containing the filename_format string.
    :type param: rclpy.Parameter
    :return: Error explanation. If empty string, everything is correct.
    :rtype: str
    """
    if not param.value.endswith(".${file_fmt}"):
        return "String '" + param.value + "' does not end with substring '.${file_fmt}'"
    if "${class_id}" not in param.value:
        return "String '" + param.value + "' does not contain substring '${class_id}'"
    return ""


def tracked_objects_have_global(param: Parameter) -> str:
    """Ensures if the list of names contains ``global`` keyword in it.

    :param param:  ROS parameter with an array of strings of tracked objects names.
    :type param: rclpy.parameter.Parameter
    :return: Error explanation. If empty string, everything is correct.
    :rtype: str
    """
    if "global" not in param.value:
        return f"No 'global' config in the '{param.name}' parameters!"
    return ""
