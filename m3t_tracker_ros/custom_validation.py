import pathlib
from rclpy.parameter import Parameter


def check_object_model_path(param: Parameter) -> str:
    """Checks correctness of the provided path with object models. Checks if the path
    exists and has at lest one object with extension ``.obj`` that could be loaded.

    :param param: ROS parameter with a string containing path to the object models dataset.
    :type param: rclpy.Parameter
    :return: Error explanation. If empty string, everything is correct.
    :rtype: str
    """
    dataset_path = pathlib.Path(param.value)
    if not dataset_path.is_dir():
        return "Provided path does not exist!"

    if len(list(dataset_path.glob("*.obj"))) == 0:
        return "No files with extension '.obj' found in the provided directory!"
    return
