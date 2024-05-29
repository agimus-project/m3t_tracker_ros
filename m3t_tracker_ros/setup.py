from setuptools import find_packages, setup

from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "m3t_tracker_ros"

module_name = "m3t_tracker_ros_parameters"
yaml_file = "m3t_tracker_ros/m3t_tracker_ros_parameters.yaml"
validation_module = "m3t_tracker_ros.custom_validation"
generate_parameter_module(module_name, yaml_file, validation_module=validation_module)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/real_time_tracker_node"],
        ),
        (
            "share/ament_index/resource_index/packages",
            ["resource/time_delay_compensation_node"],
        ),
        (
            "share/ament_index/resource_index/packages",
            ["resource/prepare_sparse_views"],
        ),
        ("share/" + package_name, ["package.xml"]),
        # (os.path.join("share", package_name, "test"), glob("test/*.py")),
        # (os.path.join("share", package_name, "test"), glob("test/*.yaml")),
        # (os.path.join("share", package_name, "test"), glob("test/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guilhem Saurel",
    maintainer_email="guilhem.saurel@laas.fr",
    description="M3T base 3D tracker for real time object tracking with HappyPose library",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "real_time_tracker_node = m3t_tracker_ros.real_time_tracker_node:main",
            "time_delay_compensation_node = m3t_tracker_ros.time_delay_compensation_node:main",
            "prepare_sparse_views = m3t_tracker_ros.prepare_sparse_views:main",
        ],
    },
)
