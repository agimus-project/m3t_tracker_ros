import os
from glob import glob
from setuptools import find_packages, setup

package_name = "m3t_tracker_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/dummy_detection_publisher"],
        ),
        (
            "share/ament_index/resource_index/packages",
            ["resource/keyboard_monitor"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guilhem Saurel",
    maintainer_email="guilhem.saurel@laas.fr",
    description="Examples for m3t_tracker_ros package",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dummy_detection_publisher = m3t_tracker_examples.dummy_detection_publisher:main",
            "keyboard_monitor = m3t_tracker_examples.keyboard_monitor:main",
        ],
    },
)
