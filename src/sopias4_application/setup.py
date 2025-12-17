import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sopias4_application"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2",
    maintainer_email="ros2@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gui = sopias4_application.gui:main",
            "controller = sopias4_application.controller:main",
            "layer = sopias4_application.layer:main",
            "planner = sopias4_application.planner:main",
        ],
    },
)
