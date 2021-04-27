import os
from glob import glob
from setuptools import setup, find_packages


package_name = "path_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JohnBetaCode",
    maintainer_email="john@kiwibot.com",
    description="Hello! you shouldn't be here",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "path_planner = path_planner.node_planner:main",
        ],
    },
)
