import os
from glob import glob
from setuptools import setup, find_packages

package_name = "graphics"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    url="https://www.kiwibot.com/",
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JohnBetaCode",
    maintainer_email="john@kiwibot.com",
    description="Hello! you shouldn't be here",
    license="Apache License 2.0",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "graphics = graphics.node_visual_gui:main",
        ],
    },
)
