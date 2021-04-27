#!/usr/bin/env python3

# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Examples:
    https://github.com/ros2/launch/blob/a89671962220c8691ea4f128717bca599c711cda/launch/examples/launch_counters.py

"""

# =============================================================================
import inspect
import yaml
import sys
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

from typing import cast

# ============================================================================
class bcolors:
    LOG = {
        "WARN": ["\033[33m", "WARN"],
        "ERROR": ["\033[91m", "ERROR"],
        "OKGREEN": ["\033[32m", "INFO"],
        "INFO": ["\033[0m", "INFO"],  # ['\033[94m', "INFO"],
        "BOLD": ["\033[1m", "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = "\033[1m"
    ENDC = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    GRAY = "\033[90m"
    UNDERLINE = "\033[4m"


def printlog(msg, msg_type="INFO", flush=True):
    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(bcolors.LOG[msg_type][1], org, caller, msg)
    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=flush)


def read_node_launch(default_nodes, default_yml_file="nodes_launch.yaml"):

    CONF_PATH = os.path.dirname(os.path.abspath(__file__))
    FILE_PATH = os.path.join(CONF_PATH, default_yml_file)

    if not os.path.exists(FILE_PATH):

        try:
            with open(FILE_PATH, "w") as outfile:
                yaml.dump(default_nodes, outfile, default_flow_style=False)
            printlog(msg="Nodes launch file created", msg_type="WARN")

        except Exception as e:
            printlog(
                msg="Error creating nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_nodes

    else:

        try:
            with open(FILE_PATH, "r") as stream:
                default_nodes = yaml.safe_load(stream)
            printlog(
                msg="Nodes local launch file {}".format(default_yml_file),
                msg_type="OKGREEN",
            )

        except Exception as e:
            printlog(
                msg="Error reading nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_nodes


def generate_launch_description():

    ld = launch.LaunchDescription(
        [
            launch.actions.LogInfo(msg="Launching Planner ROS2 ..."),
        ]
    )

    # -------------------------------------------------------------------------
    nodes = read_node_launch(default_nodes={})

    # Print nodes to launch
    srt_ = "\n\nLaunching:"
    for key, node_args in nodes.items():
        if node_args["launch"]:
            srt_ = srt_ + "\n\tNode {}\tfrom {} package".format(
                node_args["node_name"]
                if "node_name" in node_args.keys()
                else "(Executable)",
                node_args["package"],
            )

    ld = launch.LaunchDescription(
        [
            launch.actions.LogInfo(msg=srt_ + "\n"),
        ]
    )

    for key, node_args in nodes.items():
        if "from_launch" in node_args.keys():
            if node_args["launch"] and node_args["from_launch"]:
                if "package" in node_args.keys():
                    launch_dir = get_package_share_directory(
                        "{}".format(node_args["package"])
                    )
                    included_launch = launch.actions.IncludeLaunchDescription(
                        launch.launch_description_sources.PythonLaunchDescriptionSource(
                            launch_dir + "/launch/{}.py".format(node_args["file"])
                        )
                    )
                    ld.add_action(included_launch)
        else:
            if node_args["launch"]:
                ld.add_action(
                    launch_ros.actions.Node(
                        executable=node_args["node_executable"],
                        name=node_args.get("node_name", None),
                        package=node_args["package"],
                        output=node_args["output"],
                    )
                )

    # -------------------------------------------------------------------------
    # Add here your python scripts
    # ld.add_action(launch.actions.ExecuteProcess(
    #     cmd=[sys.executable, '-u', './script_name.py', '--ignore-sigint', '--ignore-sigterm']
    # ))

    return ld

    # ============================================================================
