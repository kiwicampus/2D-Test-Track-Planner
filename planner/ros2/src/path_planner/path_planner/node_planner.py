#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np
import math
import yaml
import csv
import sys
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Int32
from std_msgs.msg import Int8
from std_msgs.msg import Bool

from utils.python_utils import printlog

from usr_msgs.msg import Planner as planner_msg
from usr_msgs.msg import LandMark
from usr_msgs.msg import Waypoint
from usr_msgs.msg import TurnRef
from usr_msgs.msg import Kiwibot

from usr_srvs.srv import Move
from usr_srvs.srv import Turn

# =============================================================================
def setProcessName(name: str) -> None:
    """!
    Function for seting the process name
    @see name 'str' defining the process name
    """
    if sys.platform in ["linux2", "linux"]:
        import ctypes

        libc = ctypes.cdll.LoadLibrary("libc.so.6")
        libc.prctl(15, name, 0, 0, 0)
    else:
        raise Exception(
            "Can not set the process name on non-linux systems: " + str(sys.platform)
        )


def read_yaml_file(CONF_PATH: str, FILE_NAME: str) -> dict:
    """!
    Function for seting the process name
    @param CONF_PATH `string` absolute path to configuration of cameras
    @param FILE_NAME `string` name of cameras configuration file
    @return data_loaded `dictionary` key: camera labels, values: dictionary with camera
            properties and settings, see yaml file for more details
    """

    abs_path = os.path.join(CONF_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, "r") as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        return []


# =============================================================================
class PlannerNode(Node):
    def __init__(self) -> None:
        """
            Class constructor for path planning node
        Args:
        Returns:
        """

        # ---------------------------------------------------------------------
        Node.__init__(self, node_name="planner_node")

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # Environment variables for forware and turn profiles
        self._TURN_ACELERATION_FC = float(os.getenv("TURN_ACELERATION_FC", default=0.3))
        self._TURN_CRTL_POINTS = int(
            os.getenv("TURN_CRTL_POINTS", default=30)
        )  # Was Float before
        self._FORWARE_ACELERATION_FC = float(
            os.getenv("FORWARE_ACELERATION_FC", default=0.3)
        )
        self._FORWARE_CRTL_POINTS = int(
            os.getenv("FORWARE_CRTL_POINTS", default=30)
        )  # Was Float before
        self._TURN_TIME = float(os.getenv("TURN_TIME", default=3.0))

        # ---------------------------------------------------------------------
        # Map features
        self.map_points = []  # Landmarks or keypoints in map
        self.map_duration = 0.0  # Map duration in [s]
        self.map_difficulty = 0.0  # Map difficulty [0.0-5.0]
        self.map_distance = 0.0  # Map distance in [m]
        self.way_points = {}  # List of waypoints in the path planning routine

        self._in_execution = False

        # Read routines from the yaml file in the configs folder
        self.routines = read_yaml_file(
            CONF_PATH="/workspace/planner/configs",
            FILE_NAME="routines.yaml",
        )

        # ---------------------------------------------------------------------
        # Subscribers

        self.sub_start_routine = self.create_subscription(
            msg_type=Int32,
            topic="/graphics/start_routine",
            callback=self.cb_start_routine,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.kiwibot_state = Kiwibot()
        self.sub_kiwibot_stat = self.create_subscription(
            msg_type=Kiwibot,
            topic="/kiwibot/status",
            callback=self.cb_kiwibot_status,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.sub_routine_status = self.create_subscription(
            msg_type=Bool,
            topic="/path_planner/routine_status",
            callback=self.cb_routine_status,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        # ---------------------------------------------------------------------
        # Publishers

        self.pub_path_planner = self.create_publisher(
            msg_type=planner_msg,
            topic="/path_planner/msg",
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.pub_speaker = self.create_publisher(
            msg_type=Int8,
            topic="/device/speaker/command",
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        # ---------------------------------------------------------------------
        # Services

        # service client to turn the robot
        self.cli_robot_turn = self.create_client(Turn, "/robot/turn")

        # service client to move the robot
        self.cli_robot_move = self.create_client(Move, "/robot/move")

        try:
            self.robot_turn_req = Turn.Request()
            self.robot_move_req = Move.Request()
        except Exception as e:
            printlog(
                msg="No services for robot actions, {}".format(e),
                msg_type="ERROR",
            )

    def cb_kiwibot_status(self, msg: Kiwibot) -> None:
        """
            Callback to update kiwibot state information in visuals
        Args:
            msg: `Kiwibot` message with  kiwibot state information
                int8 pos_x      # x axis position in the map
                int8 pos_y      # y axis position in the map
                float32 dist    # distance traveled by robot
                float32 speed   # speed m/s
                float32 time    # time since robot is moving
                float32 yaw     # time since robot is moving
                bool moving     # Robot is moving
        Returns:
        """

        try:
            self.kiwibot_state = msg

        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            printlog(
                msg="{}, {}, {}, {}".format(e, exc_type, fname, exc_tb.tb_lineno),
                msg_type="ERROR",
            )

    def cb_routine_status(self, msg: Bool) -> None:
        self.pub_speaker.publish(Int8(data=0))

    def cb_start_routine(self, msg: Int32) -> None:
        """
            Callback when a routine is started from visuals
        Args:
            msg: `Int32` number of routine to load waypoint from landmarks
        Returns:
        """

        try:

            if self._in_execution:
                printlog(msg="There's already a routine in execution", msg_type="WARN")
                return

            self._in_execution = True

            # Check that the routine in received exists in the routines list
            if msg.data in self.routines.keys():

                # -------------------------------------------------------
                # Read the waypoint or landmarks for the specified route
                self.way_points = self.read_keypoints(
                    land_marks_path="/workspace/planner/configs/key_points.csv",
                    key_Points=self.routines[msg.data],
                )

                # Publish routine for graphics components
                self.pub_path_planner.publish(
                    planner_msg(
                        land_marks=[
                            LandMark(
                                neighbors=[],
                                id=idx,
                                x=int(way_point_coord[0]),
                                y=int(way_point_coord[1]),
                            )
                            for idx, way_point_coord in enumerate(
                                self.way_points["coords"]
                            )
                        ],
                        distance=self.map_distance,
                        duration=self.map_duration,
                        difficulty=self.map_difficulty,
                    )
                )

                # -------------------------------------------------------
                # Get the robot in the initial position
                printlog(
                    msg="setting the robot in origin",
                    msg_type="OKPURPLE",
                )
                self.robot_move_req.waypoints = [
                    Waypoint(
                        id=0,
                        x=int(self.way_points["coords"][0][0]),
                        y=int(self.way_points["coords"][0][1]),
                        t=0.0,
                        dt=0.0,
                    )
                ]
                move_resp = self.cli_robot_move.call(self.robot_move_req)

                # -------------------------------------------------------
                # Execute planning process
                self.pub_speaker.publish(Int8(data=2))
                for idx, way_point in enumerate(self.way_points["coords"][:-1]):

                    # -------------------------------------------------------
                    # Calculate the angle to turn the robot
                    dy = (
                        self.way_points["coords"][idx][1]
                        - self.way_points["coords"][idx + 1][1]
                    )
                    dx = (
                        self.way_points["coords"][idx + 1][0]
                        - self.way_points["coords"][idx][0]
                    )
                    ang = np.rad2deg(np.arctan2(dy, dx))
                    dang = ang - self.kiwibot_state.yaw

                    if abs(dang) > 180:
                        dang += 360
                    elif dang > 360:
                        dang -= 360
                    elif dang > 180:
                        dang -= 360
                    elif dang < -180:
                        dang += 360

                    if int(dang):

                        printlog(
                            msg=f"turning robot to reference {idx+1}",
                            msg_type="OKPURPLE",
                        )

                        # Generate the turning profile to get the robot aligned to the next landmark
                        self.robot_turn_req.turn_ref = [
                            TurnRef(
                                id=turn_reference["idx"],
                                yaw=turn_reference["a"],
                                t=turn_reference["t"],
                                dt=turn_reference["dt"],
                            )
                            for turn_reference in self.get_profile_turn(
                                dst=dang,
                                time=self._TURN_TIME,
                                pt=self._TURN_ACELERATION_FC,
                                n=self._TURN_CRTL_POINTS,
                            )
                        ]

                        move_resp = self.cli_robot_turn.call(self.robot_turn_req)

                    # -------------------------------------------------------
                    printlog(
                        msg=f"moving robot to landmark {idx}",
                        msg_type="OKPURPLE",
                    )

                    # Generate the waypoints to the next landmark
                    seg_way_points = self.get_profile_route(
                        src=self.way_points["coords"][idx],
                        dst=self.way_points["coords"][idx + 1],
                        time=self.way_points["times"][idx],
                        pt=self._FORWARE_ACELERATION_FC,
                        n=self._FORWARE_CRTL_POINTS,
                    )

                    # Move the robot to the next landmark
                    self.robot_move_req.waypoints = [
                        Waypoint(
                            id=wp["idx"],
                            x=int(wp["pt"][0]),
                            y=int(wp["pt"][1]),
                            t=wp["t"],
                            dt=wp["dt"],
                        )
                        for wp in seg_way_points
                    ]

                    move_resp = self.cli_robot_move.call(self.robot_move_req)

                # -------------------------------------------------------
                if not self._in_execution:
                    printlog(
                        msg=f"routine {msg.data} execution has been stopped",
                        msg_type="WARN",
                    )
                else:
                    printlog(
                        msg=f"routine {msg.data} has finished",
                        msg_type="OKGREEN",
                    )

                # -------------------------------------------------------
                self.pub_speaker.publish(Int8(data=3))

            else:
                printlog(
                    msg=f"routine {msg.data} does not exit",
                    msg_type="WARN",
                )
                return

        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            printlog(
                msg="{}, {}, {}, {}".format(e, exc_type, fname, exc_tb.tb_lineno),
                msg_type="ERROR",
            )

        self._in_execution = False

    def read_keypoints(self, land_marks_path: str, key_Points: list) -> list:
        """
            Reads and loads maps key points configuration and create way points
            for robots trajectory
        Args:
            land_marks_path: `string` absolute path to maps keypoints configuration path
            key_Points: `list` of tuples as (key_points, code of trajectory)
        Returns:
            way_points: `dict` coords: coordinates to follow and times: times for
                each coordinate
        """

        # Auxiliar variables
        way_points = {"coords": [], "times": [], "distances": []}
        map_difficulty = []

        # Check if file exits
        if not os.path.isfile(land_marks_path):
            print("[ERROR]: No configuration file")
            return way_points

        # Open and read csv file
        with open(land_marks_path, "r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for idx, line in enumerate(csv_reader):
                if idx != 0:
                    self.map_points.append(
                        {
                            "src_id": int(line[0]),
                            "src_coord": (int(line[1]), int(line[2])),
                            "dst_id": int(line[3]),
                            "dst_coord": (int(line[4]), int(line[5])),
                            "difficulty": float(line[6]),
                            "code": int(line[7]),
                            "description": line[8],
                            "distance": float(line[9]),
                            "time": float(line[10]),
                        }
                    )

        # Generate map
        for idx, key_pt in enumerate(key_Points[:-1]):
            match_src = [
                dic
                for dic in self.map_points
                if dic["src_id"] == key_pt[0]
                and dic["dst_id"] == key_Points[idx + 1][0]
                and dic["code"] == key_pt[1]
            ]
            if len(match_src):
                self.map_duration += match_src[0]["time"]
                self.map_distance += match_src[0]["distance"] / 100
                map_difficulty.append(match_src[0]["difficulty"])

                if not len(way_points["coords"]):
                    way_points["coords"].append(match_src[-1]["src_coord"])
                way_points["coords"].append(match_src[-1]["dst_coord"])
                way_points["times"].append(match_src[-1]["time"])
                way_points["distances"].append(match_src[-1]["distance"] / 100)

            else:
                print(
                    "[ERROR]: THERE'S NO A DEFINED TRAJECTORY FROM {} TO {}".format(
                        key_pt[0], key_Points[idx + 1][0]
                    )
                )
                break

        # Get maps difficulty
        self.map_difficulty = round(
            np.mean(map_difficulty) if len(map_difficulty) else None, 2
        )
        self.map_distance = round(self.map_distance, 2)

        return way_points

    def get_profile_route(
        self, src: tuple, dst: tuple, time: float, pt=0.3, n=30
    ) -> list:
        """
            Generates waypoints: coordinates and times with a trapezoidal profile
        Args:
            src: `tuple` origin coordinate (X, Y)
            dst: `tuple` destination coordinate (X, Y)
            time: `float` time from origin to destination
            pt: `float` deceleration/acceleration factor
            n: `int` control points to discrite the trajectory
        Returns:
            way_points: `dict` coordinates and times of trajectory with trapezoidal profile
                every element in the list is  dictionary with the keys:
                {
                    "idx": [int](index of the waypoint),
                    "pt": [tuple][int](x and y axis positions in the image space),
                    "t": [float](time for angle a),
                    "dt": [float](sept of time for angle a, is constant element)
                }
        """

        way_points = []

        # ---------------------------------------------------------------------
        # TODO: Trapezoidal speed profile
        # Add your solution here, remember that every element in the list is a dictionary
        # where every element in has the next structure and data type:
        # "idx": [int](index of the waypoint),
        # "pt": [tuple][int](x and y axis positions in the image space),
        # "t": [float](time for angle a),
        # "dt": [float](sept of time for angle a, is constant element)
        # Do not forget and respect the keys names

        # ---------------------------------------------------------------------

        d = math.sqrt(
            ((dst[0] - src[0]) ** 2 + (dst[1] - src[1]) ** 2)
        )  # Distance between src and dst

        theta = math.atan2(
            (dst[1] - src[1]), (dst[0] - src[0])
        )  # Angle between src and dst

        t_a = time * pt  # Amount of time for acceleration/deceleration phase
        t_c = time - 2 * t_a  # Amount of time for constant velocity
        v_max = d / (t_a + t_c)  # Max velocity of the trajectory

        dt = time / n  # Delta of time for the discrete trajectory
        d_a = 0  # Distance Traveled in acceleration
        d_c = 0  # Distance Traveled at max velocity
        d_d = 0  # Distance Traveled in deceleration

        for i in range(n):

            idx = i + 1
            t = dt * idx
            if t <= t_a:  # For accelerating
                b = t  # Base of the triangle
                h = v_max * t / t_a  # Height of the triangle
                d_t = b * h / 2  # Distance traveled (Area below the curve)

            elif t > t_a and t <= (t_a + t_c):  # For max velocity
                b = t - t_a  # Base of the rectangle
                h = v_max  # Height of the rectangle
                d_t = (
                    b * h + v_max * t_a / 2
                )  # Distance traveled (Area below the curve)

            else:  # For decelerating
                b = t - (t_a + t_c)  # Base of the rectangle
                h1 = v_max * (1 - b / t_a)  # Height of smaller side of the trapezoid
                h2 = v_max  # Height of bigger side of the trapezoid
                d_t = (
                    ((h1 + h2) * b / 2) + (v_max * t_a / 2) + (v_max * t_c)
                )  # Distance traveled (Area below the curve)

            x = int(math.cos(theta) * d_t) + src[0]
            y = int(math.sin(theta) * d_t) + src[1]

            wp = {"idx": idx, "pt": (x, y), "t": t, "dt": dt}
            way_points.append(wp)

        return way_points

    def get_profile_turn(self, dst: float, time: float, pt=0.3, n=30) -> list:
        """
            Generates waypoints: coordinates and times with a trapezoidal turning profile
        Args:
            dst: `tuple` target angle
            time: `float` time for turning angle
            pt: `float` deceleration/acceleration factor
            n: `int` control points to discrite the trajectory
        Returns:
            turn_points: `dict` coordinates and times of turn with trapezoidal profile
                every element in the list is  dictionary with the keys:
                {
                    "idx": [int](index of the waypoint),
                    "a": [float](yaw angle of the robot),
                    "t": [float](time for angle a),
                    "dt": [float](sept of time for angle a, is constant element)
                }
        """

        turn_points = []
        if dst == 0.0:
            return turn_points

        # ---------------------------------------------------------------------
        # TODO: Trapezoidal turn profile
        # Add your solution here, remeber that every element in the list is a dictionary
        # where every element in has the next structure and data type:
        # "idx": [int](index of the waypoint),
        # "a": [float](yaw angle of the robot),
        # "t": [float](time for angle a),
        # "dt": [float](sept of time for angle a, is constant element)
        # Do not forget and respect the keys names

        # ---------------------------------------------------------------------

        t_a = time * pt  # Amount of time for acceleration/deceleration phase
        t_c = time - 2 * t_a  # Amount of time for constant velocity
        w_max = dst / (t_a + t_c)  # Max velocity of the trajectory

        dt = time / n  # Delta of time for the discrete trajectory

        d_a = 0  # Degrees Traveled in acceleration
        d_c = 0  # Degrees Traveled at max velocity
        d_d = 0  # Degrees Traveled in deceleration

        for i in range(n):
            idx = i + 1
            t = dt * idx
            if t <= t_a:  # For accelerating
                b = t  # Base of the triangle
                h = w_max * t / t_a  # Height of the triangle
                d_a = b * h / 2  # Distance traveled (Area below the curve)

            elif t > t_a and t <= (t_a + t_c):  # For max velocity
                b = t - t_a  # Base of the rectangle
                h = w_max  # Height of the rectangle
                d_c = b * h  # Distance traveled (Area below the curve)

            else:  # For decelerating
                b = t - (t_a + t_c)  # Base of the rectangle
                h1 = w_max * (1 - b / t_a)  # Height of smaller side of the trapezoid
                h2 = w_max  # Height of bigger side of the trapezoid
                d_d = (h1 + h2) * b / 2  # Distance traveled (Area below the curve)

            d_t = d_a + d_c + d_d  # Total degrees turned

            wp = {"idx": idx, "a": d_t, "t": t, "dt": dt}
            turn_points.append(wp)

        return turn_points


# =============================================================================
def main(args=None) -> None:
    """!
    Main Functions of Local Console Node
    """
    # Initialize ROS communications for a given context.
    setProcessName("planner-node")
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    planner_node = PlannerNode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(planner_node, executor)

    # Clear thread
    planner_node.clear()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
