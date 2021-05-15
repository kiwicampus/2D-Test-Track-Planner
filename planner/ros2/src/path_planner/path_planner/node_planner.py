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
        self._TURN_CRTL_POINTS = float(os.getenv("TURN_CRTL_POINTS", default=30))
        self._FORWARE_ACELERATION_FC = float(
            os.getenv("FORWARE_ACELERATION_FC", default=0.3)
        )
        self._FORWARE_CRTL_POINTS = float(os.getenv("FORWARE_CRTL_POINTS", default=30))
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
        """

        way_points = []

        norm = np.sqrt((src[0] - dst[0]) ** 2 + (src[1] - dst[1]) ** 2)

        tau = time * pt  # Time for the first segment [s]
        Vk = norm / (time * (1 - pt))  # Max speed [ms-1]
        a = Vk / tau  # [ms-2]

        Ptau = (
            (0.5 / norm) * a * (tau ** 2) * (dst[0] - src[0]) + src[0],
            (0.5 / norm) * a * (tau ** 2) * (dst[1] - src[1]) + src[1],
        )
        Pttau = (
            (Vk * (time - 2 * tau) / norm) * (dst[0] - src[0]) + Ptau[0],
            (Vk * (time - 2 * tau) / norm) * (dst[1] - src[1]) + Ptau[1],
        )

        # ---------------------------------------------------------------------
        # Accelerated segment
        k = n * pt  # Number of points
        ptp0_norm = np.sqrt((Ptau[0] - src[0]) ** 2 + (Ptau[1] - src[1]) ** 2)
        ptp0 = (Ptau[0] - src[0], Ptau[1] - src[1])
        tseg1 = 2 * ptp0_norm / Vk  # Time for Accelerated segment [s]
        a1 = Vk / tseg1  # Acceleration for Accelerated segment [m2/s]
        times = np.linspace(0, tseg1, num=int(k))

        idx = 1
        for t in times:
            pt = (
                (0.5 / ptp0_norm) * a1 * (t ** 2) * ptp0[0] + src[0],
                (0.5 / ptp0_norm) * a1 * (t ** 2) * ptp0[1] + src[1],
            )
            way_points.append({"idx": idx, "pt": pt, "t": t, "dt": tseg1 / k})
            idx += 1

        # ---------------------------------------------------------------------
        # Constant segment
        kc = n - 2 * k
        ptaupt = (Pttau[0] - Ptau[0], Pttau[1] - Ptau[1])
        ptaupt_norm = np.sqrt((Pttau[0] - Ptau[0]) ** 2 + (Pttau[1] - Ptau[1]) ** 2)
        tseg2 = ptaupt_norm / Vk
        times = np.linspace(0, tseg2, num=int(kc))
        for t in times:
            pt = (
                (Vk * t / ptaupt_norm) * ptaupt[0] + Ptau[0],
                (Vk * t / ptaupt_norm) * ptaupt[1] + Ptau[1],
            )
            way_points.append({"idx": idx, "pt": pt, "t": t, "dt": tseg2 / kc})
            idx += 1

        # ---------------------------------------------------------------------
        # Decelerated segment
        ptpdst = (dst[0] - Pttau[0], dst[1] - Pttau[1])
        ptpdst_norm = np.sqrt((Pttau[0] - dst[0]) ** 2 + (Pttau[1] - dst[1]) ** 2)
        tseg3 = 2.0 * ptpdst_norm / Vk
        a3 = -Vk / tseg3
        times = np.linspace(0, tseg3, num=int(k))
        for t in times:
            pt = (
                (1 / ptpdst_norm) * (0.5 * a3 * (t ** 2) + (Vk * t)) * ptpdst[0]
                + Pttau[0],
                (1 / ptpdst_norm) * (0.5 * a3 * (t ** 2) + (Vk * t)) * ptpdst[1]
                + Pttau[1],
            )
            way_points.append({"idx": idx, "pt": pt, "t": t, "dt": tseg3 / k})
            idx += 1

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
        """

        turn_points = []
        if dst == 0.0:
            return turn_points

        tau = time * pt  # Time for the first segment [s]
        Vk = dst / (time * (1 - pt))  # Max speed [ms-1]
        a = Vk / tau  # [ms-2]
        Ptau = 0.5 * a * (tau ** 2)
        Pttau = Vk * (time - 2 * tau) + Ptau

        # ---------------------------------------------------------------------
        # Accelerated segment
        k = n * pt  # Number of points
        tseg1 = 2 * Ptau / Vk  # Time for Accelerated segment [s]
        a1 = Vk / tseg1  # Acceleration for Accelerated segment [m2/s]
        times = np.linspace(0, tseg1, num=int(k))

        idx = 1
        for t in times:
            ang = 0.5 * a1 * (t ** 2)
            turn_points.append({"idx": idx, "a": ang, "t": t, "dt": tseg1 / k})
            idx += 1

        # ---------------------------------------------------------------------
        # Constant segment
        kc = n - 2 * k
        ptaupt = Pttau - Ptau
        tseg2 = ptaupt / Vk
        times = np.linspace(0, tseg2, num=int(kc))
        for t in times:
            ang = (Vk * t) + Ptau
            turn_points.append({"idx": idx, "a": ang, "t": t, "dt": tseg2 / kc})
            idx += 1

        # ---------------------------------------------------------------------
        # Decelerated segment
        ptpdst = dst - Pttau
        tseg3 = 2.0 * ptpdst / Vk
        a3 = -Vk / tseg3
        times = np.linspace(0, tseg3, num=int(k))
        for t in times:
            ang = (0.5 * a3 * (t ** 2) + (Vk * t)) + Pttau
            turn_points.append({"idx": idx, "a": ang, "t": t, "dt": tseg3 / k})
            idx += 1
        turn_points[-1]["a"] = float(int(turn_points[-1]["a"]))

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
