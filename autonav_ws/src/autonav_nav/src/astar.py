#!/usr/bin/env python3

import json
from types import SimpleNamespace
from autonav_msgs.msg import Position, IMUData, PathingDebug, SafetyLights
from scr_msgs.msg import SystemState
from scr.node import Node
from scr.states import DeviceStateEnum, SystemStateEnum, SystemModeEnum
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
import copy
from heapq import heappush, heappop
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import cv_bridge
import time


GRID_SIZE = 0.1
CV_BRIDGE = cv_bridge.CvBridge()

competition_waypoints = [
    [],  # NORTH
    []  # SOUTH
]

practice_waypoints = [
    [],  # NORTH
    []  # SOUTH
]

simulation_waypoints = [
    [(35.19505, -97.43823), (35.19492, -97.43824), (35.19485, -97.43824), (35.19471, -97.43823)],  # NORTH
    [(35.19471, -97.43823), (35.19492, -97.43824), (35.19485, -97.43824), (35.19505, -97.43823)]  # SOUTH
]


def hexToRgb(color: str):
    if color[0] == "#":
        color = color[1:]
    return [int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)]


def toSafetyLights(autonomous: bool, eco: bool, mode: int, brightness: int, color: str) -> SafetyLights:
    pkg = SafetyLights()
    pkg.mode = mode
    pkg.autonomous = autonomous
    pkg.eco = eco
    pkg.brightness = brightness
    colorr = hexToRgb(color)
    pkg.red = colorr[0]
    pkg.green = colorr[1]
    pkg.blue = colorr[2]
    return pkg


class AStarNodeConfig:
    def __init__(self):
        self.waypointPopDistance = 1.1
        self.waypointDirection = 0
        self.useOnlyWaypoints = False
        self.waypointDelay = 18
        self.waypointWeight = 1.0
        self.waypointMaxWeight = 10.0
        self.horizontal_fov = 3.4
        self.vertical_fov = 2.75


class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")

        self.latitudeLength = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitudeLength = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value
        self.config = self.get_default_config()
        self.onReset()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return AStarNodeConfig()

    def init(self):
        self.configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.cfg_space_Received, 20)
        self.poseSubscriber = self.create_subscription(Position, "/autonav/position", self.pose_received, 20)
        self.debugPublisher = self.create_publisher(PathingDebug, "/autonav/debug/astar", 20)
        self.pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.safetyLightsPublisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        self.pathDebugImagePublisher = self.create_publisher(CompressedImage, "/autonav/debug/astar/image", self.qos_profile)
        self.map_timer = self.create_timer(0.1, self.create_path)

        self.reset_at = -1.0
        self.waypoint_start_time = time.time() + self.config.waypointDelay
        self.set_device_state(DeviceStateEnum.OPERATING)

    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta

    def onReset(self):
        self.last_path = None
        self.position = None
        self.cfg_spce = None
        self.cost_map = None
        self.best_pos = (0, 0)
        self.waypoints = []
        self.waypoint_start_time = self.config.waypointDelay + time.time()

    def get_waypoints_for_dir(self):
        # Get out current heading and estimate within 180 degrees which direction we are facing (north or south, 0 and 1 respectively)
        heading = self.position.theta
        direction_index = 0
        heading_degrees = abs(heading * 180 / math.pi)
        self.get_logger().info(f"Heading: {heading_degrees}")
        if heading_degrees > 120 and heading_degrees < 240:
            direction_index = 1
            self.get_logger().info("Facing South")
        else:
            self.get_logger().info("Facing North")

        return simulation_waypoints[direction_index] if self.system_mode == SystemModeEnum.SIMULATION else competition_waypoints[direction_index] if self.system_mode == SystemModeEnum.COMPETITION else practice_waypoints[direction_index]

    def system_state_transition(self, old: SystemState, updated: SystemState):
        if updated.state == SystemStateEnum.AUTONOMOUS and updated.mobility and len(self.waypoints) == 0:
            self.waypoint_start_time = time.time() + self.config.waypointDelay

        if updated.state != SystemStateEnum.AUTONOMOUS and self.device_state == DeviceStateEnum.OPERATING:
            self.onReset()

    def pose_received(self, msg: Position):
        self.position = msg

    def create_path(self):
        if self.position is None or self.cost_map is None:
            return

        robot_pos = (40, 78)
        path = self.find_path_to_point(robot_pos, self.best_pos, self.cost_map, 80, 80)
        if path is not None:
            global_path = Path()
            global_path.poses = [self.path_to_global(pp[0], pp[1]) for pp in path]
            self.last_path = path

            if self.system_state == SystemStateEnum.AUTONOMOUS and self.mobility:
                self.pathPublisher.publish(global_path)

            # Draw the cost map onto a debug iamge
            cvimg = np.zeros((80, 80), dtype=np.uint8)
            for i in range(80):
                for j in range(80):
                    cvimg[i, j] = self.cost_map[i * 80 + j] * 255
            cvimg = cv2.cvtColor(cvimg, cv2.COLOR_GRAY2RGB)

            for pp in path:
                cv2.circle(cvimg, (pp[0], pp[1]), 1, (0, 255, 0), 1)

            cv2.circle(cvimg, (self.best_pos[0], self.best_pos[1]), 1, (255, 0, 0), 1)
            cvimg = cv2.resize(cvimg, (800, 800), interpolation=cv2.INTER_NEAREST)
            self.pathDebugImagePublisher.publish(CV_BRIDGE.cv2_to_compressed_imgmsg(cvimg))

    def reconstruct_path(self, path, current):
        total_path = [current]

        while current in path:
            current = path[current]
            total_path.append(current)

        return total_path[::-1]

    def find_path_to_point(self, start, goal, map, width, height):
        looked_at = np.zeros((80, 80))
        open_set = [start]
        path = {}
        search_dirs = []

        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue
                search_dirs.append((x, y, math.sqrt(x ** 2 + y ** 2)))

        def h(point):
            return math.sqrt((goal[0] - point[0]) ** 2 + (goal[1] - point[1]) ** 2)

        def d(to_pt, dist):
            return dist + map[to_pt[1] * width + to_pt[0]] / 10

        gScore = {}
        gScore[start] = 0

        def getG(pt):
            if pt in gScore:
                return gScore[pt]
            else:
                gScore[pt] = 1000000000
                return 1000000000  # Infinity

        fScore = {}
        fScore[start] = h(start)
        next_current = [(1, start)]
        while len(open_set) != 0:
            current = heappop(next_current)[1]

            looked_at[current[0], current[1]] = 1

            if current == goal:
                return self.reconstruct_path(path, current)

            open_set.remove(current)
            for delta_x, delta_y, dist in search_dirs:

                neighbor = (current[0] + delta_x, current[1] + delta_y)
                if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
                    continue

                tentGScore = getG(current) + d(neighbor, dist)
                if tentGScore < getG(neighbor):
                    path[neighbor] = current
                    gScore[neighbor] = tentGScore
                    fScore[neighbor] = tentGScore + h(neighbor)
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                        heappush(next_current, (fScore[neighbor], neighbor))

    def cfg_space_Received(self, msg: OccupancyGrid):
        if self.position is None:
            return

        grid_data = msg.data
        temp_best_pos = (40, 78)
        best_pos_cost = -1000

        frontier = set()
        frontier.add((40, 78))
        explored = set()

        if self.config.useOnlyWaypoints == True:
            grid_data = [0] * len(msg.data)

        if len(self.waypoints) == 0 and time.time() > self.waypoint_start_time and self.waypoint_start_time != 0:
            self.waypoints = [wp for wp in self.get_waypoints_for_dir()]
            self.waypoint_start_time = 0

        if time.time() < self.waypoint_start_time and len(self.waypoints) == 0:
            time_remaining = self.waypoint_start_time - time.time()
            pathingDebug = PathingDebug()
            pathingDebug.waypoints = []
            pathingDebug.time_until_use_waypoints = time_remaining
            self.debugPublisher.publish(pathingDebug)

        if time.time() > self.reset_at and self.reset_at != -1 and self.mobility:
            self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#FFFFFF"))
            self.reset_at = -1

        if len(self.waypoints) > 0:
            next_waypoint = self.waypoints[0]
            north_to_gps = (next_waypoint[0] - self.position.latitude) * self.latitudeLength
            west_to_gps = (self.position.longitude - next_waypoint[1]) * self.longitudeLength
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            if north_to_gps ** 2 + west_to_gps ** 2 <= self.config.waypointPopDistance:
                self.waypoints.pop(0)
                self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#00FF00"))
                self.reset_at = time.time() + 1.5

            pathingDebug = PathingDebug()
            pathingDebug.desired_heading = heading_to_gps
            pathingDebug.desired_latitude = next_waypoint[0]
            pathingDebug.desired_longitude = next_waypoint[1]
            pathingDebug.distance_to_destination = north_to_gps ** 2 + west_to_gps ** 2
            wp1d = []
            for wp in self.waypoints:
                wp1d.append(wp[0])
                wp1d.append(wp[1])
            pathingDebug.waypoints = wp1d
            self.debugPublisher.publish(pathingDebug)

        depth = 0
        while depth < 50 and len(frontier) > 0:
            curfrontier = copy.copy(frontier)
            for pos in curfrontier:
                x = pos[0]
                y = pos[1]
                cost = (80 - y) * 1.3 + depth * 2.2

                if len(self.waypoints) > 0:
                    heading_err_to_gps = abs(self.getAngleDifference(self.position.theta + math.atan2(40 - x, 80 - y), heading_to_gps)) * 180 / math.pi
                    cost -= max(heading_err_to_gps, self.config.waypointMaxWeight) * self.config.waypointWeight

                if cost > best_pos_cost:
                    best_pos_cost = cost
                    temp_best_pos = pos

                frontier.remove(pos)
                explored.add(x + 80 * y)

                if y > 1 and grid_data[x + 80 * (y-1)] < 50 and x + 80 * (y-1) not in explored:
                    frontier.add((x, y - 1))

                if x < 79 and grid_data[x + 1 + 80 * y] < 50 and x + 1 + 80 * y not in explored:
                    frontier.add((x + 1, y))

                if x > 0 and grid_data[x - 1 + 80 * y] < 50 and x - 1 + 80 * y not in explored:
                    frontier.add((x - 1, y))

            depth += 1

        self.cost_map = grid_data
        self.best_pos = temp_best_pos

    def path_to_global(self, pp0, pp1):
        x = (80 - pp1) * self.config.vertical_fov / 80
        y = (40 - pp0) * self.config.horizontal_fov / 80

        new_x = x * math.cos(0) + y * math.sin(0)
        new_y = x * math.sin(0) + y * math.cos(0)
        pose = PoseStamped()
        point = Point()
        point.x = new_x
        point.y = new_y
        pose.pose.position = point
        return pose


def main():
    rclpy.init()
    node = AStarNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
