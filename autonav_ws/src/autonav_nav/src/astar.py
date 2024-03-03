#!/usr/bin/env python3

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
VERTICAL_CAMERA_DIST = 2.75
HORIZONTAL_CAMERA_DIST = 3
CV_BRIDGE = cv_bridge.CvBridge()

waypoints = {}

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
        self.waypointDelay = 10


class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")

        self.latitudeLength = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitudeLength = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value
        self.config = AStarNodeConfig()
        self.onReset()

    def init(self):
        self.configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.onConfigSpaceReceived, 20)
        self.poseSubscriber = self.create_subscription(Position, "/autonav/position", self.onPoseReceived, 20)
        self.imuSubscriber = self.create_subscription(IMUData, "/autonav/imu", self.onImuReceived, 20)
        self.debugPublisher = self.create_publisher(PathingDebug, "/autonav/debug/astar", 20)
        self.pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.safetyLightsPublisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        self.pathDebugImagePublisher = self.create_publisher(CompressedImage, "/autonav/debug/astar/image", 20)
        self.mapTimer = self.create_timer(0.1, self.createPath)

        self.resetWhen = -1.0
        self.waypointTime = time.time() + self.config.waypointDelay
        self.set_device_state(DeviceStateEnum.OPERATING)

    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta

    def onImuReceived(self, msg: IMUData):
        self.imu = msg

    def onReset(self):
        self.imu = None
        self.lastPath = None
        self.position = None
        self.configSpace = None
        self.costMap = None
        self.bestPosition = (0, 0)
        self.waypoints = []
        self.waypointTime = self.config.waypointDelay + time.time()

    def getWaypointsForDirection(self):
        # Get our current heading and estimate within 180 degrees which direction we are facing (north or south, 0 and 1 respectively)
        heading_degrees = abs(self.position.theta * 180 / math.pi)
        
        self.get_logger().info(f"Heading: {heading_degrees}")
        if heading_degrees > 120 and heading_degrees < 240:
            direction_index = 1
            self.get_logger().info("Facing South")
        else:
            direction_index = 0
            self.get_logger().info("Facing North")


        if self.system_mode == SystemModeEnum.COMPETITION:
            if direction_index == 0:
                return waypoints["compNorth"]
            else:
                return waypoints["compSouth"]
        elif self.system_mode == SystemModeEnum.SIMULATION:
            if direction_index == 0:
                return waypoints["simulationNorth"]
            else:
                return waypoints["simulationSouth"]
        else:
            return waypoints["practice"]

        

    def transition(self, old: SystemState, updated: SystemState):
        if updated.state == SystemStateEnum.AUTONOMOUS and updated.mobility and len(self.waypoints) == 0:
            self.waypointTime = time.time() + self.config.waypointDelay

        if updated.state != SystemStateEnum.AUTONOMOUS and self.device_state == DeviceStateEnum.OPERATING:
            self.onReset()

    def onPoseReceived(self, msg: Position):
        self.position = msg

    def createPath(self):
        if self.position is None or self.costMap is None:
            return

        robot_pos = (40, 78)
        path = self.findPathToPoint(robot_pos, self.bestPosition, self.costMap, 80, 80)
        if path is not None:
            global_path = Path()
            global_path.poses = [self.pathToGlobalPose(pp[0], pp[1]) for pp in path]
            self.lastPath = path
            self.pathPublisher.publish(global_path)

            # Draw the cost map onto a debug iamge
            cvimg = np.zeros((80, 80), dtype=np.uint8)
            for i in range(80):
                for j in range(80):
                    cvimg[i, j] = self.costMap[i * 80 + j] * 255
            cvimg = cv2.cvtColor(cvimg, cv2.COLOR_GRAY2RGB)

            for pp in path:
                cv2.circle(cvimg, (pp[0], pp[1]), 1, (0, 255, 0), 1)

            cv2.circle(cvimg, (self.bestPosition[0], self.bestPosition[1]), 1, (255, 0, 0), 1)
            cvimg = cv2.resize(cvimg, (800, 800),interpolation=cv2.INTER_NEAREST)
            self.pathDebugImagePublisher.publish(CV_BRIDGE.cv2_to_compressed_imgmsg(cvimg))

    def reconstructPath(self, path, current):
        total_path = [current]

        while current in path:
            current = path[current]
            total_path.append(current)

        return total_path[::-1]

    def findPathToPoint(self, start, goal, map, width, height):
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
                return self.reconstructPath(path, current)

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

    def onConfigSpaceReceived(self, msg: OccupancyGrid):
        if self.position is None or self.system_state != SystemStateEnum.AUTONOMOUS:
            return

        grid_data = msg.data
        temp_best_pos = (40, 78)
        best_pos_cost = -1000

        frontier = set()
        frontier.add((40, 78))
        explored = set()

        if self.config.useOnlyWaypoints == True:
            grid_data = [0] * len(msg.data)

        if len(self.waypoints) == 0 and time.time() > self.waypointTime and self.waypointTime != 0:
            self.waypoints = [wp for wp in self.getWaypointsForDirection()]
            self.waypointTime = 0

        if time.time() < self.waypointTime and len(self.waypoints) == 0:
            time_remaining = self.waypointTime - time.time()
            pathingDebug = PathingDebug()
            pathingDebug.waypoints = []
            pathingDebug.time_until_use_waypoints = time_remaining
            self.debugPublisher.publish(pathingDebug)

        if time.time() > self.resetWhen and self.resetWhen != -1 and self.mobility:
            self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#FFFFFF"))
            self.resetWhen = -1

        if len(self.waypoints) > 0:
            next_waypoint = self.waypoints[0]
            north_to_gps = (next_waypoint[0] - self.position.latitude) * self.latitudeLength
            west_to_gps = (self.position.longitude -next_waypoint[1]) * self.longitudeLength
            heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            if north_to_gps ** 2 + west_to_gps ** 2 <= self.config.waypointPopDistance:
                self.waypoints.pop(0)
                self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 255, "#00FF00"))
                self.resetWhen = time.time() + 1.5

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
                    cost -= max(heading_err_to_gps, 10)

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

        self.costMap = grid_data
        self.bestPosition = temp_best_pos

    def pathToGlobalPose(self, pp0, pp1):
        x = (80 - pp1) * VERTICAL_CAMERA_DIST / 80
        y = (40 - pp0) * HORIZONTAL_CAMERA_DIST / 80

        new_x = x * math.cos(0) + y * math.sin(0)
        new_y = x * math.sin(0) + y * math.cos(0)
        pose = PoseStamped()
        point = Point()
        point.x = new_x
        point.y = new_y
        pose.pose.position = point
        return pose


def main():
    # read GPS waypoints data from file
    with open("data/waypoints.csv", "r") as f:
        for line in f.readlines():
            label, lat, lon = line.split(",")

            try:
                waypoints[label].append((float(lat), float(lon)))
            except KeyError:
                waypoints[label] = []
                waypoints[label].append((float(lat), float(lon)))
                

    rclpy.init()
    node = AStarNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
