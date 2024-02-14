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

# STARTING_PT = (42.6681268, -83.218887)

# there's gotta be a better way to store these, right? like, idk, csv file?
competition_waypoints = [
    # Start Nomans, Frist Ramp, Middle Ramp, End Ramp, End Nomans
    [(42.6682837222, -83.2193403028), (42.6681206444, -83.2193606083), (42.66809863885, -83.2193606083), (42.6680766333, -83.2193606083), (42.6679277056, -83.2193276417), (42.6679817056, -83.2193316417), (42.66794, -83.2189), (42.6681268, -83.218887)],  # Start Facing North
    # [(42.6679277056,-83.2193276417),(42.6680766333,-83.2193591583),(42.6681206444,-83.2193606083),(42.6682837222 ,-83.2193403028), (42.668290020, -83.2189080), (42.6681268, -83.218887)], # Start Facing South
    # [(42.668222,-83.218472),(42.6680859611,-83.2184456444),(42.6679600583,-83.2184326556)], # Start Facing North
    # [(42.6679600583,-83.2184326556), (42.6680859611,-83.2184456444),(42.668222,-83.218472)], # Start Facing South
    [],
]

# competition_waypoints = [
#     # Start Nomans, Frist Ramp, Middle Ramp, End Ramp, End Nomans
#     [(42.6682697222 ,-83.2193403028),(42.6681206444,-83.2193606083),(42.66809863885, -83.2193598833), (42.6680766333,-83.2193591583),(42.6679277056,-83.2193276417), (42.6681268, -83.218887)], # Start Facing North
#     # [(42.6679277056,-83.2193276417),(42.6680766333,-83.2193591583),(42.6681206444,-83.2193606083),(42.6682697222,-83.2193403028), (42.6681268, -83.218887)], # Start Facing South
#     # [(42.668222,-83.218472),(42.6680859611,-83.2184456444),(42.6679600583,-83.2184326556)], # Start Facing North
#     # [(42.6679600583,-83.2184326556), (42.6680859611,-83.2184456444),(42.668222,-83.218472)], # Start Facing South
#     [],
# ]

practice_waypoints = [
    [(35.2104852, -97.44193), (35.210483, -97.44207), (35.2104841, -97.4421986), (35.2104819, -97.4423302), (35.2105455, -97.4423329),
     (35.2106096, -97.4423347), (35.2106107, -97.4422153), (35.2106078, -97.4421059), (35.2105575, -97.4420365), (35.2104852, -97.44193)]
]

simulation_waypoints = [
    [(35.19474407276026, -97.43853155838949), (35.19474444006683, -97.43853155838949), (35.19476343444084, -97.43853155838949), (35.194784303857695, -97.43853155838949), (35.19480517366998, -97.43853155838949), (35.194826043658466, -97.43853155838949), (35.194846913681346, -97.43853155838949), (35.19486778361504, -97.43853155838949), (35.19488865357613, -97.43853155838949), (35.19490952352433, -97.43853155838949), (35.19493039351497, -97.43853155908063), (35.19495092553071, -97.43853552083075), (35.19496904627728, -97.43854785157549), (35.19498133489532, -97.43856818587176), (35.194986852840486, -97.43859248699697), (35.19498544142225, -97.43861781734869), (35.19498216591245, -97.43864288876748), (35.194975713591994, -97.43866695750627), (35.19496281070465, -97.43868684693129), (35.19495206700054, -97.43870821496708), (35.19495306210649, -97.43873340525361), (35.19495373669383, -97.43875811465132), (35.1949525483157, -97.43876883071538), (35.19495038594336, -97.43877933293588), (35.1949494009205, -97.43878249462878), (35.19494565765572, -97.43878891692025), (35.19493780222935, -97.43879721568605), (35.19493412569523, -97.43880887472054), (35.194938969788005, -97.43881850730428),
     (35.19494614845655, -97.43882251503061), (35.19495732373083, -97.43884312856127), (35.194963623223224, -97.43886733394329), (35.19496923716838, -97.43889181952832), (35.194969734085255, -97.43891718333695), (35.19496864285037, -97.43894257267571), (35.19496499368157, -97.43896743978841), (35.19495396416087, -97.4389889380561), (35.19494005654008, -97.43900789250587), (35.1949230614983, -97.4390214352402), (35.19490221187155, -97.43902250398679), (35.194881438055944, -97.4390244107245), (35.19486225311435, -97.43903435637161), (35.19484266931345, -97.43904271965387), (35.19482218904126, -97.43903970952904), (35.194802405220315, -97.4390316150356), (35.19478237714437, -97.43902471055158), (35.19476156857851, -97.43902505092527), (35.194740700769096, -97.43902542041042), (35.19471983307143, -97.43902577288944), (35.194700320969666, -97.43903368010614), (35.19468202116007, -97.43904577057661), (35.19466155409562, -97.43904408814375), (35.19464101652687, -97.43903961268632), (35.194620588058406, -97.43903441002884), (35.19460015971028, -97.43902920615665), (35.194579731929494, -97.43902400228444), (35.19456317970144, -97.43901978556296), (35.194562940045294, -97.43901972465927)],
    [(35.194888446528864, -97.43902288310593), (35.194765121094015, -97.43902321108108), (35.19472527123724, -97.4390233172227), (35.19459435968777, -97.43902366555493)],
    [(35.19474411, -97.43852997), (35.19474792, -97.43863678), (35.19484711, -97.43865967), (35.19494247, -97.43875885)],
    # [(35.194816101646275, -97.43853155838949), (35.19496046401714, -97.43853155838949), (35.19501943703302, -97.4386386771562)],
    # [(35.19496918, -97.43855286), (35.19475331, -97.43860863), (35.19493775, -97.43881094)],
    [(35.19496918, -97.43855286), (35.19498780, -97.43859524), (35.19495163, -97.43871339), (35.19495182, -97.43877829), (35.19493406, -97.43879432), (35.19493812, -97.43881810), (35.19495368, -97.43882411), (35.19493357, -97.43902773), (35.19485847, -97.43902381)],
]


# convert string hexadecimal color (ex #AA05CF) to RGB tuple (ex (255, 100, 75))
def hexToRgb(color: str) -> tuple:
    # remove the leading # if it's there
    color = color.removeprefix("#")

    # return [int(channel, 16) for channel in color[::2]] # this doesn't work
    return [int(color[idx:idx+1], 16) for idx in range(0, len(color), 2)]


# create a safetylights message from given parameters
def toSafetyLights(autonomous: bool, eco: bool, mode: int, brightness: int, color: str) -> SafetyLights:
    pkg = SafetyLights()

    pkg.mode = mode
    pkg.autonomous = autonomous
    pkg.eco = eco
    pkg.brightness = brightness

    pkg.red, pkg.green, pkg.blue = hexToRgb(color)

    return pkg


# class AStarNodeConfig:
#     def __init__(self):
#         self.waypointPopDistance = 1.1
#         self.waypointDirection = 0
#         self.useOnlyWaypoints = False
#         self.waypointDelay = 17.5


# reference: https://llego.dev/posts/implementing-the-a-search-algorithm-python/ and https://en.wikipedia.org/wiki/A*_search_algorithm and https://stackabuse.com/courses/graphs-in-python-theory-and-implementation/lessons/a-star-search-algorithm/
# class to represent a node in a graph for doing A* things
class GraphNode:
    # pos is an (x, y) coordinate pair, g_cost is the cost to move from the start to here, and h_cost is the estimated cost to move from here to the goal
    def __init__(self, pos: tuple, g_cost: float, h_cost: float):
        self.pos = pos
        self.x = pos[0] # literally just for convienence
        self.y = pos[1]

        self.g_cost = g_cost # cost from start to here
        self.h_cost = h_cost # cost from here to goal (estimated)
        self.f_cost = g_cost + h_cost # total cost (man this is getting expensive)

        self.parent = None # previous node
    
    # compare this node to another
    def __lt__(self, other: GraphNode):
        # if our total cost is less than we are less (obviously)
        return self.f_cost < other.f_cost
    
class AStar:
    def __init__(self, map_grid):
        self.frontier = [] # list of open nodes we haven't seen
        self.closed = [] # nodes we have visited 
        self.map_grid = map_grid
    
    def search(self, start_node: GraphNode, goal_node: GraphNode):
        self.frontier.append(start_node)

        # while there are still nodes in the frontier
        while self.frontier:
            # sort the list to get the node with the lowest cost first
            self.frontier.sort()
            current_node = self.frontier.pop(0)

            # add the current node to the list of closed nodes
            self.closed.append(current_node)

            # if our current node is the goal node, then we've reached it
            if current_node == goal_node:
                # reached the goal node
                return self.reconstruct_path(goal_node)
                
        
            # check all the neighbors of the node
            neighbors = self.get_neighbors(current_node)

            # for each neighboring node
            for neighbor in neighbors:
                # if we've already searched it
                if neighbor in self.closed:
                    # move on
                    continue
                
                # else, calculate its cost
                g_cost = current_node.g_cost + 1 # cost to move
                h_cost = self.heuristic(neighbor, goal_node) # cost to goal node
                f_cost = g_cost + h_cost

                # check if we've found a cheaper path
                #TODO figure out what this code does
                if neighbor in self.frontier:
                    if neighbor.f_cost > f_cost:
                        self.update_node(neighbor, g_cost, h_cost, current_node)
                else:
                    self.update_node(neighbor, g_cost, h_cost, current_node)
        
        return None # if we haven't returned anything by now, there isn't a path then

    def get_neighbors(self, node: GraphNode) -> list:
        neighbors = []

        # neighbors are just the surrounding nodes
        for x in (-1, 1, 1):
            for y in (-1, 1, 1):
                neighbor_pos = (node.x + x, node.y + y)

                # check if it's in bounds
                if (0 <= neighbor_pos.x <= self.map_grid.shape[0] and 0 <= neighbor_pos.y <= self.map_grid.shape[1]):
                    # check if it's not an obstacle
                    if self.map_grid[neighbor_pos] != 1:
                        neighbors.append(neighbor_pos)

        return neighbors

    # manhattan distance from here to goal
    def heuristic(self, node, goal):
        return abs(node.x - goal.x) + abs(node.y - goal.y)
    
    # reconstruct the path 
    def reconstruct_path(self, goal_node):
        path = [goal_node]
        current = goal_node

        while current.parent != self.start_node:
            path.append(current.parent)
            current = current.parent

        return path[::-1] #reverse path (so it's from start to finish instead of finish to start)
    
    def update_node(self, node, g_cost, h_cost, current_node):
        node.g_cost = g_cost
        node.h_cost = h_cost
        node.f_cost = g_cost + h_cost
        node.parent = current_node



class AStarNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_astar")

        # reset the node
        self.onReset()

        #TODO figure out what this does
        self.latitudeLength = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitudeLength = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value
        # self.config = AStarNodeConfig()

    # initialize the node (not called directly by us)
    def init(self):
        # === subscribers ===
        self.configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/expanded", self.onConfigSpaceReceived, 20)
        self.poseSubscriber = self.create_subscription(Position, "/autonav/position", self.onPoseReceived, 20)
        self.imuSubscriber = self.create_subscription(IMUData, "/autonav/imu", self.onImuReceived, 20)

        # === publishers ===
        self.debugPublisher = self.create_publisher(PathingDebug, "/autonav/debug/astar", 20)
        self.pathPublisher = self.create_publisher(Path, "/autonav/path", 20)
        self.safetyLightsPublisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        self.pathDebugImagePublisher = self.create_publisher(CompressedImage, "/autonav/debug/astar/image", 20)

        # callback timer
        self.mapTimer = self.create_timer(0.1, self.createPath)

        #TODO figure out what this does
        self.resetWhen = -1.0

        # once all subs and pubs are set up we are good to go
        self.set_device_state(DeviceStateEnum.OPERATING)
    

    # === miscellaneous callbacks ===
    def onReset(self):
        #TODO rewrite
        self.imu = None
        self.lastPath = None
        self.position = None
        self.configSpace = None
        self.costMap = None
        self.bestPosition = (0, 0)
        self.waypoints = []
        self.waypointTime = 0.0

    #TODO
    def transition(self, old: SystemState, updated: SystemState):
        pass

    # === sensor callbacks ===
    def onImuReceived(self, msg: IMUData):
        self.imu = msg

    def onPoseReceived(self, msg: Position):
        self.position = msg
    

    # === publisher callbacks ===
    #TODO what does this do?
    # def getAngleDifference(self, to_angle, from_angle):
    #     delta = to_angle - from_angle
    #     delta = (delta + math.pi) % (2 * math.pi) - math.pi
    #     return delta


    #TODO rewrite
    def getWaypointsForDirection(self):
        direction_index = self.config.waypointDirection
        return simulation_waypoints[direction_index] if self.system_mode == SystemModeEnum.SIMULATION else competition_waypoints[direction_index] if self.system_mode == SystemModeEnum.COMPETITION else practice_waypoints[direction_index]



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
    rclpy.init()
    node = AStarNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
