#!/usr/bin/env python3

import rclpy
from scr.node import Node
from scr.states import DeviceStateEnum
# from scr_msgs.msg import SystemState, DeviceState, Log, ConfigurationInstruction
from scr_msgs.msg import SystemState, DeviceState, ConfigUpdated
from scr_msgs.srv import SetSystemState, UpdateConfig
from std_msgs.msg import Empty
from autonav_msgs.msg import Position, MotorFeedback, MotorInput, MotorControllerDebug, ObjectDetection, PathingDebug, GPSFeedback, IMUData, Conbus
from sensor_msgs.msg import CompressedImage
import asyncio
import websockets
import threading
import json
import base64
import time

big_loop = asyncio.new_event_loop()

class Limiter:
	def __init__(self) -> None:
		self.limits = {}
		self.nextAllowance = {}

	# Sets a limit for how many times per second a key can be used
	def setLimit(self, key, limit):
		self.limits[key] = limit
		self.nextAllowance[key] = 0
		
	# If it can be used, returns true and decrements the remaining uses
	def use(self, key):
		if key not in self.limits:
			return True
		
		nextUsageAfter = self.nextAllowance[key]
		if nextUsageAfter == 0:
			self.nextAllowance[key] = time.time() + (1.0 / self.limits[key])
			return True
		
		if time.time() >= nextUsageAfter:
			self.nextAllowance[key] = time.time() + (1.0 / self.limits[key])
			return True
		
		return False

class BroadcastNode(Node):
	def __init__(self):
		super().__init__("autonav_display_broadcast")

		self.port = 8023
		self.host = "0.0.0.0"
		self.sendMap = {}
		self.socketMap = {}

		self.limiter = Limiter()
		self.limiter.setLimit("/autonav/MotorInput", 5)
		self.limiter.setLimit("/autonav/MotorFeedback", 5)
		self.limiter.setLimit("/autonav/MotorControllerDebug", 1)
		self.limiter.setLimit("/autonav/imu", 5)
		self.limiter.setLimit("/autonav/gps", 5)
		self.limiter.setLimit("/autonav/position", 5)
		self.limiter.setLimit("/autonav/camera/compressed/left", 2)
		self.limiter.setLimit("/autonav/camera/compressed/right", 2)
		self.limiter.setLimit("/autonav/cfg_space/raw/image/left", 5)
		self.limiter.setLimit("/autonav/cfg_space/raw/image/right", 5)
		self.limiter.setLimit("/autonav/cfg_space/raw/image/left_small", 5)
		self.limiter.setLimit("/autonav/cfg_space/raw/image/right_small", 5)
		self.limiter.setLimit("/autonav/cfg_space/raw/debug", 5)
		self.limiter.setLimit("/autonav/debug/astar/image", 5)

		self.get_logger().info("Broadcasting on ws://{}:{}".format(self.host, self.port))

		self.systemStateSubscriber = self.create_subscription(SystemState, "/scr/system_state", self.systemStateCallback, 20)
		self.deviceStateSubscriber = self.create_subscription(DeviceState, "/scr/device_state", self.deviceStateCallback, 20)
		self.broadcastPublisher = self.create_publisher(Empty, "/scr/state/broadcast", 20)
		self.configurationInstructionSubscriber = self.create_subscription(ConfigUpdated, "/scr/config_updated", self.configurationInstructionCallback, 100)
		# self.logSubscriber = self.create_subscription(Log, "/scr/logging", self.logCallback, 20)
		# self.configurationInstructionPublisher = self.create_publisher(ConfigUpdated, "/scr/config_updated", 100)
		self.configUpdateClient = self.create_client(UpdateConfig, "/scr/update_config_client")

		self.positionSubscriber = self.create_subscription(Position, "/autonav/position", self.positionCallback, 20)
		self.motorFeedbackSubscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.motorFeedbackCallback, 20)
		self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.motorInputCallback, 20)
		self.motorControllerDebugSubscriber = self.create_subscription(MotorControllerDebug, "/autonav/MotorControllerDebug", self.motorControllerDebugCallback, 20)
		self.objectDetectionSubscriber = self.create_subscription(ObjectDetection, "/autonav/ObjectDetection", self.objectDetectionCallback, 20)
		self.pathingDebugSubscriber = self.create_subscription(PathingDebug, "/autonav/cfg_space/expanded/image", self.pathingDebugCallback, 20)
		self.gpsFeedbackSubscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.gpsFeedbackCallback, 20)
		self.imuDataSubscriber = self.create_subscription(IMUData, "/autonav/imu", self.imuDataCallback, 20)
		self.conbusSubscriber = self.create_subscription(Conbus, "/autonav/conbus/data", self.conbusCallback, 100)
		self.conbusPublisher = self.create_publisher(Conbus, "/autonav/conbus/instruction", 100)

		self.systemStateService = self.create_client(SetSystemState, "/scr/state/set_system_state")

		self.cameraSubscriberLeft = self.create_subscription(CompressedImage, "/autonav/camera/compressed/left", self.cameraCallbackLeft, self.qos_profile)
		self.cameraSubscriberRight = self.create_subscription(CompressedImage, "/autonav/camera/compressed/right", self.cameraCallbackRight, self.qos_profile)
		self.filteredSubscriberLeft = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/left", self.filteredCallbackLeft, self.qos_profile)
		self.filteredSubscriberRight = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/right", self.filteredCallbackRight, self.qos_profile)
		self.filteredSubscriberLeft = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/left_small", self.filteredCallbackLeftSmall, self.qos_profile)
		self.filteredSubscriberRight = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/right_small", self.filteredCallbackRightSmall, self.qos_profile)
		self.filteredSubscriberCombined = self.create_subscription(CompressedImage, "/autonav/cfg_space/combined/image", self.filteredCallbackCombined, self.qos_profile)
		self.bigboiSubscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/debug", self.bigboiCallback, self.qos_profile)
		self.debugAStarSubscriber = self.create_subscription(CompressedImage, "/autonav/debug/astar/image", self.debugAStarCallback, self.qos_profile)
		
		self.get_logger().info("Starting event loop")

		self.loop_thread = threading.Thread(target=self.loopthread)
		self.loop_thread.start()

	def loopthread(self):
		asyncio.set_event_loop(big_loop)
		big_loop.run_until_complete(self.start())
	
	async def start(self):
		async with websockets.serve(self.handler, self.host, self.port):
			self.get_logger().info("Started websocket server")
			await asyncio.Future()

	def getUserIdFromSocket(self, websocket):
		try:
			return websocket.path.split("?")[1].split("=")[1]
		except:
			return None

	def pushSendQueue(self, message, unique_id=None):
		if len(self.sendMap) == 0:
			return

		if unique_id is None:
			for unique_id in self.sendMap:
				self.sendMap[unique_id].append(message)
		else:
			self.sendMap[unique_id].append(message)

	async def producer(self, websocket):
		unqiue_id = self.getUserIdFromSocket(websocket)
		while True:
			if len(self.sendMap[unqiue_id]) > 0:
				await websocket.send(self.sendMap[unqiue_id].pop(0))
			else:
				await asyncio.sleep(0.01)

	async def consumer(self, websocket):
		self.get_logger().info("New websocket connection")
		unique_id = self.getUserIdFromSocket(websocket)
		async for message in websocket:
			obj = json.loads(message)
			if obj["op"] == "broadcast":
				self.broadcastPublisher.publish(Empty())

			if obj["op"] == "configuration" and "device" in obj and "json" in obj:
				self.log("Received configuration instruction for " + obj["device"])
				config_packet = UpdateConfig.Request()
				config_packet.device = obj["device"]
				config_packet.json = json.dumps(obj["json"])
				self.configUpdateClient.call_async(config_packet)

			if obj["op"] == "get_nodes":
				nodes = self.get_node_names()
				for i in range(len(nodes)):
					nodes[i] = nodes[i].replace("/", "")
				node_states = {}
				for identifier in nodes:
					node_states[identifier] = self.device_states[identifier] if identifier in self.device_states else 0
				self.pushSendQueue(json.dumps({
					"op": "get_nodes_callback",
					"nodes": nodes,
					"states": node_states,
					"configs": self.node_configs,
					"system": {
						"state": self.system_state,
						"mode": self.system_mode,
						"mobility": self.mobility
					}
				}), unique_id)

			if obj["op"] == "set_system_state":
				self.set_system_total_state(int(obj["state"]), self.system_mode, bool(obj["mobility"]))

			if obj["op"] == "conbus":
				id = int(obj["id"])
				data = list(map(lambda x: int(x), obj["data"]))
				msg = Conbus()
				msg.id = id
				msg.data = data
				msg.iterator = int(obj["iterator"]) if "iterator" in obj else 0
				self.conbusPublisher.publish(msg)

	async def handler(self, websocket):
		unique_id = self.getUserIdFromSocket(websocket)
		if unique_id in self.socketMap or unique_id is None:
			await websocket.close()
			return

		self.socketMap[unique_id] = websocket
		self.sendMap[unique_id] = []

		consumer_task = asyncio.create_task(self.consumer(websocket))
		producer_task = asyncio.create_task(self.producer(websocket))
		pending = await asyncio.wait(
			[consumer_task, producer_task],
			return_when=asyncio.FIRST_COMPLETED
		)
		for task in pending:
			for t in task:
				t.cancel()

		del self.socketMap[unique_id]
		del self.sendMap[unique_id]

	def systemStateCallback(self, msg: SystemState):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/state/system",
			"state": msg.state,
			"estop": msg.estop,
			"mobility": msg.mobility,
			"mode": msg.mode
		}))

	def deviceStateCallback(self, msg: DeviceState):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/state/device",
			"state": msg.state,
			"device": msg.device
		}))

	# def logCallback(self, msg: Log):
	# 	if msg.node == "autonav_display_broadcast":
	# 		return
	# 	self.pushSendQueue(json.dumps({
	# 		"op": "data",
	# 		"topic": "/scr/logging",
	# 		"data": msg.data,
	# 		"node": msg.node
	# 	}))

	def configurationInstructionCallback(self, msg: ConfigUpdated):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/configuration",
			"data": msg.json
		}))

	def positionCallback(self, msg: Position):
		if not self.limiter.use("/autonav/position"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/position",
			"x": msg.x,
			"y": msg.y,
			"theta": msg.theta,
			"latitude": msg.latitude,
			"longitude": msg.longitude
		}))

	def motorInputCallback(self, msg: MotorInput):
		if not self.limiter.use("/autonav/MotorInput"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/MotorInput",
			"angular_velocity": msg.angular_velocity,
			"forward_velocity": msg.forward_velocity
		}))

	def motorFeedbackCallback(self, msg: MotorFeedback):
		if not self.limiter.use("/autonav/MotorFeedback"):
			return
		
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/MotorFeedback",
			"delta_x": msg.delta_x,
			"delta_y": msg.delta_y,
			"delta_theta": msg.delta_theta
		}))

	def imuDataCallback(self, msg: IMUData):
		if not self.limiter.use("/autonav/imu"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/imu",
			"accel_x": msg.accel_x,
			"accel_y": msg.accel_y,
			"accel_z": msg.accel_z,
			"angular_x": msg.angular_x,
			"angular_y": msg.angular_y,
			"angular_z": msg.angular_z,
			"yaw": msg.yaw,
			"pitch": msg.pitch,
			"roll": msg.roll
		}))

	def gpsFeedbackCallback(self, msg: GPSFeedback):
		if not self.limiter.use("/autonav/gps"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/gps",
			"latitude": msg.latitude,
			"longitude": msg.longitude,
			"altitude": msg.altitude,
			"satellites": msg.satellites,
			"is_locked": msg.is_locked,
			"gps_fix": msg.gps_fix
		}))

	def pathingDebugCallback(self, msg: PathingDebug):
		if not self.limiter.use("/autonav/debug/astar"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/debug/astar",
			"desired_heading": msg.desired_heading,
			"desired_latitude": msg.desired_latitude,
			"desired_longitude": msg.desired_longitude,
			"distance_to_destination": msg.distance_to_destination,
			"waypoints": msg.waypoints.tolist(),
			"time_until_use_waypoints": msg.time_until_use_waypoints,
		}))

	def objectDetectionCallback(self, msg: ObjectDetection):
		if not self.limiter.use("/autonav/ObjectDetection"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/ObjectDetection",
			"sensor_1": msg.sensor_1,
			"sensor_2": msg.sensor_2,
			"sensor_3": msg.sensor_3
		}))

	def motorControllerDebugCallback(self, msg: MotorControllerDebug):
		if not self.limiter.use("/autonav/MotorControllerDebug"):
			return

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/MotorControllerDebug",
			"current_forward_velocity": msg.current_forward_velocity,
			"forward_velocity_setpoint": msg.forward_velocity_setpoint,
			"current_angular_velocity": msg.current_angular_velocity,
			"angular_velocity_setpoint": msg.angular_velocity_setpoint,
			"left_motor_output": msg.left_motor_output,
			"right_motor_output": msg.right_motor_output
		}))

	def cameraCallbackLeft(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/camera/compressed/left"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/camera/compressed/left",
			"format": msg.format,
			"data": base64_str
		}))

	def cameraCallbackRight(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/camera/compressed/right"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/camera/compressed/right",
			"format": msg.format,
			"data": base64_str
		}))

	def filteredCallbackLeft(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/cfg_space/raw/image/left"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/raw/image/left",
			"format": msg.format,
			"data": base64_str
		}))

	def filteredCallbackRight(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/cfg_space/raw/image/right"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/raw/image/right",
			"format": msg.format,
			"data": base64_str
		}))

	def filteredCallbackLeftSmall(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/cfg_space/raw/image/left_small"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/raw/image/left_small",
			"format": msg.format,
			"data": base64_str
		}))

	def filteredCallbackRightSmall(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/cfg_space/raw/image/right_small"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/raw/image/right_small",
			"format": msg.format,
			"data": base64_str
		}))

	def filteredCallbackCombined(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/cfg_space/combined/image"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/combined/image",
			"format": msg.format,
			"data": base64_str
		}))

	def bigboiCallback(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/cfg_space/raw/debug"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/raw/debug",
			"format": msg.format,
			"data": base64_str
		}))

	def debugAStarCallback(self, msg: CompressedImage):
		if not self.limiter.use("/autonav/debug/astar/image"):
			return

		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/debug/astar/image",
			"format": msg.format,
			"data": base64_str
		}))

	def conbusCallback(self, msg: Conbus):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/conbus",
			"id": msg.id,
			"data": msg.data.tolist(),
			"iterator": msg.iterator
		}))

	def init(self):
		self.set_device_state(DeviceStateEnum.OPERATING)



def main():
	rclpy.init()
	node = BroadcastNode()
	Node.run_node(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()