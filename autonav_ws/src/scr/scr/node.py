import threading
from scr.states import DeviceStateEnum, SystemStateEnum, SystemModeEnum
from scr_msgs.srv import UpdateDeviceState, UpdateSystemState, UpdateConfig
from scr_msgs.msg import DeviceState, SystemState, ConfigUpdated
from std_msgs.msg import Float64
from rclpy.node import Node as ROSNode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import scr.constants
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
import json


class Node(ROSNode):
    """
    The base node class for all nodes in the system.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.identifier = node_name

        # Create the callback groups
        self.device_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.system_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.config_updated_callback_group = MutuallyExclusiveCallbackGroup()

        # Create subscriptions and clients
        self.device_state_subscriber = self.create_subscription(DeviceState, scr.constants.Topics.DEVICE_STATE, self.on_device_state, 10)
        self.system_state_subscriber = self.create_subscription(SystemState, scr.constants.Topics.SYSTEM_STATE, self.on_system_state, 10)
        self.config_updated_subscriber = self.create_subscription(ConfigUpdated, scr.constants.Topics.CONFIG_UPDATE, self.on_config_updated, 10)
        
        self.device_state_client = self.create_client(UpdateDeviceState, scr.constants.Services.DEVICE_STATE, callback_group=self.device_state_callback_group)
        self.system_state_client = self.create_client(UpdateSystemState, scr.constants.Services.SYSTEM_STATE, callback_group=self.system_state_callback_group)
        self.config_updated_client = self.create_client(UpdateConfig, scr.constants.Services.CONFIG_UPDATE, callback_group=self.config_updated_callback_group)
        
        self.performance_publisher = self.create_publisher(Float64, scr.constants.Topics.PERFORMANCE_TRACK, 10)

        self.device_state = DeviceStateEnum.OFF
        self.system_state = SystemStateEnum.DISABLED
        self.system_mode = SystemModeEnum.COMPETITION
        self.mobility = False
        self.perf_measurements = {}

        # Create a thread to wait a sec for the node to boot without blocking the main thread
        self.booting_thread = threading.Thread(target=self.booting_worker)
        self.booting_thread.daemon = True
        self.booting_thread.start()

    def booting_worker(self):
        time.sleep(1)
        self.set_device_state(DeviceStateEnum.BOOTING)

    def jdump(self, obj):
        isDict = isinstance(obj, dict)
        if not isDict:
            return json.dumps(obj.__dict__)
        else:
            return json.dumps(obj)

    def on_device_state(self, msg: DeviceState):
        """
        Called when a device state message is received.

        :param msg: The device state message.
        """

        if msg.device is None or msg.device != self.identifier:
            return

        self.device_state = msg.state
        if msg.state == DeviceStateEnum.BOOTING:
            # Get the default config and push it to the server
            defaultConfig = self.get_default_config()
            request = UpdateConfig.Request()
            request.device = self.identifier
            request.json = self.jdump(defaultConfig)

            while not self.config_updated_client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    self.get_logger().error("Interrupted while waiting for service")
                    return
                
            try:
                result = self.config_updated_client.call(request)
                if not result.success:
                    self.get_logger().error("Failed to set default config: " + result.message)
            except Exception as e:
                self.get_logger().error("Failed to set default config: " + str(e))

            # Set the device state to standby and initialize
            self.config_updated(defaultConfig)
            self.set_device_state(DeviceStateEnum.STANDBY)
            self.init()

    def on_system_state(self, msg: SystemState):
        """
        Called when a system state message is received.

        :param msg: The system state message.
        """

        oldState = SystemState()
        oldState.state = self.system_state
        oldState.mode = self.system_mode
        oldState.mobility = self.mobility

        self.system_state_transition(oldState, msg)

        self.system_state = msg.state
        self.system_mode = msg.mode
        self.mobility = msg.mobility

    def on_config_updated(self, msg: ConfigUpdated):
        if msg.device is None or msg.device != self.identifier:
            return
        
        try:
            parsed_json = json.loads(msg.json)
            self.config_updated(parsed_json)
        except Exception as e:
            self.get_logger().error("Failed to parse config update: " + str(e))

    def system_state_transition(self, old_state: SystemState, new_state: SystemState):
        """
        Called after any update to the system state (state, mode, or mobility).

        :param old_state: The old system state.
        :param new_state: The new system state.
        """

        pass

    def init():
        """
        Called after a node is first discovered by the network. The device state will be set to BOOTING
        """

        pass

    def config_updated(self, json):
        """
        Called when the configuration is updated.
        """

        pass

    def get_default_config(self):
        """
        Gets the default configuration for the node.
        """

        self.get_logger().warn("Default config method not overridden: get_default_config()")
        return {}

    def set_device_state(self, state: DeviceStateEnum):
        """
        Sets the device state.

        :param state: The new device state.
        """

        request = UpdateDeviceState.Request()
        request.device = self.identifier
        request.state = state

        while not self.device_state_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for service")
                return

        try:
            result = self.device_state_client.call(request)
            if not result.success:
                self.get_logger().error("Failed to set device state: " + result.message)
        except Exception as e:
            self.get_logger().error("Failed to set device state: " + str(e))

    def set_system_total_state(self, state: SystemStateEnum, mode: SystemModeEnum, mobility: bool):
        """
        Sets the system state.

        :param state: The new system state.
        :param mode: The new system mode.
        :param mobility: The new mobility state.
        """

        request = UpdateSystemState.Request()
        request.state = state
        request.mode = mode
        request.mobility = mobility

        while not self.system_state_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for service")
                return

        try:
            result = self.system_state_client.call(request)
            if not result.success:
                self.get_logger().error("Failed to set system state: " + result.message)
        except Exception as e:
            self.get_logger().error("Failed to set system state: " + str(e))

    def set_system_state(self, state: SystemStateEnum):
        """
        Sets the system state.

        :param state: The new system state.
        """

        self.set_system_total_state(state, self.system_mode, self.mobility)

    def set_system_mode(self, mode: SystemModeEnum):
        """
        Sets the system mode.

        :param mode: The new system mode.
        """

        self.set_system_total_state(self.system_state, mode, self.mobility)

    def set_system_mobility(self, mobility: bool):
        """
        Sets the system mobility.

        :param mobility: The new system mobility.
        """

        self.set_system_total_state(
            self.system_state, self.system_mode, mobility)
        
    def perf_start(self, name: str):
        """
        Starts a performance measurement.

        :param self: The name of the measurement.
        """

        self.perf_measurements[name] = time.time_ns() // 1_000_000

    def perf_end(self, name: str):
        """
        Ends a performance measurement.

        :param self: The name of the measurement.
        """

        if name not in self.perf_measurements:
            self.get_logger().error("There was no performance measurement with the name '" + name + "'")
            return

        elapsed_time = time.time_ns() // 1_000_000 - self.perf_measurements[name]
        self.performance_publisher.publish(Float64(data=elapsed_time))
        del self.perf_measurements[name]

    def run_node(*node: ROSNode):
        """
        Runs the node with the correct ROS parameters and specifications

        :param node: The node to run.
        """

        executor = MultiThreadedExecutor()
        for n in node:
            executor.add_node(n)
        executor.spin()
        executor.shutdown()

    def log(self, message: str):
        rclpy.logging.get_logger("scr." + self.identifier).info(message)
