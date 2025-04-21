import rclpy
import time
import math
import threading
from enum import IntFlag, auto
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleCommand
)

class ControlModes(IntFlag):
    POSITION = auto()
    VELOCITY = auto()
    # for now we only use POSITION

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        self.get_logger().info("Offboard Control Node Alive!")

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_pub = self.create_publisher(OffboardControlMode,
                                                          '/fmu/in/offboard_control_mode',
                                                          qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
                                                         '/fmu/in/vehicle_command',
                                                         qos)
        self.traj_setpoint_pub = self.create_publisher(TrajectorySetpoint,
                                                       '/fmu/in/trajectory_setpoint',
                                                       qos)

        # Subscribers
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self._local_pos_cb, qos)
        self.create_subscription(VehicleGlobalPosition,
                                 '/fmu/out/vehicle_global_position',
                                 self._global_pos_cb, qos)
        self.create_subscription(String,
                                 '/dexi/offboard_manager',
                                 self._launch_mission_cb, qos)

        # State
        self.x = self.y = self.z = 0.0
        self.heading = 0.0
        self.alt = 0.0  # MSL altitude from global pos
        self.control_mode = ControlModes.POSITION

        # Heartbeat thread (10 Hz)
        self._hb_run = True
        hb = threading.Thread(target=self._heartbeat_loop)
        hb.daemon = True
        hb.start()

    def _heartbeat_loop(self):
        while self._hb_run:
            self._publish_offboard_control_mode()
            time.sleep(0.1)

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = True
        # velocity/accel/attitude/body_rate left false
        self.offboard_control_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # arm
        self._send_vehicle_command(msg)
        self.get_logger().info('Arm command sent')

    def takeoff(self, climb_alt):
        # NAV_TAKEOFF: param7 = climb_alt above current MSL
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param7 = float(self.alt + climb_alt)
        self._send_vehicle_command(msg)
        self.get_logger().info(f'Takeoff to +{climb_alt} m sent')

    def loiter(self, altitude):
        # DO_CHANGE_ALTITUDE = 186
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE
        msg.param1 = float(altitude)
        self._send_vehicle_command(msg)
        self.get_logger().info(f'Loiter at {altitude} m sent')

    def enable_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # custom
        msg.param2 = 6.0  # OFFBOARD
        self._send_vehicle_command(msg)
        self.get_logger().info('Switching to Offboard mode')

    def fly_forward(self, distance):
        # compute new setpoint in the current heading
        new_x = self.x + distance * math.cos(self.heading)
        new_y = self.y + distance * math.sin(self.heading)
        self._send_position_setpoint(new_x, new_y, self.z)
        self.get_logger().info(f'Flying forward {distance} m')

    def land(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self._send_vehicle_command(msg)
        self.get_logger().info('Land command sent')

    def _send_vehicle_command(self, msg: VehicleCommand):
        # fill defaults
        msg.timestamp = self.get_timestamp()
        if msg.target_system == 0:
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _send_position_setpoint(self, x, y, z, yaw=None):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw) if yaw is not None else self.heading
        self.traj_setpoint_pub.publish(msg)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.heading = msg.heading

    def _global_pos_cb(self, msg: VehicleGlobalPosition):
        self.alt = msg.alt

    def _launch_mission_cb(self, msg: String):
        if msg.data == "launch":
            threading.Thread(target=self._mission_sequence).start()
        else:
            self.get_logger().info(f'Unknown mission command: {msg.data}')

    def _mission_sequence(self):
        # 1. Arm
        self.arm()
        time.sleep(1)

        # 2. Take off +2 m
        self.takeoff(2.0)
        time.sleep(8)

        # 3. Loiter at 2 m
        self.loiter(2.0)
        time.sleep(5)

        # 4. Switch to offboard
        self.enable_offboard_mode()
        time.sleep(1)

        # 5. Fly forward 20 m
        self.fly_forward(20.0)
        time.sleep(10)

        # 6. Land
        self.land()

    def get_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def destroy_node(self):
        # stop heartbeat
        self._hb_run = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
