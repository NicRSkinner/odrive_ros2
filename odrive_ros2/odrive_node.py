"""
ROS2 package for interacting with ODrive.

Package is very basic currently, features to be added in the future.
"""
import rclpy

from rclpy.qos import LivelinessPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

import odrive
from odrive.enums import *

import time

from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Bool

from bfr_msgs.msg import AutomationState

class ODriveNode(Node):
    """ODrive package node. Contains all data for interacting with ROS2 package."""

    def run_callback(self, msg) -> None:
        """Execute start/stop based on global ready message feedback."""
        if msg.data is True and self.odrive_found is False:
            self.find_odrive()

        if msg.data is True and self.running is False and self.odrive_found is True:
            self.get_logger().info("ODrive received start/run command.")
            if self.motors_calibrated is False:
                self.get_logger().info("Motors uncalibrated, running calibration.")
                #self.calibrate_motors()

            self.get_logger().info("Running motors.")
            self.run()
        elif msg.data is False and self.running is True:
            self.get_logger().info("ODrive received stop command.")
            self.get_logger().info("Stopping motors.")
            self.stop()

    def run(self) -> None:
        """Run motors and publish ready signal."""
        self.get_logger().info("Starting Motors...")
        self.automation_state = 4
        self.publish_state()
        #self.start_motors()
        self.get_logger().info("Motors ready.")

        self.running = True
        self.drive.axis0.controller.input_vel = 0
        self.drive.axis1.controller.input_vel = 0

        self.automation_state = 3
        self.publish_state()

    def stop(self) -> None:
        """Stop motors and publish not ready signal."""
        self.get_logger().info("Stopping Motors...")
        self.automation_state = 5
        self.publish_state()
        self.stop_motors()
        self.get_logger().info("Motors stopped.")

        self.running = False
        self.automation_state = 1
        self.publish_state()

    def publish_state(self) -> None:
        """Publish state signal, so other packages can run."""
        msg = AutomationState()
        msg.state = self.automation_state
        self.state_publisher.publish(msg)

    def publish_encoder(self) -> None:
        """Publish encoder positional data."""
        msg0 = Int32()
        msg1 = Int32()

        msg0 = self.drive.axis0.encoder.shadow_count
        msg1 = self.drive.axis1.encoder.shadow_count

        self.motor0_encoder_publisher.publish(msg0)
        self.motor1_encoder_publisher.publish(msg1)

    def motor0_velocity_callback(self, msg) -> None:
        """Set velocity of axis0 based on a ROS2 message."""
        if self.running is True:
            # This motor spins backwards.
            # For some reason it has to be this way, as the calibration for this motor is backwards.
            self.drive.axis0.controller.input_vel = msg.data

    def motor1_velocity_callback(self, msg) -> None:
        """Set position setpoint of axis1 based on a ROS2 message."""
        if self.running is True:
            self.drive.axis1.controller.input_vel = -msg.data


    def publish_vbus_voltage(self) -> None:
        """Publish the current bus voltage."""
        msg = Float32()
        msg.data = self.drive.vbus_voltage
        self.vbus_voltage_publisher.publish(msg)


    def __init__(self):
        """Initialize ODrive ROS2 package. Start motors after initialized."""
        super().__init__("odrive")
        self.get_logger().info("ODrive node started.")

        self.running = False
        self.odrive_found = False
        self.motors_calibrated = False
        self.automation_state = 1

        deadlineDuration = Duration(seconds=2)
        base_qos_profile = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
			depth=1, liveliness=QoSLivelinessPolicy.AUTOMATIC, liveliness_lease_duration=deadlineDuration)

        self.state_publisher = self.create_publisher(AutomationState, "/odrive0/automationstate", 3)
        #self.velocity_publisher = self.create_publisher(Float32, "/odrive0/motor0/current_velocity", 3)
        self.motor0_encoder_publisher = self.create_publisher(Int32, "/odrive0/motor0/encoder_counts", 3)
        self.motor1_encoder_publisher = self.create_publisher(Int32, "/odrive0/motor1/encoder_counts", 3)

        self.motor0_velocity_subscriber = self.create_subscription(Float32, "/odrive0/motor0/input_vel", self.motor0_velocity_callback, 10)
        self.motor1_velocity_subscriber = self.create_subscription(Float32, "/odrive0/motor1/input_vel", self.motor1_velocity_callback, 10)
        self.global_run_subscriber = self.create_subscription(Bool, "/safety/run", self.run_callback, qos_profile=1)
        self.force_run_subscriber = self.create_subscription(Bool, "/odrive0/run", self.run_callback, 10)
        self.vbus_voltage_publisher = self.create_publisher(Float32, "/odrive0/vbus_voltage", 3)

        self.get_logger().info("Starting state publisher timer")
        self.create_timer(0.1, self.publish_state)

        self.get_logger().info("Attempting to find ODrive.")
        self.find_odrive()
        self.get_logger().info("ODrive found")

        self.get_logger().info("Starting timers...")
        #self.create_timer(0.1, self.publish_encoder)

    def find_odrive(self):
        """Find ODrive. Times out if not found."""
        self.get_logger().info("Attempting to find ODrive...")

        try:
            self.drive = odrive.find_any(timeout=3)
        except TimeoutError:
            self.get_logger().error("ODrive not found/timed out. Plug in ODrive and retry.")
            return

        if self.drive is not None:
            self.get_logger().info("ODrive found.")
            #self.get_logger().info("ODrive bus voltage: " % self.drive.vbus_voltage)
            self.odrive_found = True
        else:
            self.get_logger().error("ODrive not found. Reason unknown.")


    def calibrate_motors(self):
        """Run initial calibration of both motors."""
        if self.drive.axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and self.drive.axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.get_logger().info("Motor pre calibrated, trying to start motors")
            self.motors_calibrated = True
            self.start_motors()
            return

        self.get_logger().info("Starting motor calibration.")
        self.drive.axis0.requested_state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.drive.axis1.requested_state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        time.sleep(10)

        if self.drive.axis0.current_state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION or self.drive.axis1.current_state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
            self.get_logger().info("Motors calibrating.")
            time.sleep(1)

        self.get_logger().info("Calibration finished.")

        self.motors_calibrated = True

        self.start_motors()

    def start_motors(self):
        """Start motors for ODrive. Putting them in a run state."""
        if self.drive.axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and self.drive.axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            return

        while self.drive.axis0.requested_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        while self.drive.axis1.requested_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        self.drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def stop_motors(self):
        """Stop motors from ODrive. Putting them in an idle state."""
        if self.drive.axis0.current_state == AXIS_STATE_IDLE and self.drive.axis1.current_state == AXIS_STATE_IDLE:
            return

        self.drive.axis0.requested_state = AXIS_STATE_IDLE
        self.drive.axis1.requested_state = AXIS_STATE_IDLE


def main(args=None):
    """Entry point."""
    try:
        rclpy.init(args=args)
        node = ODriveNode()
        rclpy.spin(node)
        node.destroy_node(self=node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
