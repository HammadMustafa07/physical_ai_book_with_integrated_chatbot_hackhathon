"""
Voice Command Processing Example for ROS 2

This example demonstrates how to implement a basic voice command processing pipeline
that recognizes speech, maps to intents, and publishes ROS 2 messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import threading


class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for voice status
        self.status_publisher = self.create_publisher(String, '/voice_status', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set minimum energy threshold for silence detection
        self.recognizer.dynamic_energy_threshold = True

        # Start voice recognition in a separate thread
        self.voice_thread = threading.Thread(target=self.listen_for_commands)
        self.voice_thread.daemon = True
        self.voice_thread.start()

        self.get_logger().info("Voice Command Processor initialized")

    def listen_for_commands(self):
        """Continuously listen for voice commands"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info("Listening for voice commands...")

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=3.0)

                # Recognize speech
                command_text = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Heard command: {command_text}")

                # Process the command
                self.process_command(command_text)

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Speech recognition error: {e}")

    def process_command(self, command_text):
        """Process the recognized command and execute appropriate action"""
        # Publish the recognized command for status
        status_msg = String()
        status_msg.data = f"Processing: {command_text}"
        self.status_publisher.publish(status_msg)

        # Map voice commands to robot actions
        if "move forward" in command_text or "go forward" in command_text:
            self.move_forward()
        elif "move backward" in command_text or "go back" in command_text:
            self.move_backward()
        elif "turn left" in command_text:
            self.turn_left()
        elif "turn right" in command_text:
            self.turn_right()
        elif "stop" in command_text or "halt" in command_text:
            self.stop_robot()
        elif "spin left" in command_text:
            self.spin_left()
        elif "spin right" in command_text:
            self.spin_right()
        else:
            self.get_logger().warn(f"Unknown command: {command_text}")
            # Publish unknown command status
            status_msg = String()
            status_msg.data = f"Unknown command: {command_text}"
            self.status_publisher.publish(status_msg)

    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.5  # Forward speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Moving forward")

    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -0.5  # Backward speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Moving backward")

    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.angular.z = 0.5  # Left turn speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Turning left")

    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.angular.z = -0.5  # Right turn speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Turning right")

    def spin_left(self):
        """Spin robot left in place"""
        twist = Twist()
        twist.angular.z = 1.0  # Spin left speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Spinning left")

    def spin_right(self):
        """Spin robot right in place"""
        twist = Twist()
        twist.angular.z = -1.0  # Spin right speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Spinning right")

    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        # All velocities are zero by default
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Stopping robot")


def main(args=None):
    rclpy.init(args=args)

    voice_processor = VoiceCommandProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()