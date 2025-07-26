#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_tcp_frame = 'tool0'  # Robot TCP frame
        self.object_frame = 'kost'  # The object's frame

        # Initial transform storage
        self.initial_tcp_translation = None
        self.initial_object_translation = None

        # Data storage for relative movements
        self.time_series = []
        self.robot_translations = {'x': [], 'y': [], 'z': []}
        self.object_translations = {'x': [], 'y': [], 'z': []}
        self.errors_translations = {'x': [], 'y': [], 'z': []}

        # Timer callback (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Get the robot TCP's transform relative to the world
            tcp_transform = self.tf_buffer.lookup_transform('world', self.object_frame, rclpy.time.Time())
            # Get the object's transform relative to the robot TCP frame
            object_transform = self.tf_buffer.lookup_transform(self.object_frame, self.robot_tcp_frame, rclpy.time.Time())

            # Robot TCP Translation
            tcp_translation = np.array([tcp_transform.transform.translation.x,
                                        tcp_transform.transform.translation.y,
                                        tcp_transform.transform.translation.z])

            # Object Translation in TCP frame
            object_translation = np.array([object_transform.transform.translation.x,
                                           object_transform.transform.translation.y,
                                           object_transform.transform.translation.z])

            # Store the initial positions if they haven't been stored yet
            if self.initial_tcp_translation is None:
                self.initial_tcp_translation = tcp_translation
                self.initial_object_translation = object_translation

            # Compute the relative position for the object (starting from zero)
            relative_object_translation = object_translation - self.initial_object_translation

            # Store time for plotting
            self.time_series.append(self.get_clock().now().nanoseconds / 1e9)  # Convert to seconds

            # Store object data for plotting
            self.object_translations['x'].append(relative_object_translation[0])
            self.object_translations['y'].append(relative_object_translation[1])
            self.object_translations['z'].append(relative_object_translation[2])

            # Store robot data for plotting
            self.robot_translations['x'].append(tcp_translation[0] - self.initial_tcp_translation[0])
            self.robot_translations['y'].append(tcp_translation[1] - self.initial_tcp_translation[1])
            self.robot_translations['z'].append(tcp_translation[2] - self.initial_tcp_translation[2])

            # Calculate errors
            self.errors_translations['x'].append(self.robot_translations['x'][-1] - self.object_translations['x'][-1])
            self.errors_translations['y'].append(self.robot_translations['y'][-1] - self.object_translations['y'][-1])
            self.errors_translations['z'].append(self.robot_translations['z'][-1] - self.object_translations['z'][-1])

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

    def plot_results(self):
        # Convert lists to numpy arrays for easier plotting
        time_series = np.array(self.time_series)

        # Create subplots: one for translations and one for errors
        fig, axs = plt.subplots(2, 1, figsize=(12, 10))
        fig.tight_layout(pad=4.0)

        # Plot translations (X, Y, Z)
        axs[0].plot(time_series, self.object_translations['x'], label='Object X', color='r')
        axs[0].plot(time_series, self.robot_translations['x'], label='Robot X', color='b', linestyle='--')
        axs[0].set_title('Translations (Object vs Robot in TCP Frame)')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Position (meters)')
        axs[0].legend()

        # axs[0].plot(time_series, self.object_translations['y'], label='Object Y', color='r', alpha=0.5)
        # axs[0].plot(time_series, self.robot_translations['y'], label='Robot Y', color='b', alpha=0.5, linestyle='--')
        # axs[0].plot(time_series, self.object_translations['z'], label='Object Z', color='r', alpha=0.5)
        # axs[0].plot(time_series, self.robot_translations['z'], label='Robot Z', color='b', alpha=0.5, linestyle='--')

        # Plot errors
        axs[1].plot(time_series[:len(self.errors_translations['x'])], self.errors_translations['x'], label='Error X', color='g')
        axs[1].plot(time_series[:len(self.errors_translations['y'])], self.errors_translations['y'], label='Error Y', color='m')
        axs[1].plot(time_series[:len(self.errors_translations['z'])], self.errors_translations['z'], label='Error Z', color='c')
        axs[1].set_title('Errors (Robot vs Object in TCP Frame)')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Error (meters)')
        axs[1].legend()

        # Set limits for error plots to zoom in on small values
        axs[1].set_ylim(-0.01, 0.01)  # Adjust this range as needed for your specific error values

        # Save the plot as a .jpg file
        plt.savefig("robot_vs_object_motion_with_errors.jpg", format='jpg', dpi=300)
        plt.close(fig)  # Close the figure to free memory

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Plot the results once the node stops
    node.plot_results()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()















"""
def plot_results(self):
        # Convert lists to numpy arrays for easier plotting
        time_series = np.array(self.time_series)

        fig, axs = plt.subplots(3, 3, figsize=(12, 8))
        fig.tight_layout(pad=3.0)

        # Plot translations (X, Y, Z)
        axs[0, 0].plot(time_series, self.object_translations['x'], label='Object X', color='r')
        axs[0, 0].plot(time_series, self.robot_translations['x'], label='Robot X', color='b')
        axs[0, 0].set_title('Translation X (relative)')
        axs[0, 0].legend()

        axs[0, 1].plot(time_series, self.object_translations['y'], label='Object Y', color='r')
        axs[0, 1].plot(time_series, self.robot_translations['y'], label='Robot Y', color='b')
        axs[0, 1].set_title('Translation Y (relative)')
        axs[0, 1].legend()

        axs[0, 2].plot(time_series, self.object_translations['z'], label='Object Z', color='r')
        axs[0, 2].plot(time_series, self.robot_translations['z'], label='Robot Z', color='b')
        axs[0, 2].set_title('Translation Z (relative)')
        axs[0, 2].legend()

        # Plot rotations (Roll, Pitch, Yaw)
        axs[1, 0].plot(time_series, self.object_rotations['roll'], label='Object Roll', color='r')
        axs[1, 0].plot(time_series, self.robot_rotations['roll'], label='Robot Roll', color='b')
        axs[1, 0].set_title('Rotation Roll (relative)')
        axs[1, 0].legend()

        axs[1, 1].plot(time_series, self.object_rotations['pitch'], label='Object Pitch', color='r')
        axs[1, 1].plot(time_series, self.robot_rotations['pitch'], label='Robot Pitch', color='b')
        axs[1, 1].set_title('Rotation Pitch (relative)')
        axs[1, 1].legend()

        axs[1, 2].plot(time_series, self.object_rotations['yaw'], label='Object Yaw', color='r')
        axs[1, 2].plot(time_series, self.robot_rotations['yaw'], label='Robot Yaw', color='b')
        axs[1, 2].set_title('Rotation Yaw (relative)')
        axs[1, 2].legend()

        plt.show()
"""