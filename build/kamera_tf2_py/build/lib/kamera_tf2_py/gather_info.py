import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import csv
import matplotlib.pyplot as plt

# Function to calculate Euclidean distance between two points (for error calculation)
def calculate_error(position, initial_position):
    error = math.sqrt(
        (position[0] - initial_position[0])**2 +
        (position[1] - initial_position[1])**2 +
        (position[2] - initial_position[2])**2
    )
    return error

# TF Listener Node for ROS 2
class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.first_position = None
        self.first_orientation = None

        self.log_file = open('transform_error_data.csv', mode='w')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['time', 'x', 'y', 'z', 'max_error', 'qx', 'qy', 'qz', 'qw'])

        # Set up a timer to repeatedly check for transforms
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        try:
            # Lookup the transform between 'target_frame' and 'source_frame'
            trans = self.tf_buffer.lookup_transform('kost', 'tool0', rclpy.time.Time())

            # Get the translation (position)
            translation = trans.transform.translation
            position = (translation.x, translation.y, translation.z)

            # Get the rotation (orientation as a quaternion)
            rotation = trans.transform.rotation
            orientation = (rotation.x, rotation.y, rotation.z, rotation.w)

            # Store the first position and use it as the reference
            if self.first_position is None:
                self.first_position = position
                self.first_orientation = orientation

            # Calculate error relative to the first position
            error = calculate_error(position, self.first_position)

            # Log the data (timestamp, translation, error, rotation)
            timestamp = self.get_clock().now().to_msg().sec
            self.csv_writer.writerow([timestamp, *position, error, *orientation])

            self.get_logger().info(f"Position: {position}, Error: {error}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Transform not available")

    def close_log_file(self):
        self.log_file.close()

# Function to plot the position error over time and save it as a PNG file
def plot_error():
    times = []
    errors = []

    # Read the data from the CSV file
    with open('transform_error_data.csv', mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            times.append(float(row['time']))
            errors.append(float(row['error']))

    # Plotting the error over time
    plt.figure()
    plt.plot(times, errors, label='Position Error')

    # Add labels and title
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.title('Position Error Relative to First Position Over Time')

    # Save the plot as a PNG file
    plt.savefig('position_error.png')
    plt.close()  # Close the plot window after saving

# Main function to run the listener and plot the data
def main(args=None):
    rclpy.init(args=args)

    # Create the TF listener node
    tf_listener_node = TFListenerNode()

    try:
        # Spin the node until it's shut down
        rclpy.spin(tf_listener_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close log file and shut down ROS
        tf_listener_node.close_log_file()
        tf_listener_node.destroy_node()
        rclpy.shutdown()

        # After gathering data, plot the error and trajectory
        plot_error()


if __name__ == '__main__':
    main()
