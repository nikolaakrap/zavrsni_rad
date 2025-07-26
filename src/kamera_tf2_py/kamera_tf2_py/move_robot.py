import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2Servo
from geometry_msgs.msg import TwistStamped
import rclpy.time
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
from std_srvs.srv import Trigger
import math
import matplotlib.pyplot as plt

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')

        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.callback_group = ReentrantCallbackGroup()

        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        base_link_name = 'base_link'
        end_effector_name = 'tool0'

        self.moveit = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name='ur_manipulator',
            use_move_group_action=True
        )

        self.servo = MoveIt2Servo(
            node=self,
            frame_id='tool0',
            linear_speed=3.0,
            angular_speed=3.0,
            enable_at_init=True,
            callback_group=self.callback_group
        )

        self.timer = self.create_timer(0.1, self.update_servo, callback_group=self.callback_group)
        self.stylus_srv = self.create_service(Trigger, 'toggle_stylus_tracking', self.toggle_stylus_switch)
        self.bone_srv = self.create_service(Trigger, 'toggle_bone_tracking', self.toggle_bone_switch)
        
        self.stylus_tracking = True
        self.bone_tracking = False

        self.last_known_transform = None

        # Arrays to store movement data
        self.kost_linear_positions = []
        self.kost_angular_positions = []
        self.tool0_linear_positions = []
        self.tool0_angular_positions = []
        self.z_axis_angles = []

    def toggle_stylus_switch(self, request, response):
        self.stylus_tracking = not self.stylus_tracking

        if not self.stylus_tracking:
            self.get_logger().info("Stopped stylus tracking. Switching to kost marker.")
            self.last_known_transform = self.get_last_transform('tool0')
        else:
            self.get_logger().info("Stylus tracking enabled.")

        response.success = True
        response.message = f'Stylus tracking: {"ON" if self.stylus_tracking else "OFF"}'
        return response

    def toggle_bone_switch(self, request, response):
        self.bone_tracking = not self.bone_tracking

        if not self.bone_tracking:
            self.get_logger().info("Bone tracking disabled. Plotting movements.")
            self.plot_movements()  # Plot and save the movements when bone tracking is turned off
        else:
            self.get_logger().info("Bone tracking enabled.")

        response.success = True
        response.message = f'Bone tracking: {"ON" if self.bone_tracking else "OFF"}'
        return response

    def get_last_transform(self, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, 'kost', time=rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform for {target_frame}: {e}")
            return None

    def update_servo(self):
        linear_velocity = np.array([0.0, 0.0, 0.0])
        angular_velocity = np.array([0.0, 0.0, 0.0])

        try:
            if self.stylus_tracking:
                pass
            elif self.bone_tracking and self.last_known_transform:
                transform2 = self.tf_buffer.lookup_transform('tool0', 'kost', time=rclpy.time.Time())
                transform3 = self.tf_buffer.lookup_transform('kost', 'tool0', time=rclpy.time.Time())

                linn = np.array([
                    -transform3.transform.translation.x,
                    -transform3.transform.translation.y,
                    -transform3.transform.translation.z
                ])

                kost_linear_position = np.array([
                    transform2.transform.translation.x,
                    transform2.transform.translation.y,
                    transform2.transform.translation.z
                ])
                self.kost_linear_positions.append(kost_linear_position)

                quat = np.array([
                    transform2.transform.rotation.x,
                    transform2.transform.rotation.y,
                    transform2.transform.rotation.z,
                    transform2.transform.rotation.w
                ])

                kost_angular_position = R.from_quat(quat).as_euler('xyz')
                self.kost_angular_positions.append(kost_angular_position)

                tool0_linear_position = np.array([
                    self.last_known_transform.transform.translation.x,
                    self.last_known_transform.transform.translation.y,
                    self.last_known_transform.transform.translation.z
                ])
                

                tool0_quat = np.array([
                    self.last_known_transform.transform.rotation.x,
                    self.last_known_transform.transform.rotation.y,
                    self.last_known_transform.transform.rotation.z,
                    self.last_known_transform.transform.rotation.w
                ])


                transform_tool0 = self.tf_buffer.lookup_transform('base_link_marker', 'tool0', time=rclpy.time.Time())
                transform_kost = self.tf_buffer.lookup_transform('base_link_marker', 'kost', time=rclpy.time.Time())

                # Get the Z-axes from the transformations
                tool0_quatt = np.array([
                    transform_tool0.transform.rotation.x,
                    transform_tool0.transform.rotation.y,
                    transform_tool0.transform.rotation.z,
                    transform_tool0.transform.rotation.w
                ])
                kost_quatt = np.array([
                    transform_kost.transform.rotation.x,
                    transform_kost.transform.rotation.y,
                    transform_kost.transform.rotation.z,
                    transform_kost.transform.rotation.w
                ])

                # Convert quaternions to rotation matrices
                z_axis_tool0 = R.from_quat(tool0_quatt).as_matrix()[:, 2]  # Extract Z-axis
                z_axis_kost = R.from_quat(kost_quatt).as_matrix()[:, 2]    # Extract Z-axis

                # Compute the angle between the Z-axes using dot product
                dot_product = np.dot(z_axis_tool0, z_axis_kost)
                angle = np.arccos(dot_product)  # angle in radians

                # Store the angle for plotting
                self.z_axis_angles.append(angle)



                linear_velocity = kost_linear_position - tool0_linear_position
                angular_velocity = R.from_quat(quat) * R.from_quat(tool0_quat).inv()
                angular_velocity = angular_velocity.as_euler('xyz')
                self.tool0_linear_positions.append(linn)
                self.tool0_angular_positions.append(angular_velocity)
                linear_difference = np.linalg.norm(linear_velocity)
                angular_difference = np.linalg.norm(angular_velocity)
                
                if linear_difference > 0.00009 and angular_difference > 0.0001:
                    self.servo.servo(
                        linear=(linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                        angular=(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                        enable_if_disabled=True
                    )
                elif linear_difference < 0.00008:
                    self.servo.servo(
                        linear=(-linear_velocity[0], -linear_velocity[1], -linear_velocity[2]),
                        angular=(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                        enable_if_disabled=True
                    )
                else:
                    self.servo.servo(
                        linear=(0.0, 0.0, 0.0),
                        angular=(0.0, 0.0, 0.0),
                        enable_if_disabled=True
                    )
            else:
                self.servo.servo(
                    linear=(0.0, 0.0, 0.0),
                    angular=(0.0, 0.0, 0.0),
                    enable_if_disabled=True
                )
            
        except Exception as e:
            self.get_logger().warn(f"{e}")


    def plot_movements(self):
        kost_linear = np.array(self.kost_linear_positions) * 1000
        kost_angular = np.array(self.kost_angular_positions)
        tool0_linear = np.array(self.tool0_linear_positions) * 1000
        tool0_angular = np.array(self.tool0_angular_positions)

        kost_linear_zeroed = kost_linear - kost_linear[0]
        tool0_linear_zeroed = tool0_linear - tool0_linear[0]
        kost_angular_zeroed = kost_angular - kost_angular[0]

        linear_error = np.abs(kost_linear_zeroed - tool0_linear_zeroed)
        angular_error = np.abs(kost_angular_zeroed - tool0_angular)

        time_vector = np.arange(len(kost_linear_zeroed)) * 0.1

        # Euclidean distance between tool0 and kost over time, normalized to start from 0
        euclidean_distances = np.linalg.norm(kost_linear - tool0_linear, axis=1)
        euclidean_distances -= euclidean_distances[0]  # Subtract initial distance to normalize to zero

        # Angle between the z-axes of tool0 and kost
        z_axis_angles = []

        for i in range(len(kost_angular)):
            # Extract quaternions for tool0 and kost
            kost_quat = R.from_euler('xyz', kost_angular[i])
            tool0_quat = R.from_euler('xyz', tool0_angular[i])

            # Convert to rotation matrices
            kost_rotation_matrix = kost_quat.as_matrix()
            tool0_rotation_matrix = tool0_quat.as_matrix()

            # Extract Z-axes (the third column of the rotation matrices)
            kost_z_axis = kost_rotation_matrix[:, 2]
            tool0_z_axis = tool0_rotation_matrix[:, 2]

            # Calculate the angle between the two Z-axes using the dot product
            dot_product = np.dot(kost_z_axis, tool0_z_axis)
            angle_dot = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Clip to avoid numerical errors
            
            # Optionally: calculate the angle using the cross product as a verification
            cross_product = np.cross(kost_z_axis, tool0_z_axis)
            sin_theta = np.linalg.norm(cross_product) / (np.linalg.norm(kost_z_axis) * np.linalg.norm(tool0_z_axis))
            angle_cross = np.arcsin(np.clip(sin_theta, -1.0, 1.0))

            # Verify that both approaches give similar results
            z_axis_angles.append(angle_cross)  # You could also append angle_cross if you want to use that instead

        fig, axs = plt.subplots(2, 2, figsize=(12, 10))

        # Plot the translational movements for kost and tool0
        axs[0, 0].plot(time_vector, kost_linear_zeroed[:, 0], label="kost_x", color='blue', linestyle='--')
        axs[0, 0].plot(time_vector, kost_linear_zeroed[:, 1], label="kost_y", color='green', linestyle='--')
        axs[0, 0].plot(time_vector, kost_linear_zeroed[:, 2], label="kost_z", color='red', linestyle='--')
        axs[0, 0].plot(time_vector, tool0_linear_zeroed[:, 0], label="tcp_x", color='blue')
        axs[0, 0].plot(time_vector, tool0_linear_zeroed[:, 1], label="tcp_y", color='green')
        axs[0, 0].plot(time_vector, tool0_linear_zeroed[:, 2], label="tcp_z", color='red')
        axs[0, 0].set_title('Translacija')
        axs[0, 0].set_ylabel('Pomak [mm]')
        axs[0, 0].set_xlabel('Vrijeme [s]')
        axs[0, 0].legend()

        # Plot the rotational movements for kost and tool0
        axs[0, 1].plot(time_vector, kost_angular_zeroed[:, 0], label="kost_qx", color='blue', linestyle='--')
        axs[0, 1].plot(time_vector, kost_angular_zeroed[:, 1], label="kost_qy", color='green', linestyle='--')
        axs[0, 1].plot(time_vector, kost_angular_zeroed[:, 2], label="kost_qz", color='red', linestyle='--')
        axs[0, 1].plot(time_vector, tool0_angular[:, 0], label="tcp_qx", color='blue')
        axs[0, 1].plot(time_vector, tool0_angular[:, 1], label="tcp_qy", color='green')
        axs[0, 1].plot(time_vector, tool0_angular[:, 2], label="tcp_qz", color='red')
        axs[0, 1].set_title('Rotacija')
        axs[0, 1].set_ylabel('Zakret [rad]')
        axs[0, 1].set_xlabel('Vrijeme [s]')
        axs[0, 1].legend()

        # Plot the changing Euclidean distance (starting from zero)
        axs[1, 0].plot(time_vector, euclidean_distances, label="euklidska pogreška", color='blue')
        axs[1, 0].set_title('Euklidska pogreška slijeđenja reference')
        axs[1, 0].set_ylabel('Pogreška [mm]')
        axs[1, 0].set_xlabel('Vrijeme [s]')
        axs[1, 0].legend()

        # Plot the angle between the Z-axes of tool0 and kost
        axs[1, 1].plot(time_vector, self.z_axis_angles, label="kut", color='red')
        axs[1, 1].set_title('Prostorni kut između z-osi')
        axs[1, 1].set_ylabel('Kut [rad]')
        axs[1, 1].set_xlabel('Vrijeme [s]')
        axs[1, 1].legend()

        plt.tight_layout()
        plt.savefig('movements.svg')
        self.get_logger().info("Movement plot saved as movements.svg")







    def start(self):
        self.get_logger().info("Starting robot controller")
        self.servo.enable()

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    node.start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
