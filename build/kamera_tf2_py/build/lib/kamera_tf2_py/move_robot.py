import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2Servo
from geometry_msgs.msg import TwistStamped
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
from std_srvs.srv import Trigger

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
            linear_speed=0.4,
            angular_speed=0.8,
            enable_at_init=True,
            callback_group=self.callback_group
        )

        self.timer = self.create_timer(0.1, self.update_servo, callback_group=self.callback_group)
        self.stylus_srv = self.create_service(Trigger, 'toggle_stylus_tracking', self.toggle_stylus_switch)
        self.bone_srv = self.create_service(Trigger, 'toggle_bone_tracking', self.toggle_bone_switch)
        
        self.stylus_tracking = True
        self.bone_tracking = False

        self.last_known_transform = None

    def toggle_stylus_switch(self, request, response):
        self.stylus_tracking = not self.stylus_tracking

        if not self.stylus_tracking:
            self.get_logger().info("Stopped stylus tracking. Switching to kost marker.")
            self.last_known_transform = self.get_last_transform('stylus')
        else:
            self.get_logger().info("Stylus tracking enabled.")

        response.success = True
        response.message = f'Stylus tracking: {"ON" if self.stylus_tracking else "OFF"}'
        return response

    def toggle_bone_switch(self, request, response):
        self.bone_tracking = not self.bone_tracking


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
                transform = self.tf_buffer.lookup_transform('tool0', 'stylus', time=rclpy.time.Time())

                linear_velocity = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])

                quat = np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])

                r = R.from_quat(quat)
                angular_velocity = r.as_euler('xyz')

                self.get_logger().info(f"Stylus linear transform: {linear_velocity}")
                self.get_logger().info(f"Stylus angular transform: {angular_velocity}")

                self.stylus_robot_distance = np.linalg.norm(linear_velocity)
                self.get_logger().info(f"Distance to stylus marker: {self.stylus_robot_distance} meters")

                if self.stylus_robot_distance > 0.1:
                    self.servo.servo(
                        linear=(linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                        angular=(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                        enable_if_disabled=True
                    )
                else:
                    self.servo.servo(
                        linear=(0.0, 0.0, 0.0),
                        angular=(0.0, 0.0, 0.0),
                        enable_if_disabled=True
                    )
            elif self.bone_tracking and self.last_known_transform:
                transform2 = self.tf_buffer.lookup_transform('tool0', 'kost', time=rclpy.time.Time())

                linear_velocity = np.array([
                    transform2.transform.translation.x - self.last_known_transform.transform.translation.x,
                    transform2.transform.translation.y - self.last_known_transform.transform.translation.y,
                    transform2.transform.translation.z - self.last_known_transform.transform.translation.z
                ])

                quat = np.array([
                    transform2.transform.rotation.x * self.last_known_transform.transform.rotation.x,
                    transform2.transform.rotation.y * self.last_known_transform.transform.rotation.y,
                    transform2.transform.rotation.z * self.last_known_transform.transform.rotation.z,
                    transform2.transform.rotation.w * self.last_known_transform.transform.rotation.w
                ])

                r = R.from_quat(quat)
                angular_velocity = r.as_euler('xyz')

                # Reverse the movement direction for "kost"
                #linear_velocity = -linear_velocity

                self.get_logger().info(f"Kost linear transform: {linear_velocity}")
                self.get_logger().info(f"Kost angular transform: {angular_velocity}")

                kost_robot_distance = np.linalg.norm(linear_velocity)
                self.get_logger().info(f"Distance to kost marker: {kost_robot_distance} meters")

                if kost_robot_distance > self.stylus_robot_distance - 0.1:
                    self.servo.servo(
                        linear=(linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                        #angular=(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                        enable_if_disabled=True
                    )

                elif kost_robot_distance < self.stylus_robot_distance - 0.005:
                    self.servo.servo(
                        linear=(-linear_velocity[0], -linear_velocity[1], -linear_velocity[2]),
                        #angular=(-angular_velocity[0], -angular_velocity[1], -angular_velocity[2]),
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



"""
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2Servo
from geometry_msgs.msg import TwistStamped
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
from std_srvs.srv import Trigger

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
            linear_speed=0.4,
            angular_speed=0.8,
            enable_at_init=True,
            callback_group=self.callback_group
        )

        self.timer = self.create_timer(0.1, self.update_servo, callback_group=self.callback_group)
        self.stylus_srv = self.create_service(Trigger, 'toggle_stylus_tracking', self.toggle_stylus_switch)
        self.bone_srv = self.create_service(Trigger, 'toggle_bone_tracking', self.toggle_bone_switch)
        
        self.stylus_tracking = True
        self.bone_tracking = False

        self.last_known_transform = None

    def toggle_stylus_switch(self, request, response):
        self.stylus_tracking = not self.stylus_tracking

        if not self.stylus_tracking:
            self.get_logger().info("Stopped stylus tracking. Switching to kost marker.")
            self.last_known_transform = self.get_last_transform('stylus')
        else:
            self.get_logger().info("Stylus tracking enabled.")

        response.success = True
        response.message = f'Stylus tracking: {"ON" if self.stylus_tracking else "OFF"}'
        return response

    def toggle_bone_switch(self, request, response):
        self.bone_tracking = not self.bone_tracking


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
                transform = self.tf_buffer.lookup_transform('tool0', 'stylus', time=rclpy.time.Time())

                linear_velocity = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])

                quat = np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])

                r = R.from_quat(quat)
                angular_velocity = r.as_euler('xyz')

                self.get_logger().info(f"Stylus linear transform: {linear_velocity}")
                self.get_logger().info(f"Stylus angular transform: {angular_velocity}")

                self.stylus_robot_distance = np.linalg.norm(linear_velocity)
                self.get_logger().info(f"Distance to stylus marker: {self.stylus_robot_distance} meters")

                if self.stylus_robot_distance > 0.1:
                    self.servo.servo(
                        linear=(linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                        angular=(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                        enable_if_disabled=True
                    )
                else:
                    self.servo.servo(
                        linear=(0.0, 0.0, 0.0),
                        angular=(0.0, 0.0, 0.0),
                        enable_if_disabled=True
                    )
            elif self.bone_tracking and self.last_known_transform:
                transform2 = self.tf_buffer.lookup_transform('tool0', 'kost', time=rclpy.time.Time())

                linear_velocity = np.array([
                    transform2.transform.translation.x - self.last_known_transform.transform.translation.x,
                    transform2.transform.translation.y - self.last_known_transform.transform.translation.y,
                    transform2.transform.translation.z - self.last_known_transform.transform.translation.z
                ])

                quat = np.array([
                    transform2.transform.rotation.x * self.last_known_transform.transform.rotation.x,
                    transform2.transform.rotation.y * self.last_known_transform.transform.rotation.y,
                    transform2.transform.rotation.z * self.last_known_transform.transform.rotation.z,
                    transform2.transform.rotation.w * self.last_known_transform.transform.rotation.w
                ])

                r = R.from_quat(quat)
                angular_velocity = r.as_euler('xyz')

                # Reverse the movement direction for "kost"
                #linear_velocity = -linear_velocity

                self.get_logger().info(f"Kost linear transform: {linear_velocity}")
                self.get_logger().info(f"Kost angular transform: {angular_velocity}")

                kost_robot_distance = np.linalg.norm(linear_velocity)
                self.get_logger().info(f"Distance to kost marker: {kost_robot_distance} meters")

                if kost_robot_distance > self.stylus_robot_distance - 0.1:
                    self.servo.servo(
                        linear=(linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                        #angular=(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                        enable_if_disabled=True
                    )

                elif kost_robot_distance < self.stylus_robot_distance - 0.005:
                    self.servo.servo(
                        linear=(-linear_velocity[0], -linear_velocity[1], -linear_velocity[2]),
                        #angular=(-angular_velocity[0], -angular_velocity[1], -angular_velocity[2]),
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
"""