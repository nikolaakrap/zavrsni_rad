from sksurgerynditracker.nditracker import NDITracker
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class KameraTF2(Node):

    def __init__(self):
        super().__init__('kamera_tf2')
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        settings_vega = {
            "tracker type": "vega",
            "ip address": "192.168.1.5",
            "port": 8765,
            "romfiles": [
                "/home/nikolaakrap/NDI/Tool Definition Files/8700338.rom",
                "/home/nikolaakrap/NDI/Tool Definition Files/8700340.rom",
                "/home/nikolaakrap/NDI/Tool Definition Files/FEM0001.rom"
            ]
        }
        self.tracker = NDITracker(settings_vega)
        self.tracker.start_tracking()
        self.get_logger().info("Tracker uspjesno pokrenut")
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
            try:
                matrica = self.tracker.get_frame()
                if matrica:
                    Mat_stat = matrica[3][0]  # staticni marker
                    Mat_sty = matrica[3][1]  # stylus
                    Mat_kost = matrica[3][2] # kost
    
                    robot_marker = np.array([[0, 1, 0, 0], 
                                             [1, 0, 0, -19],
                                             [0, 0, -1, -154.717],
                                             [0, 0, 0, 1]])

                    kost_rot = np.array([[0, 0, -1, 0], 
                                         [-1, 0, 0, 0],
                                         [0, 1, 0, 0],
                                         [0, 0, 0, 1]])               

                    for x in np.nditer(Mat_kost):
                        # ako kamera ne vidi stylus, njegov frame poklapa s tool frameom robota zbog čega se robot zaustavlja
                        if np.isnan(x):
                            t1 = TransformStamped()
                            t1.header.stamp = self.get_clock().now().to_msg()
                            t1.header.frame_id = 'tool0'
                            t1.child_frame_id = 'stylus'
                            t1.transform.rotation.w = 1.0
                            t1.transform.rotation.x = 0.0
                            t1.transform.rotation.y = 0.0
                            t1.transform.rotation.z = 0.0
                            t1.transform.translation.x = 0.0
                            t1.transform.translation.y = 0.0 
                            t1.transform.translation.z = 0.0 
                            self.br.sendTransform(t1)
                        else:
                            # racunanje pozicije stylusa u frameu staticnog markera
                            sty_stat = np.dot(np.linalg.inv(Mat_stat), Mat_sty)
                            sty_stat = np.matmul(sty_stat, robot_marker)
                            sty_stat_quat = R.from_matrix(sty_stat[:3, :3]).as_quat()
                            sty_stat_trans = [sty_stat[0][3], sty_stat[1][3], sty_stat[2][3]]

                            # prikaz stylusa u rviz-u
                            t1 = TransformStamped()
                            t1.header.stamp = self.get_clock().now().to_msg()
                            t1.header.frame_id = 'base_link_marker'
                            t1.child_frame_id = 'stylus'
                            t1.transform.rotation.w = sty_stat_quat[3]
                            t1.transform.rotation.x = sty_stat_quat[0]
                            t1.transform.rotation.y = sty_stat_quat[1]
                            t1.transform.rotation.z = sty_stat_quat[2]
                            t1.transform.translation.x = sty_stat_trans[0] / 1000 # iz mm u m
                            t1.transform.translation.y = sty_stat_trans[1] / 1000 # iz mm u m
                            t1.transform.translation.z = sty_stat_trans[2] / 1000 # iz mm u m
                            self.br.sendTransform(t1)
                 

                            # racunanje pozicije kosti u frameu staticnog markera
                            kost_stat = np.dot(np.linalg.inv(Mat_stat), Mat_kost)
                            kost_stat = np.matmul(kost_stat, kost_rot)
                            kost_stat_quat = R.from_matrix(kost_stat[:3, :3]).as_quat()
                            kost_stat_trans = [kost_stat[0][3], kost_stat[1][3], kost_stat[2][3]]

                            # prikaz kosti u rviz-u
                            t2 = TransformStamped()
                            t2.header.stamp = self.get_clock().now().to_msg()
                            t2.header.frame_id = 'base_link_marker'
                            t2.child_frame_id = 'kost'
                            t2.transform.rotation.w = kost_stat_quat[3]
                            t2.transform.rotation.x = kost_stat_quat[0]
                            t2.transform.rotation.y = kost_stat_quat[1]
                            t2.transform.rotation.z = kost_stat_quat[2]
                            t2.transform.translation.x = kost_stat_trans[0] / 1000 # iz mm u m
                            t2.transform.translation.y = kost_stat_trans[1] / 1000 # iz mm u m
                            t2.transform.translation.z = kost_stat_trans[2] / 1000 # iz mm u m
                            self.br.sendTransform(t2)

                else:
                    self.get_logger().warn("nan")

            except Exception as e:
                self.get_logger().error(f"Nije uhvacen frame: {e}")

    def destroy_node(self):
        self.tracker.stop_tracking()
        self.tracker.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KameraTF2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

