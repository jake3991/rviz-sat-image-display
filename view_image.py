import cv2
import numpy as np
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import TransformStamped
import tf2_ros

from scipy.spatial.transform import Rotation as R

import gtsam

from utils import load_config, write_config


class SatImageCloudNode(Node):
    def __init__(self):
        super().__init__('sat_image_cloud')

        # Publisher
        self.pub = self.create_publisher(PointCloud2, 'sat_image_cloud', 10)

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # Load configs
        self.config = load_config("config/config.yaml")
        self.transform_config = load_config("config/transform.yaml")

        # Load image
        self.sat_img = cv2.imread("images/" + self.config.get("image"))

        # Keyboard trick window
        cv2.imshow("test", np.zeros((100, 100)))

        # Params
        self.lat = self.config.get('lattitude')
        self.long = self.config.get('longitude')
        self.zoom = self.config.get('zoom')

        #self.image_size = self.sat_img.shape[0]
        #assert self.sat_img.shape[0] == self.sat_img.shape[1]

        self.yaw_angle = self.transform_config.get('yaw')
        self.delta_x = self.transform_config.get('x')
        self.delta_y = self.transform_config.get('y')
        self.delta_z = self.transform_config.get('z')
        self.flip = self.transform_config.get('flip')

        # Scale (meters per pixel)
        self.meters_per_pixel = (1/2) * (
            156543.03392 * np.cos(self.lat * np.pi / 180.) / (2**self.zoom)
        )

        # Generate point cloud once
        self.pc2 = self.generate_pointcloud()

        # Control steps
        self.step_meters = 1.0
        self.step_degrees = 1.0

        # Timer loop (~30 Hz)
        self.timer = self.create_timer(0.03, self.loop)

    def generate_pointcloud(self):
        points = []

        for i in range(self.sat_img.shape[0]):
            for j in range(self.sat_img.shape[1]):
                r, g, b = self.sat_img[i][j]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

                points.append([
                    (i - self.sat_img.shape[0] / 2) * self.meters_per_pixel,
                    (j - self.sat_img.shape[0] / 2) * self.meters_per_pixel,
                    0.0,
                    rgb
                ])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        header = Header()
        header.frame_id = "sat_frame"

        return point_cloud2.create_cloud(header, fields, points)

    def loop(self):
        k = cv2.waitKey(1)

        # ESC to exit and save
        if k == 27:
            config = [
                {"x": self.delta_x},
                {"y": self.delta_y},
                {"z": self.delta_z},
                {"yaw": self.yaw_angle},
                {"flip": self.flip}
            ]
            write_config(config, "config/transform.yaml")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        # Step size
        if k == ord('o'):
            self.step_meters += 0.1
        if k == ord('l'):
            self.step_meters -= 0.1

        # Rotation
        if k == ord('q'):
            self.yaw_angle -= self.step_degrees
        if k == ord('e'):
            self.yaw_angle += self.step_degrees

        # Translation
        if k == ord('s'):
            self.delta_y -= self.step_meters
        if k == ord('w'):
            self.delta_y += self.step_meters
        if k == ord('a'):
            self.delta_x += self.step_meters
        if k == ord('d'):
            self.delta_x -= self.step_meters

        # Z
        if k == ord('f'):
            self.delta_z -= 0.1
        if k == ord('r'):
            self.delta_z += 0.1

        # Flip
        if k == ord('x'):
            self.flip = 180 if self.flip == 0 else 0

        # Pose
        pose = gtsam.Pose2(
            self.delta_x,
            self.delta_y,
            np.radians(self.yaw_angle)
        )

        # TF message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "sat_frame"

        t.transform.translation.x = pose.x()
        t.transform.translation.y = pose.y()
        t.transform.translation.z = self.delta_z

        # Quaternion using scipy
        roll = np.radians(self.flip)
        pitch = 0.0
        yaw = pose.theta()

        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

        # Publish cloud
        self.pc2.header.stamp = t.header.stamp
        self.pub.publish(self.pc2)


def main(args=None):
    rclpy.init(args=args)
    node = SatImageCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()