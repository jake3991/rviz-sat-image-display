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

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from interactive_markers import InteractiveMarkerServer

from scipy.spatial.transform import Rotation as R

import gtsam

from utils import load_config, write_config


class SatImageCloudNode(Node):
    def __init__(self):
        super().__init__('sat_image_cloud')

        # -----------------------------
        # Publisher
        # -----------------------------
        self.pub = self.create_publisher(PointCloud2, 'sat_image_cloud', 10)

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # -----------------------------
        # Config
        # -----------------------------
        self.config = load_config("config/config.yaml")
        self.transform_config = load_config("config/transform.yaml")

        # Image
        self.sat_img = cv2.imread("images/" + self.config.get("image"))
        cv2.imshow("test", np.zeros((100, 100)))

        # -----------------------------
        # Initial state
        # -----------------------------
        self.lat = self.config.get('lattitude')
        self.long = self.config.get('longitude')
        self.zoom = self.config.get('zoom')

        self.image_size = self.sat_img.shape[0]

        self.yaw_angle = float(self.transform_config.get('yaw'))
        self.delta_x = float(self.transform_config.get('x'))
        self.delta_y = float(self.transform_config.get('y'))
        self.delta_z = float(self.transform_config.get('z'))
        self.flip = float(self.transform_config.get('flip'))

        # scale
        self.meters_per_pixel = (1 / 2) * (
            156543.03392 * np.cos(self.lat * np.pi / 180.0) / (2 ** self.zoom)
        )

        # Precompute cloud once
        self.pc2 = self.generate_pointcloud()

        # -----------------------------
        # Interactive Marker Server
        # -----------------------------
        self.server = InteractiveMarkerServer(self, "sat_marker")
        self.create_interactive_marker()
        self.server.applyChanges()

        # Timer loop
        self.timer = self.create_timer(0.03, self.loop)

    # =========================================================
    # Point Cloud Generation
    # =========================================================
    def generate_pointcloud(self):
        points = []

        for i in range(self.sat_img.shape[0]):
            for j in range(self.sat_img.shape[1]):
                r, g, b = self.sat_img[i][j]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

                points.append([
                    (i - self.sat_img.shape[0] / 2) * self.meters_per_pixel,
                    (j - self.sat_img.shape[1] / 2) * self.meters_per_pixel,
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

    # =========================================================
    # Interactive Marker
    # =========================================================
    def create_interactive_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = "map"
        im.name = "sat_frame_control"
        im.description = "Satellite Frame"
        im.scale = 30.0

        im.pose.position.x = self.delta_x
        im.pose.position.y = self.delta_y
        im.pose.position.z = self.delta_z

        q = R.from_euler(
            'xyz',
            [np.radians(self.flip), 0.0, np.radians(self.yaw_angle)]
        ).as_quat()

        im.pose.orientation.x = q[0]
        im.pose.orientation.y = q[1]
        im.pose.orientation.z = q[2]
        im.pose.orientation.w = q[3]

        # =====================================================
        # 1. XY PLANE MOVE (coarse positioning)
        # =====================================================
        move_xy = InteractiveMarkerControl()
        move_xy.name = "move_xy"
        move_xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_xy.always_visible = True

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 2.0
        box.scale.y = 2.0
        box.scale.z = 0.1
        box.color.r = 0.2
        box.color.g = 0.6
        box.color.b = 1.0
        box.color.a = 0.5

        move_xy.markers.append(box)
        im.controls.append(move_xy)

        # =====================================================
        # 2. EXTRA DIRECTION (X-axis only fine adjustment)
        # =====================================================
        move_x = InteractiveMarkerControl()
        move_x.name = "move_x"
        move_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # X axis direction (IMPORTANT: orientation defines axis)
        move_x.orientation.w = 1.0
        move_x.orientation.x = 1.0
        move_x.orientation.y = 0.0
        move_x.orientation.z = 0.0

        im.controls.append(move_x)

        # =====================================================
        # 3. Z MOVE (keep if you still want height control)
        # =====================================================
        move_z = InteractiveMarkerControl()
        move_z.name = "move_z"
        move_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_z.orientation.w = 1.0
        move_z.orientation.x = 0.0
        move_z.orientation.y = 0.0
        move_z.orientation.z = 1.0

        im.controls.append(move_z)

        # =====================================================
        # 4. YAW ROTATION
        # =====================================================
        rotate_y = InteractiveMarkerControl()
        rotate_y.name = "rotate_y"
        rotate_y.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        # Y axis (IMPORTANT CHANGE)
        rotate_y.orientation.w = 1.0
        rotate_y.orientation.x = 0.0
        rotate_y.orientation.y = 1.0
        rotate_y.orientation.z = 0.0

        im.controls.append(rotate_y)

        # Register
        self.server.insert(im)
        self.server.setCallback(im.name, self.process_feedback)

    # =========================================================
    # Marker Callback
    # =========================================================
    def process_feedback(self, feedback):
        p = feedback.pose.position
        q = feedback.pose.orientation

        self.delta_x = p.x
        self.delta_y = p.y
        self.delta_z = p.z

        r = R.from_quat([q.x, q.y, q.z, q.w])
        euler = r.as_euler('xyz')

        self.flip = np.degrees(euler[0])
        self.yaw_angle = np.degrees(euler[2])

    # =========================================================
    # Main loop
    # =========================================================
    def loop(self):
        cv2.waitKey(1)

        # Pose from state
        pose = gtsam.Pose2(
            self.delta_x,
            self.delta_y,
            np.radians(self.yaw_angle)
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "sat_frame"

        t.transform.translation.x = self.delta_x
        t.transform.translation.y = self.delta_y
        t.transform.translation.z = self.delta_z

        q = R.from_euler(
            'xyz',
            [np.radians(self.flip), 0.0, pose.theta()]
        ).as_quat()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

        # Publish cloud
        self.pc2.header.stamp = t.header.stamp
        self.pub.publish(self.pc2)


# =========================================================
# Main
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = SatImageCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()