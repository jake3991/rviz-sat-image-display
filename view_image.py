import cv2
import numpy as np
import struct
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import rospy
import tf
import gtsam

from utils import load_config, write_config

#setup the ROS node and publisher 
rospy.init_node("sat_image_cloud")
pub = rospy.Publisher("sat_image_cloud", PointCloud2, queue_size=2)

#read the config
config = load_config("config/config.yaml")
transform_config = load_config("config/transform.yaml")

#read in the sat image
sat_img = cv2.imread("images/"+config.get("image"))

#show a blank image, this is a trick to read the keyboard
cv2.imshow("test", np.zeros((100,100)))

#push the config into some varibles
lat = config.get('lattitude')
long = config.get('longitude')
zoom = config.get('zoom')
image_size = sat_img.shape[0]
assert(sat_img.shape[0] == sat_img.shape[1])
yaw_angle = transform_config.get('yaw')
delta_x = transform_config.get('x')
delta_y = transform_config.get('y')
delta_z = transform_config.get('z')
flip = transform_config.get('flip')

#get the image scale 
meters_per_pixel = (1/2)*(156543.03392 * np.cos(lat * np.pi / 180.) / (2**zoom) )

#containers for the point cloud
points = []

#loop over the image and convert it to an RGBA point cloud
for i in range(sat_img.shape[0]):
    for j in range(sat_img.shape[1]):
        
        #parse out and convert the RGB values to RGBA
        r, g, b = sat_img[i][j]
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
        points.append( [(i-sat_img.shape[0]/2) * meters_per_pixel, (j-sat_img.shape[0]/2) * meters_per_pixel, 0., rgb] )

#define the point cloud fields
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 12, PointField.UINT32, 1),
          ]

#define the header
header = Header()
header.frame_id = "sat_frame"
pc2 = point_cloud2.create_cloud(header, fields, points)

#define the step size for movement
step_meters = 1.0
step_degrees = 1.0

#loop to read the keyboard
while True:
    k = cv2.waitKey(1)

    #if esc
    if k == 27:
        config = [{"x":delta_x},
                    {"y":delta_y},
                    {"z":delta_z},
                    {"yaw":yaw_angle},
                    {"flip":flip}]
        write_config(config,"config/transform.yaml")
        cv2.destroyAllWindows()
        break

    #change step size
    if k == ord('o'):
        step_meters += .1
    if k == ord('l'):
        step_meters -= .1

    if k == ord('q'):
        yaw_angle -= step_degrees
    if k == ord('e'):
        yaw_angle += step_degrees

    #move down
    if k == ord('s'):  
        delta_y -= step_meters

    #move up
    if k == ord('w'):
        delta_y += step_meters

    #move left
    if k == ord('a'):
        delta_x += step_meters

    #move right
    if k == ord('d'):
        delta_x -= step_meters

    #lower
    if k == ord('f'):
        delta_z -= .1
    
    #raise
    if k == ord('r'):
        delta_z += .1

    #flip
    if k == ord('x'):
        if flip == 0:
            flip = 180
        else:
            flip = 0

    pose = gtsam.Pose2(delta_x,delta_y,np.radians(yaw_angle))
    #pose_inv = pose.inverse()
    br = tf.TransformBroadcaster()
    br.sendTransform((pose.x(),pose.y(),delta_z),
                        tf.transformations.quaternion_from_euler(np.radians(flip), 0, pose.theta()),
                        rospy.Time.now(),
                        "sat_frame",
                        "map")
    pub.publish(pc2)
