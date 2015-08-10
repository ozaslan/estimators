import os
import cv
import rospy
import numpy
import rosbag

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Get the path of the rosbag file
bag_path = os.environ['ROS_DATASET_PATH'] + '/glen_canyon_22July2015_after_lunch/06.bag';
print('Path to rosbag file is : \'' + bag_path + '\'');

curr_path = os.path.dirname(os.path.abspath(__file__));

# Start a new directory for extracting images
images_path = curr_path + '/images';
if not os.path.exists(images_path):
    os.makedirs(images_path)
print('Images are saved at ' + images_path);

meas_odom_file = open(curr_path + '/meas_odom.txt', 'w');
ukf_odom_file  = open(curr_path + '/ukf_odom.txt' , 'w');

bag = rosbag.Bag(bag_path)

for topic, msg, t in bag.read_messages(topics=['/control_odom']):
  pos  = msg.pose.pose.position;
  quat = msg.pose.pose.orientation;
  array =   array = str([t.to_sec(), pos.x, pos.y, pos.z, quat.w, quat.x, quat.y, quat.z]).strip('[]');
  ukf_odom_file.write(array + '\n');

for topic, msg, t in bag.read_messages(topics=['/tunnel_localizer/odom_out']):
  pos  = msg.pose.pose.position;
  quat = msg.pose.pose.orientation;
  array =   array = str([t.to_sec(), pos.x, pos.y, pos.z, quat.w, quat.x, quat.y, quat.z]).strip('[]');
  meas_odom_file.write(array + '\n');

# Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
bridge = CvBridge()

idx = 0;
for topic, msg, t in bag.read_messages(topics=['/cam0/image']):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8");
  except CvBridgeError, e:
    print e
  image_name = images_path + '/cam0_%05d.png' % idx;
  cv.SaveImage(image_name, cv.fromarray(numpy.array(cv_image)));
  idx = idx + 1;

idx = 0;
for topic, msg, t in bag.read_messages(topics=['/cam1/image']):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8");
  except CvBridgeError, e:
    print e
  image_name = images_path + '/cam1_%05d.png' % idx;
  cv.SaveImage(image_name, cv.fromarray(numpy.array(cv_image)));
  idx = idx + 1;

idx = 0;
for topic, msg, t in bag.read_messages(topics=['/cam2/image']):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8");
  except CvBridgeError, e:
    print e
  image_name = images_path + '/cam2_%05d.png' % idx;
  cv.SaveImage(image_name, cv.fromarray(numpy.array(cv_image)));
  idx = idx + 1;

idx = 0;
for topic, msg, t in bag.read_messages(topics=['/cam3/image']):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8");
  except CvBridgeError, e:
    print e
  image_name = images_path + '/cam3_%05d.png' % idx;
  cv.SaveImage(image_name, cv.fromarray(numpy.array(cv_image)));
  idx = idx + 1;

bag.close()
