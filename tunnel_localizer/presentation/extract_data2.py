import os
import cv2
import sys
import yaml
import rospy
import numpy
import rosbag
import rospkg
from termcolor import colored

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Get the path of the rosbag file
bag_path = 'panaromic02.bag';
print colored('>>> Path to rosbag file is : \'' + bag_path + '\'', 'green');

curr_path = os.path.dirname(os.path.abspath(__file__));

# Start a new directory for extracting images
images_path = curr_path + '/images';
if not os.path.exists(images_path):
    os.makedirs(images_path)
print colored('>>> Images are saved at ' + images_path, 'green');

meas_odom_file = open(curr_path + '/meas_odom.txt', 'w');
ukf_odom_file  = open(curr_path + '/ukf_odom.txt' , 'w');

bag = rosbag.Bag(bag_path)

# Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
bridge = CvBridge()

idx = 0;
# iterate over the messages
for topic, msg, t in bag.read_messages(topics=['/tunnel_localizer/debug/panaromic_image']):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8");
  except CvBridgeError, e:
    print e
  # generate file name
  image_name = images_path + '/panaromic_%05d.png' % idx;
  cv2.imwrite(image_name, cv_image);
  # print progress bar to feel better :)
  sys.stdout.write('.');
  sys.stdout.flush();

  idx = idx + 1;

bag.close()
