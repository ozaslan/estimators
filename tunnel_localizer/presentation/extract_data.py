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
bag_path = os.environ['ROS_DATASET_PATH'] + '/glen_canyon_22July2015_after_lunch/06.bag';
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

cams = ['cam0', 'cam1', 'cam2', 'cam3'];
calib_files = ['ins_khex_right_cam.yaml',
                'ins_khex_top_cam.yaml',
                'ins_khex_bottom_cam.yaml',
                'ins_khex_left_cam.yaml'];



# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
# rospack.list_pkgs() 

padding = 400;

for cam in cams:
  print colored('>>> Unpacking images from topic : /' + cam + '/image', 'green');

  # load the camera calibration data
  calib_data_path = rospack.get_path('calib_data') + '/camera/' + calib_files[int(cam[-1])];
  print colored('>>> Camera calibration data path : ' + calib_data_path, 'green');

  with open(calib_data_path, 'r') as stream:
    calib_file = yaml.load(stream)
  
  dist_coeffs = calib_file['distortion_coefficients']['data'];
  cam_matrix  = calib_file['camera_matrix']['data'];
  
  cam_matrix  = numpy.reshape(numpy.array(cam_matrix), (3, 3))
  cam_matrix[0:2, 2] = cam_matrix[0:2, 2] + padding;
  dist_coeffs = numpy.array(dist_coeffs)

  print colored('>>> Distortion coeff : \n', 'green'), colored(dist_coeffs, 'yellow')
  print colored('>>> Camera matrix (after padding) : \n', 'green'), colored(cam_matrix , 'yellow')

  idx = 0;
  # iterate over the messages
  for topic, msg, t in bag.read_messages(topics=['/' + cam + '/image']):
    try:
      cv_image = bridge.imgmsg_to_cv2(msg, "bgr8");
    except CvBridgeError, e:
      print e
    # generate file name
    image_name = images_path + '/' + cam + '_%05d.png' % idx;
    # add border before undistort
    cv_image = cv2.copyMakeBorder(cv_image, padding, padding, padding, padding, cv2.BORDER_CONSTANT, 0);
    # undistort
    cv_image = cv2.undistort(cv_image, cam_matrix, dist_coeffs)
    # write the image to file
    cv2.imwrite(image_name, cv_image);
    # print progress bar to feel better :)
    sys.stdout.write('.');
    sys.stdout.flush();

    idx = idx + 1;
  print 

bag.close()
