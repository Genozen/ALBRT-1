# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type

######### FADNet is conflicted with cv2 omg
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

########## put FADNet in src. And add it to sys.path
import sys
import os
ws_path = os.path.abspath(os.getcwd())
sys.path.append(ws_path+"/src/cv_basics/cv_basics/FADNet")
print(sys.path)
from detecter import detector
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription_left = self.create_subscription(
      Image, 
      'tara_img_left', 
      self.listener_callback_left, 
      10)

    self.subscription_right = self.create_subscription(
      Image, 
      'tara_img_right', 
      self.listener_callback_right, 
      10)

    self.subscription_left # prevent unused variable warning
    self.subscription_right # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    timer_period = 0.01
    self.timer = self.create_timer(timer_period, self.FADNet_timer_callback)

    self.current_left_frame = []
    self.current_right_frame = []


    
    # Hardcode parse config of shell script    
    class Opt():
      def __init__(self, net, dataset, model, outf, filelist, filepath):
        self.net = net
        self.dataset = dataset
        self.model = model
        self.outf = outf
        self.filelist = filelist
        self.filepath = filepath
        self.rp = './result'
        self.devices = '0'
        self.batch_size =  1


    self.opt = Opt(net = 'fadnet',
                  dataset = 'sceneflow',
                  model = ws_path + "/src/cv_basics/cv_basics/FADNet/models/fadnet.pth",
                  outf = 'detect_results/tara_test',
                  filelist = 'lists/monkaa_release.list',
                  filepath = 'data')

    self.detector = detector(self.opt)
   
  def listener_callback_left(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame left')
 
    # Convert ROS Image message to OpenCV image
    self.current_left_frame = self.br.imgmsg_to_cv2(data)


  def listener_callback_right(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame right')
 
    # Convert ROS Image message to OpenCV image
    self.current_right_frame = self.br.imgmsg_to_cv2(data)
  

  def FADNet_timer_callback(self):
    
    if((self.current_left_frame == []) or (self.current_right_frame == [])):
      return
    # Hardcode parse config of shell script




    # print("opt: ", self.opt)
    # detect(self.opt, self.current_left_frame, self.current_right_frame)

    # Display image
    cv2.imshow("camera_left", self.current_left_frame)
    cv2.imshow("camera_right", self.current_right_frame)
    cv2.waitKey(1)
    run_one_time = False
    if(not run_one_time):
      self.detector.run_detect(self.current_left_frame, self.current_right_frame)
      # run_one_time = True
      self.get_logger().info('FADNet predicting')

    # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    # disparity = stereo.compute(self.current_left_frame, self.current_right_frame)
    # norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    # norm = norm.astype(np.uint8)
    # print(normG)
    # cv2.imshow('disp', norm)
    """
    Perform FADNET Here:
    get current left and right img, maybe store them in a buffer?d
    """


  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()