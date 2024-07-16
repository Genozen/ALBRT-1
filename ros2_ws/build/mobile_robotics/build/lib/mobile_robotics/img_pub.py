# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_left = self.create_publisher(Image, 'mg_left', 10)
    self.publisher_right = self.create_publisher(Image, 'img_right', 10)
      
    # We will publish a message every 0.1 seconds
    timer_period = 0.0001  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
    self.cap2 = cv2.VideoCapture(2)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
    ret2, frame2 = self.cap2.read()
    if (ret == True): #& (ret2 == True):
      # frame = np.array(frame) 
      # frame2 = np.array(frame2)
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      # self.publisher_left.publish(self.br.cv2_to_imgmsg(np.array(frame)))
      # self.publisher_right.publish(self.br.cv2_to_imgmsg(np.array(frame2)))
      scale_percent = 10
      width = int(frame.shape[1] * scale_percent / 100)
      height = int(frame.shape[0] * scale_percent / 100)
      dim = (width, height)

      resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
      resized2 = cv2.resize(frame2, dim, interpolation = cv2.INTER_AREA)
      cv2.imshow('temp1', resized)
      cv2.imshow('temp2', resized2)
      cv2.waitKey(1)
      # Display the message on the console
      self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()