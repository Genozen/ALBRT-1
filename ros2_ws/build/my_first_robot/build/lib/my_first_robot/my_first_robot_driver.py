"""
Auth: NH

dev notes:
Issue - RuntimeError: No access to /dev/mem.  Try running as root!
Solution - sudo usermod -a -G gpio $USER

Issue - Can't connect to pigpio at localhost(8888) Did you start the pigpio daemon? E.g. sudo pigpiod
Solution - sudo pigpiod


"""

# GPIO and Motors
import pigpio
from time import sleep
import curses

# ROS2 stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist


class JoyListender(Node):
    def __init__(self):
        super().__init__('joystick_listener')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialization of pigpio and the motors
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Cannot connect to pigpiod daemon. Ensure it's running with 'sudo pigpiod'")
        
        # Define the GPIO pins for the motors
        self.frmotor_forward = 6
        self.frmotor_backward = 5
        self.flmotor_forward = 16
        self.flmotor_backward = 12
        self.brmotor_forward = 26
        self.brmotor_backward = 19
        self.blmotor_forward = 21
        self.blmotor_backward = 20
        
        self.flag_autonomous = False

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z


        # if angular_z > 0:
        #     # if command wants left and foward
        #     if linear_x > 0:
        #         self.pi.write(self.frmotor_forward, 1)
        #         self.pi.write(self.brmotor_forward, 1)
        #         # sleep(0.1)
        #         # self.stop()
        #     # if command wants left backward
        #     elif linear_x < 0:
        #         self.pi.write(self.flmotor_backward, 1)
        #         self.pi.write(self.blmotor_backward, 1)
        #         # sleep(0.1)
        #         # self.stop()     
        #     else:
        #         self.rotate_left()
        #         # sleep(0.1)
        #         # self.stop()
        # elif angular_z < 0:
        #     # if command wants right and foward
        #     if linear_x > 0:
        #         self.pi.write(self.flmotor_forward, 1)
        #         self.pi.write(self.blmotor_forward, 1)
        #         # sleep(0.1)
        #         # self.stop()
        #     # if command wants right backward
        #     elif linear_x < 0:
        #         self.pi.write(self.frmotor_backward, 1)
        #         self.pi.write(self.brmotor_backward, 1)
        #         # sleep(0.1)
        #         # self.stop()     
        #     else:
        #         self.rotate_right()
        #         # sleep(0.1)
        #         # self.stop()
        # elif linear_x > 0:
        #     self.forward()
        #     # sleep(0.1)
        #     # self.stop()
        # elif linear_x < 0:
        #     self.reverse()
        #     # sleep(0.1)
        #     # self.stop()
        # else:
        #     self.stop()
            
        # sleep(0.025)
        # self.stop()
            

        # Override to use Nav2 Goal Pose cmd_vel
        if (angular_z != 0) | (linear_x != 0):
            self.flag_autonomous = True
        
        
        # if angular_z > 0:
        #     # if command wants left and foward
        #     if abs(angular_z) > 0.8:
        #         print("Rotate Left")
        #         self.rotate_left()
            
        #     elif linear_x > 0:
        #         self.pi.write(self.frmotor_forward, 1)
        #         self.pi.write(self.brmotor_forward, 1)
        #         print("Foward Left")
        #         # sleep(0.1)
        #         # self.stop()
        #     # if command wants left backward
        #     elif linear_x < 0:
        #         self.pi.write(self.flmotor_backward, 1)
        #         self.pi.write(self.blmotor_backward, 1)
        #         print("Back Left")
        #         # sleep(0.1)
        #         # self.stop()     
        #     # else:
        #         # self.rotate_left()
        #         # sleep(0.03)
        #         # self.stop()
        # elif angular_z < 0:
        #     # if command wants right and foward
        #     if abs(angular_z) > 0.8:
        #         print("Rotate Right")
        #         self.rotate_right()
            
        #     elif linear_x > 0:
        #         self.pi.write(self.flmotor_forward, 1)
        #         self.pi.write(self.blmotor_forward, 1)
        #         print("Foward Right")
        #         # sleep(0.1)
        #         # self.stop()
        #     # if command wants right backward
        #     elif linear_x < 0:
        #         self.pi.write(self.frmotor_backward, 1)
        #         self.pi.write(self.brmotor_backward, 1)
        #         print("Back Right")
        #         # sleep(0.1)
        #         # self.stop()     
        #     # else:
        #         # self.rotate_right()
        #         # sleep(0.03)
        #         # self.stop()
        # elif linear_x > 0:
        #     self.forward()
        #     # sleep(0.1)
        #     # self.stop()
        # elif linear_x < 0:
        #     self.reverse()
        #     # sleep(0.1)
        #     # self.stop()
        # # else:
        #     # self.stop()
        #     # self.flag_autonomous = False
        # sleep(0.025)
        # self.stop()
            
        # # prioritize turning over linear movement
        if angular_z > 0:
            self.rotate_left()
            sleep(0.03)
            self.stop()
        if angular_z < 0:
            self.rotate_right()
            sleep(0.03)
            self.stop()
        if linear_x > 0:
            self.forward()
            sleep(0.05)
            self.stop()
        if linear_x < 0:
            self.reverse()
            sleep(0.05)
            self.stop()
        if linear_x == 0 and angular_z == 0:
            self.stop()
            
        # sleep(0.02)
        # self.stop()

        print(f"linear_x: {linear_x} | angular_z: {angular_z}")

    def joy_callback(self, msg):
        
        return # Early termination to NOT RUN CODE BELOW
        
        
        # pass
        x_cmd = msg.axes[0]
        y_cmd = msg.axes[1]
        r_cmd = msg.axes[3]
        button_B = msg.buttons[1] # button B
        
        if (x_cmd != 0) & (y_cmd != 0) & (r_cmd != 0):
            self.flag_autonomous = False # override Nav2 cmd_vel with Joy stick
            print(f"Joy Override ---- y_cmd: {y_cmd} | x_cmd: {x_cmd}")
        
        threshold = 0.5

        if self.flag_autonomous == False:
            # Moving forward and reverse
            if y_cmd > threshold:
                self.forward()
            elif y_cmd < -threshold:
                self.reverse()
            # Sliding left and right
            elif x_cmd > threshold and abs(y_cmd) < threshold:
                self.slide_left()
            elif x_cmd < -threshold and abs(y_cmd) < threshold:
                self.slide_right()
            # Rotating based on combined x and y commands
            elif r_cmd > threshold:
                self.rotate_left()
            elif r_cmd < -threshold:
                self.rotate_right()
            #Stop if joystick is in or near the center
            else:
                self.stop()

        # press B to stop all
        if button_B == 1:
            self.stop()


    def forward(self):
        print('forward')
        self.pi.write(self.frmotor_forward, 1)
        self.pi.write(self.frmotor_backward, 0)
        self.pi.write(self.flmotor_forward, 1)
        self.pi.write(self.flmotor_backward, 0)
        self.pi.write(self.brmotor_forward, 1)
        self.pi.write(self.brmotor_backward, 0)
        self.pi.write(self.blmotor_forward, 1)
        self.pi.write(self.blmotor_backward, 0)

    def reverse(self):
        print('reverse')
        self.pi.write(self.frmotor_forward, 0)
        self.pi.write(self.frmotor_backward, 1)
        self.pi.write(self.flmotor_forward, 0)
        self.pi.write(self.flmotor_backward, 1)
        self.pi.write(self.brmotor_forward, 0)
        self.pi.write(self.brmotor_backward, 1)
        self.pi.write(self.blmotor_forward, 0)
        self.pi.write(self.blmotor_backward, 1)

    def stop(self):
        print('stop')
        self.pi.write(self.frmotor_forward, 0)
        self.pi.write(self.frmotor_backward, 0)
        self.pi.write(self.flmotor_forward, 0)
        self.pi.write(self.flmotor_backward, 0)
        self.pi.write(self.brmotor_forward, 0)
        self.pi.write(self.brmotor_backward, 0)
        self.pi.write(self.blmotor_forward, 0)
        self.pi.write(self.blmotor_backward, 0)

    def slide_left(self):
        print('slide left combo:')
        self.forward_left()
        self.reverse_left()

    def slide_right(self):
        print('slide right combo:')
        self.forward_right()
        self.reverse_right()

    def forward_right(self):
        print('forward right')
        self.pi.write(self.flmotor_forward, 1)
        self.pi.write(self.brmotor_forward, 1)

    def reverse_left(self):
        print('reverse left')
        self.pi.write(self.flmotor_backward, 1)
        self.pi.write(self.brmotor_backward, 1)

    def forward_left(self):
        print('forward left')
        self.pi.write(self.frmotor_forward, 1)
        self.pi.write(self.blmotor_forward, 1)

    def reverse_right(self):
        print('reverse right')
        self.pi.write(self.frmotor_backward, 1)
        self.pi.write(self.blmotor_backward, 1)

    def rotate_left(self):
        print('rotate left')
        self.pi.write(self.frmotor_forward, 1)
        self.pi.write(self.brmotor_forward, 1)
        self.pi.write(self.flmotor_backward, 1)
        self.pi.write(self.blmotor_backward, 1)

    def rotate_right(self):
        print('rotate right')
        self.pi.write(self.frmotor_backward, 1)
        self.pi.write(self.brmotor_backward, 1)
        self.pi.write(self.flmotor_forward, 1)
        self.pi.write(self.blmotor_forward, 1)

    def __del__(self):
        self.pi.stop()  # Cleanly stop pigpio at the end
        
def main(args=None):
    rclpy.init(args=args)
    joystick_listener = JoyListender()
    rclpy.spin(joystick_listener)
    joystick_listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
