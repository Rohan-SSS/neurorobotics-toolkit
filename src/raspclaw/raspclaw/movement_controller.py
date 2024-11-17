import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pathlib import Path


sys.path.append(str(Path('/ws/ros_ws/src/raspclaw/ext/raspclawsdk/server')))
from move import*

class MovementController(Node):

    def __init__(self):
        super().__init__('MovementController') ## name should be the same as the described one 
        self.subscription = self.create_subscription(# creates a subscriptions 
            Twist,
            'cmd_vel', ## default command velocity default topic for publishing velocty commands
            self.listener_callback,
            10)
        self.linear_threshold = 0.1
        self.angular_threshold = 0.1

    def listener_callback(self, msg):
        self.get_logger().info('Received Twist message: linear.x = %f, linear.y = %f, angular.z = %f' % 
                                (msg.linear.x, msg.linear.y, msg.angular.z))
        
        # Convert Twist message to discrete commands
        self.process_twist(msg)
            
    '''
        method to conver twist messages into discrete commands using if-else
def convert_twist_to_discrete(twist_msg):
    linear_threshold = 0.1  
    angular_threshold = 0.1  

    
    linear_velocity = twist_msg.linear.x  # Forward/backward speed
    angular_velocity = twist_msg.angular.z  # Turning speed

    left_command = 0
    right_command = 0

    # Check if the robot is moving forward or backward
    if abs(linear_velocity) > linear_threshold:
        if linear_velocity > 0:
            # Moving forward
            left_command = 1
            right_command = 1
        elif linear_velocity < 0:
            # Moving backward
            left_command = -1
            right_command = -1

    # Check if the robot is turning
    if abs(angular_velocity) > angular_threshold:
        if angular_velocity > 0:
            # Turning right
            left_command = 1
            right_command = -1
        elif angular_velocity < 0:
            # Turning left
            left_command = -1
            right_command = 1

    # Return discrete commands
    return left_command, right_command

     '''     
    def process_twist(self, msg):
        # Define speed and step input based on Twist message
        speed = int(msg.linear.x * 100)  # Scale linear.x to a suitable speed range (example scaling)
        step_input = 1  # steps taken will be 1 at the speed specified 
        # turn right or letf based on agular.z that is we check the angular.z value first and if it si 0 then we proceed to move it forward/back 
        if msg.angular.z > 0 :  # turning right
            command_input('right')
        elif msg.angular.z < 0:  # turn left
            command_input('left')
        else:  # move forward or backward based on linear.x
            if speed > 0:
                command_input('forward')  # move forward
            elif speed < 0:
                command_input('backward')  # move backward



        






## task 1 - import move,py into this script 
## task 2 - modify minimal sub node to receive twist messsages
## task 3 - implement if-else logic to convert twist message into discrete message fo raspclaw sdk
## task 4 - use one of the methods from move.py to move the servo using the node.
## move.py is in ros_ws->build->controllers->src->raspclaw->ext/raspclaw->server
## the setup.py file is in ros_ws->build->opencv_tests
## moved the file 
## ./scripts/build_ros_packages.sh this is from within the container for outside - djinn build nrt 
