import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pathlib import Path


sys.path.append(str(Path('/ws/ros_ws/src/raspclaw/ext/raspclawsdk/server')))
from move import*

class MovementController(Node):

    def __init__(self):
        super().__init__('MovementController') 
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel', 
            self.listener_callback,
            10)
        self.linear_threshold = 0.1
        self.angular_threshold = 0.1

    def listener_callback(self, msg):
        self.get_logger().info('Received Twist message: linear.x = %f, linear.y = %f, angular.z = %f' % 
                                (msg.linear.x, msg.linear.y, msg.angular.z))
        
        
        self.process_twist(msg)
            
       
    def process_twist(self, msg):
        
        speed = int(msg.linear.x * 100)  
        step_input = 1  
        
        if msg.angular.z > 0 :  # turning right
            command_input('right')
        elif msg.angular.z < 0:  # turn left
            command_input('left')
        else:  
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