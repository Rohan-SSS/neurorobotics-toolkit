import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pathlib import Path


sys.path.append(str(Path('/ws/ros_ws/src/raspclaw/ext/raspclawsdk/server')))
import move  

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.linear_threshold = 0.1
        self.angular_threshold = 0.1

    def listener_callback(self, msg):
        self.get_logger().info('Received Twist message: linear.x = %f, linear.y = %f, angular.z = %f' % 
                                (msg.linear.x, msg.linear.y, msg.angular.z))
        
        # Convert Twist message to discrete commands
        self.process_twist(msg)
            
    '''
        method to conver twist messages into discrete commands using if-else
        def twist_to_discreet(msg)
     '''     
    def process_twist(self, msg):
        # Define speed and step input based on Twist message
        speed = int(msg.linear.x * 100)  # Scale linear.x to a suitable speed range (example scaling)
        step_input = 1  # Example step input; adjust as necessary
        # turn right or letf based on agular.z
        if msg.angular.z > 0 :  # turning right
            move.move(step_input, speed, 'right')
        elif msg.angular.z < 0:  # turn left
            move.move(step_input, speed, 'left')
        else:  # move forward or backward based on linear.x
            if speed > 0:
                move.move(step_input, speed, 'no')  # move forward
            elif speed < 0:
                move.move(step_input, -speed, 'no')  # move backward

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        






## task 1 - import move,py into this script 
## task 2 - modify minimal sub node to receive twist messsages
## task 3 - implement if-else logic to convert twist message into discrete message fo raspclaw sdk
## task 4 - use one of the methods from move.py to move the servo using the node.
## move.py is in ros_ws->build->controllers->src->raspclaw->ext/raspclaw->server
## the setup.py file is in ros_ws->build->opencv_tests
