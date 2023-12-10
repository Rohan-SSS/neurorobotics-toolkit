import rclpy
from .survey import SurveyNavigator, AirSimNode


def main(args=None):
    rclpy.init(args=args)

    airsim_node = AirSimNode("airsim_node")

    rclpy.spin(airsim_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airsim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

