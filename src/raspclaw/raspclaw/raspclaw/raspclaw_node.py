import movement_controller

def main():## this is the main function that is called / acces point used to access all the nodes.
    print('Hi from raspclaw.')
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()## completes the whole loop (pasting the code)


if __name__ == '__main__':
    main()
## this is where all the subscriber for raspclaw will be defined and given parameters and started. entry pooint intp any raspclaw code

##