import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'nerve_impulse',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Motor Neuron (Subscriber) started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received nerve impulse: "{msg.data}"')
        # In a real robot, this might trigger a motor action
        self.process_nerve_impulse(msg.data)

    def process_nerve_impulse(self, impulse_data):
        # Simulate processing of the nerve impulse
        self.get_logger().info('Processing nerve impulse...')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        simple_subscriber.get_logger().info('Stopping subscriber...')
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()