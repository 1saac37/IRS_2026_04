import json # Add this import at the top
import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class PLCHMIListener(Node):
    def __init__(self):
        super().__init__('plc_hmi_listener') # Node name

        # Create publisher: topic_name, message_type, queue_size
        self.publisher_ = self.create_publisher(String, 'hmi/unified_status',10)

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'plc/status',
            self.listener_callback,
            10
        )

        # Timer to publish every 2 seconds
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0



    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data) # parse JSON string into Python dict
            stamp = data["stamp"]
            box = data["box"]
            counts = data["counts"]
            print("📥📥 Received PLC status:")
            print(f" ⏱ Time: {stamp['sec']}.{stamp['nanosec']}")
            print(f" 📦📦 Box weight raw={box['weight_raw']}")
            print(f" 📍📍 Location: {box['location']}")
            print(f" 🔢🔢 Counts: big={counts['big']}, medium={counts['medium']}, "
            f"small={counts['small']}, total={counts['total']}")
            print() # ���� empty line at the end
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")

    def timer_callback(self):
        msg = String()
        msg.data = f'Bug spotted doing pineapple dance in aisle {self.i}!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    plc_hmi_listener = PLCHMIListener()
    rclpy.spin(plc_hmi_listener) # Keep node running
    plc_hmi_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()