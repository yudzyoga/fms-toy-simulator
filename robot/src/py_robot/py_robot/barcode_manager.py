import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int8
from py_srvcli_interfaces.srv import GetValue

class BarcodeManager(Node):
    def __init__(self):
        super().__init__('barcode_manager')
        self.cur_val = 0    # store latest barcode read
        self.is_update_new_values = True    # trigger on-off mechanism
        self.srv = self.create_service(GetValue, 'barcode_scan', self.get_barcode_value)

        timer_period = 3.0  # read every seconds
        self.publisher_ = self.create_publisher(Int64, 'barcode_read_current', 1)   # publish the barcode read 
        self.timer = self.create_timer(timer_period, self.timer_callback)   # iterate new values and broadcast the barcode data

        self.sub_stacklight = self.create_subscription(Int8, 'stack_light_state', self.callback_stacklight, 1)  # get stacklight state for triggering execution

    def callback_stacklight(self, msg: Int8):
        # * only update new values if the state is operational. otherwise stop
        self.is_update_new_values = (int(msg.data) == 0)

    def timer_callback(self):
        if self.is_update_new_values:
            self.cur_val = random.randint(10000, 99999)
        msg = Int64()
        msg.data = self.cur_val
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing - Barcode: "%s"' % msg.data)

    def get_barcode_value(self, request, response):
        response.result = self.cur_val
        return response
        
def main():
    # * init
    rclpy.init()

    # * loop over the node
    barcode_manager = BarcodeManager()
    rclpy.spin(barcode_manager)

    # * destroy if exited
    barcode_manager.destroy_node()
    rclpy.shutdown()

        
if __name__ == "__main__":
    main()