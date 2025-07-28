import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from py_srvcli_interfaces.srv import Empty

class EButton(Node):
    def __init__(self):
        super().__init__('emergency_button_manager')
        self.cur_state = False  # store current state
        # * generate two services, each for enable and reset the button
        self.srv = self.create_service(Empty, 'enable_ebutton', self.enable_ebutton_callback)
        self.srv = self.create_service(Empty, 'reset_ebutton', self.reset_ebutton_callback)

        # * publish button state
        self.publisher_ = self.create_publisher(Bool, 'ebutton_state', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # * update publish data
        msg = Bool()
        msg.data = self.cur_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing - EButton: "%s"' % msg.data)

    def enable_ebutton_callback(self, request, response):
        self.cur_state = False
        self.get_logger().info('Enable EButton State: "%s"' % self.cur_state)
        return response
        
    def reset_ebutton_callback(self, request, response):
        self.cur_state = True
        self.get_logger().info('Enable EButton State: "%s"' % self.cur_state)
        return response
        
        
def main():
    # * init
    rclpy.init()

    # * loop
    ebutton_handle = EButton()
    rclpy.spin(ebutton_handle)

    # * destroy during exit
    ebutton_handle.destroy_node()
    rclpy.shutdown()

        
if __name__ == "__main__":
    main()