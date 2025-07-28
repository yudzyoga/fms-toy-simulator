import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from py_srvcli_interfaces.srv import Empty

class DoorHandle(Node):

    def __init__(self):
        super().__init__('door_handle_manager')
        self.cur_state = True   # store door state
        self.srv = self.create_service(Empty, 'switch_door_state', self.callback_door_switch)

        self.publisher_ = self.create_publisher(Bool, 'door_handle_state', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # * publish door state
        msg = Bool()
        msg.data = self.cur_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing - Door: "%s"' % msg.data)

    def callback_door_switch(self, request, response):
        # * switch door state
        self.cur_state = not self.cur_state
        self.get_logger().info('Switch Door State: "%s"' % self.cur_state)
        return response
        
def main():
    # * init
    rclpy.init()

    # * loop over
    door_handle = DoorHandle()
    rclpy.spin(door_handle)

    # * destroy if exit
    door_handle.destroy_node()
    rclpy.shutdown()

        
if __name__ == "__main__":
    main()