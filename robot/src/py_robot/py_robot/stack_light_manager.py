import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool
from py_srvcli_interfaces.srv import Empty

class StackLight(Node):

    def __init__(self):
        super().__init__('stack_light_manager')
        timer_period = 0.1  # seconds
        self.last_state_door = False # open by default for it to be paused
        self.last_state_ebutton = False # active by default (not pushed)
        
        # * start with PAUSED state
        self.cur_state = 1
        self.publisher_ = self.create_publisher(Int8, 'stack_light_state', 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # * subscribe both door and ebutton state to change this state
        self.sub_door = self.create_subscription(Bool, 'door_handle_state', self.callback_door, 1)
        self.sub_ebutton = self.create_subscription(Bool, 'ebutton_state', self.callback_ebutton, 1)

    def timer_callback(self):
        msg = Int8()
        msg.data = self.cur_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing - StackLight: "%s"' % msg.data)

    def callback_door(self, msg: Bool):
        # * store and change state only if its different from before
        if(self.last_state_door != msg.data):
            self.last_state_door = msg.data
            self.change_state()

    def callback_ebutton(self, msg:Bool):
        # * store and change state only if its different from before
        if(self.last_state_ebutton != msg.data):
            self.last_state_ebutton = msg.data
            self.change_state()
        
    def change_state(self):
        # * decide the state here
        if(self.last_state_ebutton):
            self.cur_state = -1 # emergency
        else: 
            if(self.last_state_door):
                self.cur_state = 0 # operational
            else:
                self.cur_state = 1 # paused

def main():
    # * init
    rclpy.init()

    # * loop
    stacklight_handle = StackLight()
    rclpy.spin(stacklight_handle)

    # * destroy
    stacklight_handle.destroy_node()
    rclpy.shutdown()

        
if __name__ == "__main__":
    main()