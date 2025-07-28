import rclpy
import threading
import tkinter as tk
from rclpy.node import Node
from std_msgs.msg import Bool
from py_srvcli_interfaces.srv import Empty

class RobotUINode(Node):
    def __init__(self):
        super().__init__('ui_manager')
        # * get all the services to trigger changes
        self.call_door_service = self.create_client(Empty, "switch_door_state")
        self.call_ebutton_enable = self.create_client(Empty, "enable_ebutton")
        self.call_ebutton_reset = self.create_client(Empty, "reset_ebutton")

    def service_call_switch_door_state(self):
        # * simply call request without storing result since we are using Empty class
        request = Empty.Request()
        future = self.call_door_service.call_async(request)
        future.add_done_callback(self.handle_result)            

    def service_call_enable_ebutton(self):
        # * simply call request without storing result since we are using Empty class
        request = Empty.Request()
        future = self.call_ebutton_enable.call_async(request)
        future.add_done_callback(self.handle_result)

    def service_call_reset_ebutton(self):
        # * simply call request without storing result since we are using Empty class
        request = Empty.Request()
        future = self.call_ebutton_reset.call_async(request)
        future.add_done_callback(self.handle_result)

    def handle_result(self, future):
        # * callback from services, just print not store anything
        result = future.result()
        if result is not None:
            print("Success:", result)
        else:
            print("Service call failed.")
            


class RobotUI:
    def __init__(self, robot_ui_node: RobotUINode):
        self.root = tk.Tk()
        self.root.title("Trigger Panel")

        self.door_state = True
        self.ebutton_state = False

        self.img_door_open = tk.PhotoImage(file="/ws/sprites_robot/door_open.png")
        self.img_door_close = tk.PhotoImage(file="/ws/sprites_robot/door_close.png")
        self.img_button_push = tk.PhotoImage(file="/ws/sprites_robot/button_push.png")
        self.img_button_reset = tk.PhotoImage(file="/ws/sprites_robot/button_reset.png")
        self.button_door = tk.Button(self.root, image=self.img_door_open if self.door_state else self.img_door_close, text="Open Door", command=self.callback_door)
        self.button_door.pack(fill='both', expand=True)  # makes button fill entire panel
        self.button_ebutton = tk.Button(self.root, image=self.img_button_reset if self.ebutton_state else self.img_button_push, text="Push Button", command=self.callback_ebutton)
        self.button_ebutton.pack(fill='both', expand=True)  # makes button fill entire panel

        self.robot_ui_node = robot_ui_node

    def callback_door(self):
        print("Door callback!")
        # * simply switching states for door
        self.door_state = not self.door_state
        if(self.door_state):
            self.button_door.config(image=self.img_door_open)
        else:
            self.button_door.config(image=self.img_door_close)
        self.robot_ui_node.service_call_switch_door_state()

    def callback_ebutton(self):
        print("EButton callback!")
        # * perform switching for emergency button as well, but use two services instead
        self.ebutton_state = not self.ebutton_state
        if(self.ebutton_state):
            self.button_ebutton.config(image=self.img_button_reset)
            self.robot_ui_node.service_call_reset_ebutton()
        else:
            self.button_ebutton.config(image=self.img_button_push)
            self.robot_ui_node.service_call_enable_ebutton()

    def run_all(self):
        self.root.mainloop()



def main():
    # * init ROS and UI
    rclpy.init()
    robot_ui_node = RobotUINode()
    robot_ui = RobotUI(robot_ui_node)

    # * run ros in side threads
    threading.Thread(target=rclpy.spin, args=(robot_ui_node,), daemon=True).start()

    # * perform main loop of UI
    robot_ui.run_all()

    # * remove all
    robot_ui_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()