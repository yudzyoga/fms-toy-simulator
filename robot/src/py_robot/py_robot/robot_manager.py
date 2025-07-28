import os
import time
import json
import rclpy
import httpx
import random
import uvicorn
import asyncio
import threading
import websockets
from threading import Lock
from rclpy.node import Node
from pydantic import BaseModel
from fastapi import FastAPI, Request
from std_msgs.msg import Int64, Int8, Bool
from py_srvcli_interfaces.srv import GetValue

# FMS_WS_URL = "ws://fms:8081/ws/updates"
FMS_WS_URL = os.environ["FMS_WS_URL"]
FMS_API_URL = "http://fms:8081/confirmPick"

# * confirm for pick action
class confirmPickItem(BaseModel):
    pickId: int = 123
    pickSuccessful: bool = True
    errorMessage: str = "null"
    itemBarcode: int = 123

# * define robot App
class RobotApp:
    def __init__(self):
        # * locking mechanism to avoid multiple thread access at a time
        self.lock = Lock()

        # * store all states
        self.state_barcode = 12345
        self.state_door = True
        self.state_ebutton = False
        self.state_api_get = True
        self.state_robot = "idle"
        self.state_stacklight = 0
        self.websocket = None
        self.ws_loop = None

    def get_state_ws(self):
        # * get all states in json format
        with self.lock:
            return {
                "state_apiget": bool(self.state_api_get),
                "state_door": bool(self.state_door),
                "state_ebutton": bool(self.state_ebutton),
                "state_barcode": int(self.state_barcode),
                "state_machine": bool(self.state_robot == "idle"),
                "state_stacklight": int(self.state_stacklight)
            }
        
    def manual_state_update(self, state_barcode: int=None, state_robot: str=None, state_door: bool=None, state_ebutton: bool=None, state_stacklight: Int8=None, state_apiget: bool=None):
        # * check for any changes in all variables
        with self.lock:
            is_anything_changed = False
            if(state_barcode is not None): 
                if(self.state_barcode != state_barcode):
                    self.state_barcode = state_barcode
                    is_anything_changed = True
            if(state_robot is not None): 
                if(self.state_robot != state_robot):
                    self.state_robot = state_robot
                    is_anything_changed = True
            if(state_door is not None): 
                if(self.state_door != state_door):
                    self.state_door = state_door
                    is_anything_changed = True
            if(state_ebutton is not None): 
                if(self.state_ebutton != state_ebutton):
                    self.state_ebutton = state_ebutton
                    is_anything_changed = True
            if(state_stacklight is not None): 
                if(self.state_stacklight != state_stacklight):
                    self.state_stacklight = state_stacklight
                    is_anything_changed = True
            if(state_apiget is not None):
                if(self.state_api_get != state_apiget):
                    self.state_api_get = state_apiget
                    is_anything_changed = True
            
            # * directly update the stacklight here
            if(self.state_door and not self.state_ebutton):
                if(self.state_stacklight != 0):
                    self.state_stacklight = 0
                    is_anything_changed = True
            else:
                if (self.state_ebutton):
                    if(self.state_stacklight != -1):
                        self.state_stacklight = -1
                        is_anything_changed = True
                elif not (self.state_door):
                    if(self.state_stacklight != 1):
                        self.state_stacklight = 1
                        is_anything_changed = True

        # * if anything changes, trigger the websocket push to all clients
        # * this enables real-time update from any variable change in the code
        if(is_anything_changed):
            self.trigger_ws_push()

    def start_ws_client(self):
        # * run websocket asynchrobously
        threading.Thread(target=lambda: asyncio.run(self.websocket_loop()), daemon=True).start()

    async def websocket_loop(self):
        # * connect to websocket and wait (do nothing)
        self.ws_loop = asyncio.get_event_loop()
        async with websockets.connect(FMS_WS_URL) as websocket:
            self.websocket = websocket
            await asyncio.Event().wait()

    def trigger_ws_push(self):
        # * perform push for any changes in the data
        if self.websocket and self.ws_loop:
            msg = json.dumps(self.get_state_ws())
            asyncio.run_coroutine_threadsafe(self.websocket.send(msg), self.ws_loop)
            print(f"Sent_ws_push: {msg}")

class RobotManager(Node):
    def __init__(self, robot_app: RobotApp):
        super().__init__('robot_manager')

        # * store robot app for state update
        self.robot_app = robot_app

        # * create barcode client, and all other subscription to update states
        self.call_barcode = self.create_client(GetValue, "barcode_scan")
        self.sub_barcode = self.create_subscription(Int64, 'barcode_read_current', self.callback_scanner, 1)
        self.sub_door = self.create_subscription(Bool, 'door_handle_state', self.callback_door, 1)
        self.sub_ebutton = self.create_subscription(Bool, 'ebutton_state', self.callback_ebutton, 1)
        self.sub_stacklight = self.create_subscription(Int8, 'stack_light_state', self.callback_stacklight, 1)
        
    def callback_scanner(self, msg: Int64):
        # * update the barcode scan value
        self.robot_app.manual_state_update(state_barcode=int(msg.data))

    def callback_door(self, msg: Bool):
        # * update the door state
        self.robot_app.manual_state_update(state_door=msg.data)

    def callback_ebutton(self, msg: Bool):
        # * update the emergency button state
        self.robot_app.manual_state_update(state_ebutton=msg.data)

    def callback_stacklight(self, msg: Int8):
        # * update stacklight state
        self.robot_app.manual_state_update(state_stacklight=int(msg.data))

    def call_scanner_service(self):
        # * call the barcode scanner service
        request = GetValue.Request()
        future = self.call_barcode.call_async(request)
        future.add_done_callback(self.handle_result)

    def handle_result(self, future):
        response = future.result()
        if response is not None:
            # * update the barcode state before pushing the value to JSON response
            self.robot_app.manual_state_update(state_barcode=int(response.result))
            

def create_api(robot_app: RobotApp, robot_manager: RobotManager):
    api_app = FastAPI()

    @api_app.post("/pick")
    async def do_task(request: Request):
        data = await request.json()
        asyncio.create_task(simulate_robot(data))
        return {"status": "accepted"}

    async def simulate_robot(data):
        # * check current state
        state = robot_app.get_state_ws()
        state_stacklight = state['state_stacklight']

        # * if operational
        if(state_stacklight == 0):
            # * update robot state to running
            robot_app.manual_state_update(state_robot="running")

            # * simulate work
            await asyncio.sleep(2)

            # * update recent barcode read
            robot_manager.call_scanner_service()
            state = robot_app.get_state_ws()

            # * fill up JSON return data 
            cpi = confirmPickItem(
                pickId=int(data["pickId"]),
                pickSuccessful=True,
                itemBarcode=state["state_barcode"]
            )

            # * Async post info to confirm pick
            body = cpi.model_dump()
            is_success = False
            api_check_state = state['state_apiget']
            async with httpx.AsyncClient() as client:
                response = await client.post(FMS_API_URL, json=body)
                if response.status_code == 200:
                    is_success = True
            api_check_state = not api_check_state if is_success else api_check_state

            # * update robot state to idle
            robot_app.manual_state_update(state_robot="idle", state_apiget=api_check_state)
        else:
            # * check if emergency
            is_color_red = (state_stacklight == -1)

            # * always pickSuccessful = false
            state = robot_app.get_state_ws()
            cpi = confirmPickItem(
                pickId=int(data["pickId"]),
                pickSuccessful=False,
                errorMessage="Emergency Button Pressed!" if is_color_red else "Door Opened!",
                itemBarcode=state["state_barcode"]
            )

            # * Post the update into confirmPick 
            body = cpi.model_dump()
            is_success = False
            api_check_state = state['state_apiget']
            async with httpx.AsyncClient() as client:
                response = await client.post(FMS_API_URL, json=body)
                if response.status_code == 200:
                    is_success = True

            # * switch api state to trigger websocket push
            api_check_state = not api_check_state if is_success else api_check_state
            
            robot_app.manual_state_update(state_robot="idle", state_apiget=api_check_state)            

    @api_app.get("/status")
    async def get_status():
        return robot_app.get_state_ws()

    return api_app

def start_api_server(api_app: FastAPI):
    # * initalize api server
    config = uvicorn.Config(app = api_app, host='0.0.0.0', port=8080)
    server = uvicorn.Server(config)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server.serve())

def main():
    # * delay to wait for FMS initialization
    time.sleep(1.0)

    # * initalize App to store states
    state = RobotApp()
    state.start_ws_client()

    # * initialize ROS and RobotManager node
    rclpy.init()
    robot_manager = RobotManager(state)

    # * create and start api on the side
    api_app = create_api(state, robot_manager)
    threading.Thread(target=start_api_server, args=(api_app,)).start()

    # * run ROS loop in main thread
    rclpy.spin(robot_manager)
    
    # * destroy in exit
    robot_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()