import asyncio
import threading
import tkinter as tk
import websockets
import json
import httpx
import os
import time 

from tkinter import ttk, Tk, scrolledtext
import tkinter.font as tkfont

# FMS_WS_URL = "ws://fms:8081/ws/updates"
FMS_WS_URL = os.environ["FMS_WS_URL"]
FMS_API_URL = "http://fms:8081/pick"
FMS_API_GET_URL = "http://fms:8081/getPick"

class App:
    def __init__(self):
        # * initialize all ui elements, only declare "self." to most necessary parts
        self._init_UI_elements()

        # * start websocket client
        threading.Thread(target=self.start_ws_listener, daemon=True).start()

        # * store last api request state,
        # * this variable changed and updated through websocket
        # * to trigger data into JSON Response field
        self.state_api_request = None

    def _init_UI_elements(self):
        # * init
        self.root = Tk()
        self.root.title("User App HMI Panel")
        self.root.geometry("1350x800")
        
        # * main container
        main_panel = tk.Frame(self.root)
        main_panel.pack(fill='both', expand=True, padx=10, pady=10)

        # * split LR
        left_panel = tk.Frame(main_panel, bg="lightgray")
        left_panel.grid(row=0, column=0, sticky="nsew")
        right_panel = tk.Frame(main_panel, bg="white")
        right_panel.grid(row=0, column=1, sticky="nsew")
        
        # ! Configure column/row weights for resizing
        main_panel.columnconfigure(0, weight=1)
        main_panel.columnconfigure(1, weight=1)
        main_panel.rowconfigure(0, weight=1)
        left_panel.columnconfigure(0, weight=1)
        left_panel.rowconfigure(0, weight=1)
        left_panel.rowconfigure(1, weight=1)

        left_top_panel = tk.Frame(left_panel)
        left_top_panel.grid(row=0, column=0, sticky="nsew")
        left_bottom_panel = tk.Frame(left_panel)
        left_bottom_panel.grid(row=1, column=0, sticky="nsew")


        # ! -------------- LEFT TOP PANEL -----------------------    
        left_top_panel.rowconfigure(0, weight=1)
        left_top_panel.rowconfigure(1, weight=0)
        left_top_panel.columnconfigure(0, weight=1)
        left_top_panel.columnconfigure(1, weight=1)

        left_top_panel_L = tk.Frame(left_top_panel)
        left_top_panel_L.grid(row=0, column=0, sticky="nsew", padx=(5, 5), pady=(5, 5))
        
        left_top_panel_R = tk.Frame(left_top_panel)
        left_top_panel_R.grid(row=0, column=1, sticky="nsew", padx=(5, 5), pady=(5, 5))
        
        left_top_panel_L.columnconfigure(0, weight=1)
        left_top_panel_L.rowconfigure(0, weight=1)
        left_top_panel_L.grid_propagate(False)
        left_top_panel_R.columnconfigure(0, weight=1)
        left_top_panel_R.rowconfigure(0, weight=1)
        left_top_panel_R.grid_propagate(False)

        # # * scrollable left
        small_font = tkfont.Font(family="Courier", size=10)  # or any other font
        self.scroll_json_input = scrolledtext.ScrolledText(left_top_panel_L, wrap=tk.WORD, font=small_font)
        self.scroll_json_input.grid(row=0, column=0, sticky="nsew", pady=(0, 0))

        self.scroll_json_output = scrolledtext.ScrolledText(left_top_panel_R, wrap=tk.WORD, state='disabled')
        self.scroll_json_output.grid(row=0, column=1, sticky="nsew", pady=(0, 0))

        self.scroll_json_input.insert(tk.END, "{\n  \"pickId\": 123, \
                                                \n  \"quantity\": 4\n}")
        
        self.scroll_json_output.config(state='normal')
        self.scroll_json_output.insert(tk.END, "{}")
        self.scroll_json_output.config(state='disabled')
        # ! -------------- END LEFT TOP PANEL -----------------------    


        # ! -------------- START LEFT BOTTOM PANEL --------------         
        left_bottom_panel.rowconfigure(0, weight=1)
        left_bottom_panel.rowconfigure(1, weight=5)
        left_bottom_panel.columnconfigure(0, weight=1)
        left_bottom_panel.columnconfigure(1, weight=0)
        left_bottom_panel.grid_propagate(False)
        
        left_bottom_panel_T = tk.Frame(left_bottom_panel)
        left_bottom_panel_T.grid(row=0, column=0, sticky="nsew")
        left_bottom_panel_B = tk.Frame(left_bottom_panel)
        left_bottom_panel_B.grid(row=1, column=0, sticky="nsew")
        
        left_bottom_panel_T.columnconfigure(0, weight=1)
        left_bottom_panel_T.columnconfigure(1, weight=4)
        left_bottom_panel_T.rowconfigure(0, weight=1)
        left_bottom_panel_T.grid_propagate(False)
        left_bottom_panel_B.columnconfigure(0, weight=1)
        left_bottom_panel_B.columnconfigure(1, weight=1)
        left_bottom_panel_B.rowconfigure(0, weight=1)
        left_bottom_panel_B.grid_propagate(False)
        
        # * start - request button panel
        request_button = tk.Button(left_bottom_panel_T, text="Send Request", command=self.send_command)
        request_button.grid(row=0, column=0, padx=(5, 5), pady=(5, 5))
        
        self.request_label_var = tk.StringVar()
        request_label = tk.Label(left_bottom_panel_T, textvariable=self.request_label_var).grid(row=0, column=1, sticky="nsew")
        self.request_label_var.set("Ready!")
        # * end - request button panel
        # ! -------------- END LEFT BOTTOM PANEL -------------- 


        # ! -------------- START RIGHT PANEL --------------         
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=1)
        right_panel.rowconfigure(1, weight=1)
        right_panel.rowconfigure(2, weight=2)

        right_panel_barcode = tk.Frame(right_panel, bg="white")
        right_panel_barcode.grid(row=0, column=0, sticky="nsew", padx=(10, 10), pady=(10, 10))
        right_panel_robot = tk.Frame(right_panel, bg="white")
        right_panel_robot.grid(row=1, column=0, sticky="nsew", padx=(10, 10), pady=(10, 10))
        right_panel_status = tk.Frame(right_panel, bg="white")
        right_panel_status.grid(row=2, column=0, sticky="nsew")

        right_panel_barcode.grid_propagate(False)
        right_panel_robot.grid_propagate(False)
        right_panel_status.grid_propagate(False)

        right_panel_status.rowconfigure(0, weight=1)
        right_panel_status.columnconfigure(0, weight=2)
        right_panel_status.columnconfigure(1, weight=1)

        right_bottom_bool_status_panel = tk.Frame(right_panel_status, bg="white")
        right_bottom_bool_status_panel.grid(row=0, column=0, sticky="nsew", padx=(10, 10), pady=(10, 10))
        right_bottom_lamp_status_panel = tk.Frame(right_panel_status, bg="white")
        right_bottom_lamp_status_panel.grid(row=0, column=1, sticky="nsew", padx=(10, 10), pady=(10, 10))

        right_bottom_bool_status_panel.rowconfigure(0, weight=1)
        right_bottom_bool_status_panel.columnconfigure(0, weight=1)
        right_bottom_bool_status_panel.grid_propagate(False)
        # ! -------------- END RIGHT PANEL --------------         


        # ! ------------- START barcode_status PANEL -------------
        self.img_barcode_scanner = tk.PhotoImage(file="sprites/barcode_scanner.png")

        self.barcode_label = tk.Label(right_panel_barcode, bg="white", font=("Arial", 30), text="12345", compound="center", fg="black", image=self.img_barcode_scanner)
        self.barcode_label.grid(row=0, column=0, sticky="nsew")

        right_panel_barcode.rowconfigure(0, weight=1)
        right_panel_barcode.columnconfigure(0, weight=1)
        right_panel_barcode.grid_propagate(False)
        # ! ------------- END barcode_status PANEL -------------

        
        # ! ------------- START robot_status PANEL -------------
        self.img_robot_idle = tk.PhotoImage(file="sprites/robot_idle.png")
        self.img_robot_run = tk.PhotoImage(file="sprites/robot_running.png")

        self.robot_label = tk.Label(right_panel_robot, bg="white", image=self.img_robot_idle)
        self.robot_label.grid(row=0, column=0, sticky="nsew")

        right_panel_robot.rowconfigure(0, weight=1)
        right_panel_robot.columnconfigure(0, weight=1)
        right_panel_robot.grid_propagate(False)
        # ! ------------- END robot_status PANEL -------------
        
        
        # ! ------------- START bool_status PANEL -------------
        right_bottom_bool_status_panel.columnconfigure(0, weight=1)
        right_bottom_bool_status_panel.rowconfigure(0, weight=1)
        right_bottom_bool_status_panel.rowconfigure(1, weight=1)

        right_bottom_bool_status_panel_T = tk.Frame(right_bottom_bool_status_panel)
        right_bottom_bool_status_panel_T.grid(row=0, column=0, sticky="nsew", padx=(10, 10), pady=(10, 10))
        right_bottom_bool_status_panel_B = tk.Frame(right_bottom_bool_status_panel)
        right_bottom_bool_status_panel_B.grid(row=1, column=0, sticky="nsew", padx=(10, 10), pady=(10, 10))

        # * top = doorhandle_state
        # * bottom = emergency button state
        self.img_doorhandle_state = tk.PhotoImage(file="sprites/doorhandle_state.png")
        self.img_ebutton_state = tk.PhotoImage(file="sprites/ebutton_state.png")

        self.doorhandle_label = tk.Label(right_bottom_bool_status_panel_T, bg="white", font=("Arial", 30), text="False", compound="center", fg="black", image=self.img_doorhandle_state)
        self.doorhandle_label.grid(row=0, column=0, sticky="nsew")
        self.ebutton_label = tk.Label(right_bottom_bool_status_panel_B, bg="white", font=("Arial", 30), text="False", compound="center", fg="black", image=self.img_ebutton_state)
        self.ebutton_label.grid(row=0, column=0, sticky="nsew")

        right_bottom_bool_status_panel_T.rowconfigure(0, weight=1)
        right_bottom_bool_status_panel_T.columnconfigure(0, weight=1)
        right_bottom_bool_status_panel_T.grid_propagate(False)
        right_bottom_bool_status_panel_B.rowconfigure(0, weight=1)
        right_bottom_bool_status_panel_B.columnconfigure(0, weight=1)
        right_bottom_bool_status_panel_B.grid_propagate(False)
        # ! ------------- END bool_status PANEL -------------        


        # ! ------------- START lamp_status PANEL -------------
        self.img_stack_light_red = tk.PhotoImage(file="sprites/stacklight_red.png")
        self.img_stack_light_yellow = tk.PhotoImage(file="sprites/stacklight_yellow.png")
        self.img_stack_light_green = tk.PhotoImage(file="sprites/stacklight_green.png")

        self.stack_light_label = tk.Label(right_bottom_lamp_status_panel, bg="white", image=self.img_stack_light_yellow)
        # self.stack_light_label.image = self.img_stack_light_yellow  # keep reference
        self.stack_light_label.grid(row=0, column=0, sticky="nsew")

        right_bottom_lamp_status_panel.rowconfigure(0, weight=1)
        right_bottom_lamp_status_panel.columnconfigure(0, weight=1)
        right_bottom_lamp_status_panel.grid_propagate(False)
        # ! ------------- END lamp_status PANEL -------------

    def enter_mainloop_UI(self):
        self.root.mainloop()



    # ! ----- START API handler ----- 
    def send_command(self):
        # *run rest api async in background
        threading.Thread(target=self.run_rest_api_async, daemon=True).start()

    def run_rest_api_async(self):
        asyncio.run(self.async_rest_api_call())

    async def async_rest_api_call(self):
        # * check 1: if json data is readable
        is_readable = False
        json_data = {}
        try:
            json_data = json.loads(self.scroll_json_input.get("1.0", tk.END).strip())
            is_readable = True
        except:
            pass
        if(is_readable):
            self.request_label_var.set("[Info] Request read done!")
        else:
            self.request_label_var.set("[Error] JSON parse error")

        # * check 2: if json pickId and quantity are exist and both are ints
        is_keys_check = False
        keys_check = ['pickId', "quantity"]
        for kc in reversed(keys_check):
            if((kc in json_data.keys()) and isinstance(json_data[kc], int)):
                keys_check.remove(kc)
        is_keys_check = (len(keys_check) == 0)  # if all necessary keys exist, continue
        if(is_keys_check):
            self.request_label_var.set("[Info] Keys check done!")
        else:
            self.request_label_var.set("[Error] JSON elements incomplete!")
            
        # * check 3: submit request. only send request if valid
        is_valid = is_readable and is_keys_check
        is_request_sent = False
        if(is_valid):
            try:
                self.root.after(0, self.update_ui_api, "Waiting API response ...")
                async with httpx.AsyncClient(timeout=5) as client:
                    response = await client.post(FMS_API_URL, json=json_data)
                    result = response.json()
                    print("API Response:", result)
                    
                    # Optional: update UI from response
                    self.root.after(0, self.update_ui_api, result)
                    is_request_sent = True
            except Exception as e:
                print("Async REST API error:", e)

        if(is_request_sent):
            self.request_label_var.set("[Info] Request sent!")
        else:
            if not is_valid:
                self.request_label_var.set("[Error] JSON structure is wrong!")                
            else:
                self.request_label_var.set("[Error] Connection issue, request not sent!")

    def update_ui_api(self, message):
        # * update ui for api response
        self.request_label_var.set(message)

    def update_ui_response(self, json_data):
        # * update ui for JSON response
        self.scroll_json_output.config(state='normal')
        self.scroll_json_output.delete("1.0", tk.END)
        self.scroll_json_output.insert(tk.END, json.dumps(json_data, indent=2))
        self.scroll_json_output.config(state='disabled')

    # ! ----- WS handler ----- 
    def start_ws_listener(self):
        asyncio.run(self.ws_handler())

    async def ws_handler(self):
        try:
            async with websockets.connect(FMS_WS_URL) as websocket:
                while True:
                    message = await websocket.recv()
                    print("Received from FMS:", message)

                    # * check state
                    json_data = json.loads(message)
                    self.root.after(0, self.update_ui_ws, json_data)
                    is_get_needed = False
                    if(self.state_api_request is not None):

                        # * IMPORTANT: check if the state_api is equal with previous one or not
                        # * this is where the triggering mechanism through websocket works, which is
                        # * storing temporary variable and compare. if the variable changes, simply
                        # * perform get command from FMS to acquire response JSON
                        if(self.state_api_request != json_data["state_apiget"]):
                            is_get_needed = True
                            self.state_api_request = json_data["state_apiget"]
                    else:
                        self.state_api_request = json_data["state_apiget"]

                    # * if get command needed, pull new data altogether and update
                    if is_get_needed:
                        asyncio.create_task(self.async_rest_api_call_get_pick_confirm())
        except Exception as e:
            print("WebSocket error:", e)
            self.root.after(0, self.update_ui_api, "[Error] Connection lost")

    async def async_rest_api_call_get_pick_confirm(self):
        # * get the data from FMS asynchronously, then set UI values
        try:
            async with httpx.AsyncClient(timeout=5) as client:
                response = await client.get(FMS_API_GET_URL)
                result = response.json()
                
                # Optional: update UI from response
                self.root.after(0, self.update_ui_response, result)
                self.root.after(0, self.update_ui_api, "Ready!")

        except Exception as e:
            print("Async REST API error:", e)

    def update_ui_ws(self, json_message):
        # * update barcode
        self.barcode_label.config(text=json_message["state_barcode"])

        # * update state of the robot
        state_robot = json_message["state_machine"]
        self.robot_label.config(image=self.img_robot_idle if state_robot else self.img_robot_run)

        # * update door state
        state_door = json_message["state_door"]
        state_ebutton = json_message["state_ebutton"]
        self.doorhandle_label.config(text="True" if state_door else "False")
        self.ebutton_label.config(text="True" if state_ebutton else "False")

        # * update stacklight
        state_stacklight = json_message["state_stacklight"]
        if(state_stacklight == -1):
            self.stack_light_label.config(image=self.img_stack_light_red)
        elif (state_stacklight == 1):
            self.stack_light_label.config(image=self.img_stack_light_yellow)
        elif (state_stacklight == 0):
            self.stack_light_label.config(image=self.img_stack_light_green)


# ----- Main -----
if __name__ == "__main__":
    # * delay to wait for FMS start first
    time.sleep(1.0)

    app = App()
    app.enter_mainloop_UI()
