import os, sys

currDir = os.path.dirname(os.path.realpath(__file__))
rootDir = os.path.abspath(os.path.join(currDir, '..'))
if rootDir not in sys.path: # add parent dir to paths
    sys.path.append(rootDir)

from rpc.server import ZeroMQServer
from rpc.controller.na_cpg import NaCPG, create_fully_connected_adjacency
import torch
import matplotlib.pyplot as plt
# from SerTest import SerTestClass
import time
import math
import pdb
# import cv2
import numpy as np
# from picamera2 import Picamera2
from copy import deepcopy

try:
    from robohatlib.Robohat import Robohat
    from robohatlib import RobohatConstants
    from robohatlib.hal.assemblyboard.PwmPlug import PwmPlug
    from robohatlib import RobohatConfig
    from robohatlib.hal.assemblyboard.ServoAssemblyConfig import ServoAssemblyConfig
    from robohatlib.hal.assemblyboard.servo.ServoData import ServoData
    from robohatlib.hal.datastructure.Color import Color
    from robohatlib.hal.datastructure.ExpanderDirection import ExpanderDir
    from robohatlib.hal.datastructure.ExpanderStatus import ExpanderStatus
    from robohatlib.driver_ll.datastructs.IOStatus import IOStatus

    from testlib import GuitarConfig

except ImportError:
    print("Failed to import all dependencies for SerTestClass")
    raise

possible_states = ["WAIT", "ROAM", "TARGET_FOUND",
                   "WALK", "SEARCH", "CHARGER_FOUND"]

class FSM:
    def __init__(self):
        self.state = "WAIT"

        self.robohat = Robohat(GuitarConfig.SERVOASSEMBLY_1_CONFIG, GuitarConfig.SERVOASSEMBLY_2_CONFIG, GuitarConfig.TOPBOARD_ID_SWITCH)
        self.robohat.init(GuitarConfig.SERVOBOARD_1_DATAS_LIST, GuitarConfig.SERVOBOARD_2_DATAS_LIST)
        self.robohat.do_buzzer_beep()
        
        # self.picam2 = Picamera2()
        # self.picam2.preview_configuration.main.format = "RGB888"
        # self.picam2.configure("preview")
        # self.picam2.start()

        # self.target_color_max = np.array([175,50,20], np.uint8)
        # self.target_color_min = np.array([180,255,255], np.uint8)

        adj_dict = create_fully_connected_adjacency(6)
        self.controller = NaCPG(adj_dict, angle_tracking=True)

        print(f"Initial state: {self.state}")

    def transition(self, new_state):
        if new_state in possible_states:
            print(f"Transitioning from {self.state} to {new_state}")
            self.state = new_state
            self.execute_state_action()
        else:
            print(f"Invalid state transition attempted: {new_state}")

    def run(self):
        server = ZeroMQServer()
        
        while True:
            print("Waiting for command from client...")
            command = server.listen()

            print("Received command:", command)
            if command == "WAIT":
                print("In WAIT state")
            elif command == "ROAM":
                print("In ROAM state")
            elif command == "TARGET_FOUND":
                print("In TARGET_FOUND state")
            elif command == "WALK":
                self.walk()
            elif command == "SEARCH":
                print("In SEARCH state")
                self.search()
            elif command == "CHARGER_FOUND":
                print("In CHARGER_FOUND state")

    def execute_state_action(self):
        if self.state == "WAIT":
            self.wait()
        elif self.state == "ROAM":
            self.roam()
        elif self.state == "TARGET_FOUND":
            self.target_found()
        elif self.state == "WALK":
            self.walk()
        elif self.state == "SEARCH":
            self.search()
        elif self.state == "CHARGER_FOUND":
            self.charger_found()

    def wait(self):
        pass

    def roam(self):
        pass

    def target_found(self):
        pass

    def walk(self):

    #     params = {'phase': [-6.28318531,  6.28318531,  0.47379837,  6.28318531, -1.01466379, 6.28318531, -0.53651186, -6.28318531], 
    # 'w': [ 25.13274123,  -4.65737525, -12.56637061,  -2.73507415,  -0.36040327,   0.13695773, -12.56637061,  -0.17071314], 
    # 'amplitudes': [-0.17184973, -6.28318531, -6.28318531, -0.47228612, -0.97673394, 3.05915668,  6.28318531,  6.28318531],
    #  'ha': [0., 0., 0., 0., 0., 0., 0., 0.], 
    # 'b': [ 100.       ,  100.       ,  100.       ,  100.       ,    -100.       ,  100.       ,    5.1925288,  100.       ]}

        params = {'phase': [2.899070294631235, 6.283185307179586, 1.7994643249693831, -0.17146238764692262, 6.283185307179586, -6.283185307179586],
                   'w': [6.283185307179586, -6.283185307179586, -1.7235319197065173, 6.283185307179586, -6.283185307179586, 6.283185307179586], 
                   'amplitudes': [6.283185307179586, 0.7960518971175583, -6.283185307179586, 6.283185307179586, 6.283185307179586, 6.283185307179586], 
                   'ha': [0.21720323749166623, 1.3804375923123502, 0.49772477238766694, 10.0, -10.0, 0.4632997437112862],
                   'b': [100.0, 1.6177798493588214, 100.0, 100.0, -100.0, -100.0]}

        self.controller.set_param_with_dict(params)

        angles = [.0]*16  # Initialize with dummy values
        # b has been added, but not in use because current parameters were trained with previous version of NA CPG
        buf = 50
        # update_count = 0
        first_angles = None
        while True:
            angle_buffer = []
            for j in range(buf):
                for _ in range(20):
                    angles_radians = self.controller.forward(1.0).tolist()
                    angles_degrees = [int((angle + math.pi) / (2 * math.pi) * 180) for angle in angles_radians]
                    angles[0:2], angles[2:4], angles[4:6], angles[6:8] = angles_degrees[4:6], angles_degrees[2:4], angles_degrees[0:2], angles_degrees[6:8]
                angle_buffer.append(deepcopy(angles))
            for i in range(5*5*5*5*5*buf):
                l = i % buf
                print("Servo angles (degrees):", angle_buffer[l])
                self.robohat.update_servo_data_direct(angle_buffer[l])
                time.sleep(0.05)
 
    def search(self):
        pass
        # params = {'phase': [-6.28318531, -1.14581646,  0.44064909, -1.52731181,  1.43443942,
        # 6.28318531,  4.0185173 ,  6.28318531], 'w': [ -1.70297506, -11.52369599,   1.03248715,  -3.81787687,
        # -0.15725899,  -1.96215584,   0.37505434,  -0.15555631], 'amplitudes': [-1.57079633,  0.82234301, -1.57079633, -0.23272299, -1.30865045,
        # 1.05665486, -0.59531369, -1.3773699 ], 'ha': [0.02582832, 0.01784225, 0.02252759, 0.01019621, 0.00621575,
        # 0.00118579, 0.03031356, 0.01800144], 'b': [ 1.29011006e-01,  1.00000000e+02, -1.00000000e+02,  4.83189888e-01,
        # 1.00000000e+02, -1.51697539e-02, -8.23871361e-01, -2.00560108e+00]}

        # self.controller.set_param_with_dict(params)

        # angles = [.0]*16  # Initialize with dummy values
        # # b has been added, but not in use because current parameters were trained with previous version of NA CPG
        # buf = 50
        # # update_count = 0
        
        # angle_buffer = []
        # for j in range(buf):
        #     for i in range(20):
        #         angles_radians = self.controller.forward(1.0).tolist()
        #         angles_degrees = [int((angle + math.pi) / (2 * math.pi) * 180) for angle in angles_radians]
        #         angles[:8] = angles_degrees
        #     angle_buffer.append(deepcopy(angles))
            
        # while True:
        #     for i in range(buf):
        #         l = i % buf
        #         print("Servo angles (degrees):", angle_buffer[l])
        #         self.robohat.update_servo_data_direct(angle_buffer[l])
        #         time.sleep(0.05)
            
        #     print("------------CAMERA STEP!---------")
        #     #picam2 = self.robohat.get_camera()
        #     #im = picam2.get_capture_array()
        #     im = self.picam2.capture_array()
        #     height, width = im.shape[:2]  # Get height and width of one frame
        #     image_center_x = width // 2
        #     image_center_y = height // 2
        #     hsv_frame = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        #     # Lower range for red (0 to 10)
        #     lower = np.array([100,100,70])
        #     upper = np.array([130,255,255])
        #     blue_mask = cv2.inRange(hsv_frame, lower, upper)

        #     # Upper range for red (160 to 179)
        #     #lower_red2 = np.array([140, 50, 50])
        #     #upper_red2 = np.array([179, 255, 255])
        #     #mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)

        #     # Combine the two masks to get the final red mask
        #     #red_mask = mask1 + mask2
        #     # Apply an optional morphological operation (e.g., erosion/dilation) to clean the mask
        #     kernel = np.ones((5, 5), "uint8")
        #     blue_mask = cv2.dilate(blue_mask, kernel)
        #     contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #     if contours:
        #         largest_contour = max(contours, key=cv2.contourArea)
        #         area = cv2.contourArea(largest_contour)

        #         if area > 400:
        #             print("------- CONTOUR FOUND ----------")
        #             x, y, w, h = cv2.boundingRect(largest_contour)
        #             cv2.rectangle(im, (x, y), (x+w, y+h), (0, 0, 255), 2)
        #             cv2.imwrite("image.png", im)
        #             cv2.imwrite("mask.png", blue_mask)
        #             center_x = x + w // 2
        #             center_y = y + h // 2
        #             tolerance = 250

        #             # Check if the rectangle is centered
        #             is_centered_x = abs(center_x - image_center_x) <= tolerance
        #             #is_centered_y = abs(center_y - image_center_y) <= tolerance

        #             if is_centered_x:
        #                 print("Rectangle is centered.")
        #                 for i in range(buf):
        #                     l = i % buf
        #                     self.robohat.update_servo_data_direct(angle_buffer[l])
        #                     time.sleep(0.05)
        #                 self.picam2.stop()
        #                 self.walk()
        #                 return
        #             else:
        #                 print("Rectangle is NOT centered.")

        #             # Optional: Print the deviation for feedback
        #             print(f"Deviation X: {abs(center_x - image_center_x)}, Deviation Y: {abs(center_y - image_center_y)}")


    def charger_found(self):
        pass


                
