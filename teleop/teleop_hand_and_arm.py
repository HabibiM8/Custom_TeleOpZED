import numpy as np

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig
import pyzed.sl as sl

import plotter as lib_plotter
import pyqtgraph as pg

from pathlib import Path
import time
import yaml
from multiprocessing import Process, shared_memory, Queue, Manager, Event, Lock

import cv2
import zmq
import pickle
import zlib

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from robot_control.robot_hand import H1HandController
from teleop.robot_control.robot_arm import H1ArmController, kNumMotors
from teleop.robot_control.robot_arm_ik import Arm_IK



plotter = lib_plotter.Plotter(plot_title='Joint Angles', plot_grid=True, plot_legend=True, plot_size=(800, 300), as_app=True)
plotter.plot_colors = [pg.intColor(i % 8) for i in range(16)]

class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (376, 672)# (480,640) #(720, 1280)
        self.crop_size_w = 1
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        # Create a Camera object
        zed = sl.Camera()
        self.zed = zed

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA  # Use HD720 opr HD1200 video mode, depending on camera type.
        init_params.camera_fps = 100  # Set fps at 60

        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(err) + ". Exit program.")
            exit()

        # Capture 50 frames and stop
        i = 0
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()
    
    def step(self):
        zed = self.zed
        if zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
            timestamp = zed.get_timestamp(
                sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
            # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image.get_width(), image.get_height(),
            #         timestamp.get_milliseconds()))

        bgr = np.hstack((self.image_left.numpy()[self.crop_size_h:, self.crop_size_w:-self.crop_size_w],
                         self.image_right.numpy()[self.crop_size_h:, self.crop_size_w:-self.crop_size_w]))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGRA2RGB)

        np.copyto(self.img_array, rgb)

        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        head_rmat = head_mat[:3, :3]

        left_wrist_mat[2, 3] +=0.45
        right_wrist_mat[2,3] +=0.45
        left_wrist_mat[0, 3] +=0.20
        right_wrist_mat[0,3] +=0.20

        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

        # print(left_qpos)
        # print(right_qpos)
        # print('-------------------')
        plotter.plot('left_qpos', left_qpos, labels=['q', 's'], legends=[i for i  in range(len(left_qpos))])
        plotter.plot('right_qpos', right_qpos, labels=['q', 's'], legends=[i for i  in range(len(right_qpos))])
        plotter.update()


        return head_rmat, left_wrist_mat, right_wrist_mat, left_qpos, right_qpos


if __name__ == '__main__':
    manager = Manager()
    image_queue = manager.Queue()
    teleoperator = VuerTeleop('inspire_hand.yml')
    # h1hand = H1HandController()
    h1arm = H1ArmController()
    arm_ik = Arm_IK()
    #sm = VRImage((720, 1280)) #((480,640))
    #image_process = Process(target=image_receiver, args=(sm, teleoperator.resolution, teleoperator.crop_size_w, teleoperator.crop_size_h))
    #image_process.start()
            
    try:
        user_input = input("Please enter the start signal (enter 's' to start the subsequent program):")
        if user_input.lower() == 's':
            while True:
                armstate = None
                armv = None 
                #frame = sm.read_image()
                #np.copyto(teleoperator.img_array, np.array(frame))
                #handstate = h1hand.get_hand_state()

                q_poseList=np.zeros(kNumMotors) # h1 with 20 motors
                q_tau_ff=np.zeros(kNumMotors)
                armstate,armv = h1arm.GetMotorState()

                ik_armstate = np.concatenate((armstate[4:], armstate[:4]))
                ik_armv= np.concatenate((armv[4:], armv[:4]))
                print("armstate: ", armstate)
                print("ik_armstate: ", ik_armstate)

                head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
                sol_q ,tau_ff, flag = arm_ik.ik_fun(left_pose, right_pose, ik_armstate, ik_armv)
                if flag:
                    q_poseList[12:20] = np.concatenate((sol_q[4:], sol_q[:4]))
                    q_tau_ff[12:20] = np.concatenate((tau_ff[4:], tau_ff[:4]))
                    q_poseList[12:12+4] = armstate[:4]
                    q_tau_ff[12:12+4] = np.zeros(4)

                else:
                    q_poseList[12:20] = armstate
                    q_tau_ff = np.zeros(20)

                plotter.plot('left_arm', q_poseList[12:12+4], labels=['q', 's'], legends=[i for i  in range(4)])
                plotter.plot('right_arm', q_poseList[12+4:], labels=['q', 's'], legends=[i for i  in range(4)])

                # h1arm.SetMotorPose(q_poseList, q_tau_ff)

                # if right_qpos is not None and left_qpos is not None:
                #     # 4,5: index 6,7: middle, 0,1: pinky, 2,3: ring, 8,9: thumb
                #     right_angles = [1.7 - right_qpos[i] for i in [4, 6, 2, 0]]
                #     right_angles.append(1.2 - right_qpos[8])
                #     right_angles.append(0.5 - right_qpos[9])

                #     left_angles = [1.7- left_qpos[i] for i in  [4, 6, 2, 0]]
                #     left_angles.append(1.2 - left_qpos[8])
                #     left_angles.append(0.5 - left_qpos[9])
                #     h1hand.crtl(right_angles,left_angles)

    except KeyboardInterrupt:
        exit(0)
