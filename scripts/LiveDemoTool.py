#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from std_msgs.msg import Header
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import time
from copy import copy
from scipy.io import savemat

# Try to find and import PyCATMAUS
rospy.init_node('bone_segmentation_node', anonymous=True)
rospy.loginfo("Starting Bone Segmentation Node")

# Check for PyCATMAUS installation
paths_to_try = [
    os.path.dirname(os.path.abspath(__file__)),  # Current script directory
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "PyCATMAUS")
]

for path in paths_to_try:
    if os.path.exists(path) and path not in sys.path:
        sys.path.append(path)
        rospy.loginfo(f"Added PyCATMAUS path: {path}")

try:
    from PyCATMAUS.SegBone import RunBoneSeg as BoneSeg
    from PyCATMAUS.TransFunction import quat2rotm, transfrom_i2l, imgp
    rospy.loginfo("PyCATMAUS successfully loaded!")
except ModuleNotFoundError as e:
    rospy.logwarn(f"PyCATMAUS import failed: {e}")

# ROS Node for Data Exchange
class DataExchangeNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.motion_buff = None
        self.img_buff = np.zeros((480, 640))

        self.img_subscriber = rospy.Subscriber('/us_image', Image, self.update_img)
        self.motion_subscriber = rospy.Subscriber('/vicon/clarius_5_marker/clarius_5_marker', TransformStamped, self.update_motion)

    def update_img(self, data):
        try:
            self.img_buff = self.bridge.imgmsg_to_cv2(data, "mono8")
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")

    def update_motion(self, data):
        self.motion_buff = data.transform

# Main GUI App
class CatMausApp:
    def __init__(self, master):
        self.master = master
        self.master.title("CAT&MAUS Demo")

        # GUI Colors
        self.bg_color = '#d9d9d9'
        self.btn_on = '#229955'
        self.btn_off = '#bbbbbb'

        # Figure and canvas
        self.fig = plt.figure(figsize=(14, 6.4), dpi=100, facecolor=self.bg_color, tight_layout=True)
        ax1 = self.fig.add_subplot(121, title="Ultrasound Image", frameon=False)
        ax1.tick_params(axis='both', which='both', bottom=False, left=False, labelleft=False, labelbottom=False)

        self.USI1 = ax1.imshow(np.zeros((480, 640)), cmap='gray', vmin=0, vmax=255)
        self.Seg1, = ax1.plot([], [], 'r.')

        self.ax2 = self.fig.add_subplot(122, projection='3d')
        self.ax2.set_xlim([50, 450])
        self.ax2.set_ylim([1150, 1550])
        self.ax2.set_zlim([800, 1200])
        self.scanner_x, = self.ax2.plot3D([0, 20], [0, 0], [0, 0], color='r')
        self.scanner_y, = self.ax2.plot3D([0, 0], [0, 20], [0, 0], color='g')
        self.scanner_z, = self.ax2.plot3D([0, 0], [0, 0], [0, 20], color='b')
        self.reconstruct_3D, = self.ax2.plot3D([], [], [], 'b.', ms=1)

        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack()

        # Buttons
        self.button_connect = tk.Button(master, text="Connect Data", bg=self.btn_off, command=self.connect_data)
        self.button_connect.pack()
        self.button_seg = tk.Button(master, text="BoneSeg", bg=self.btn_off, command=self.toggle_segmentation)
        self.button_seg.pack()
        self.button_reconstruct = tk.Button(master, text="Start Reconstruction", bg=self.btn_off, command=self.toggle_reconstruction)
        self.button_reconstruct.pack()
        self.button_save = tk.Button(master, text="Save Data", bg=self.btn_off, command=self.save_reconstruction)
        self.button_save.pack()

        # Flags
        self.data_node = None
        self.var_connected = False
        self.var_segswitch = False
        self.var_reconstruct = False

        self.master.mainloop()

    def connect_data(self):
        if not self.var_connected:
            self.data_node = DataExchangeNode()
            self.var_connected = True
            self.button_connect.config(text="Connected", bg=self.btn_on)
            self.update_display()
        else:
            self.var_connected = False
            self.button_connect.config(text="Connect Data", bg=self.btn_off)

    def toggle_segmentation(self):
        self.var_segswitch = not self.var_segswitch
        color = self.btn_on if self.var_segswitch else self.btn_off
        self.button_seg.config(bg=color)
        if not self.var_segswitch:
            self.Seg1.set_xdata([])
            self.Seg1.set_ydata([])

    def toggle_reconstruction(self):
        self.var_reconstruct = not self.var_reconstruct
        text = "Stop Reconstruction" if self.var_reconstruct else "Start Reconstruction"
        color = self.btn_on if self.var_reconstruct else self.btn_off
        self.button_reconstruct.config(text=text, bg=color)

    def update_display(self):
        if self.var_connected:
            img = self.data_node.img_buff
            self.USI1.set_data(img)

            if self.var_segswitch:
                seg_coords = BoneSeg(img, F0=1, F1=1, Bth=0.1, JC=1)
                self.Seg1.set_xdata(seg_coords[0])
                self.Seg1.set_ydata(seg_coords[1])

            if self.var_reconstruct and self.data_node.motion_buff:
                trans = self.data_node.motion_buff.translation
                rot = self.data_node.motion_buff.rotation
                pose = [trans.x * 1000, trans.y * 1000, trans.z * 1000]
                rotm = quat2rotm([rot.w, rot.x, rot.y, rot.z])
                seg_3D = transfrom_i2l(np.array((seg_coords[0], seg_coords[1])))
                transform_matrix = np.column_stack((rotm, pose))
                transform_matrix = np.row_stack((transform_matrix, [0, 0, 0, 1]))
                seg_glo = np.matmul(transform_matrix, seg_3D)
                cloud_3d = self.reconstruct_3D.get_data_3d()
                self.reconstruct_3D.set_data_3d(
                    np.append(cloud_3d[0], seg_glo[0]),
                    np.append(cloud_3d[1], seg_glo[1]),
                    np.append(cloud_3d[2], seg_glo[2])
                )

            self.canvas.draw()
            self.master.after(30, self.update_display)

    def save_reconstruction(self):
        cloud_3d = self.reconstruct_3D.get_data_3d()
        data = {"x": cloud_3d[0], "y": cloud_3d[1], "z": cloud_3d[2]}
        savemat("reconstruction.mat", data)
        rospy.loginfo("3D Reconstruction saved!")

if __name__ == "__main__":
    root = tk.Tk()
    app = CatMausApp(root)
