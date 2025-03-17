#!/usr/bin/env python3
import os
import rospy
import subprocess
import tkinter as tk
from tkinter import filedialog
from std_msgs.msg import String  # Change this to your actual topic type

def select_rosbag():
    """Opens a file dialog for selecting a ROS bag file."""
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    file_path = filedialog.askopenfilename(
        title="Select a ROS bag file",
        filetypes=[("ROS Bag Files", "*.bag")],  # Filter for .bag files
        initialdir=os.path.expanduser("~")  # Start in user's home directory
    )
    
    if file_path:
        print(f"Selected ROS bag: {file_path}")
        return file_path
    else:
        print("No file selected.")
        return None

def play_rosbag(bag_file):
    """Plays the selected ROS bag file in a separate process."""
    if bag_file:
        play_process = subprocess.Popen(["rosbag", "play", bag_file],stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return play_process

def callback(msg):
    """Callback function for the subscribed topic."""
    rospy.loginfo(f"Received message: {msg.data}")

def subscribe_to_topic():
    """Subscribes to a topic and processes incoming messages."""
    rospy.init_node("rosbag_subscriber", anonymous=True)
    rospy.Subscriber("/us_image", String, callback)  # Change topic and type
    rospy.spin()

if __name__ == "__main__":
    selected_bag = select_rosbag()
    
    if selected_bag:
        try:
            rosbag_process = play_rosbag(selected_bag)  # Start playing in a separate process
            rospy.sleep(1)  # Give time for rosbag to start
            subscribe_to_topic()  # Start subscriber while the bag is playing
        except KeyboardInterrupt:
            print("\nShutting down...")
            rosbag_process.terminate()
        # finally:
        # if KeyboardInterrupt:
        #         rosbag_process.terminate()  # Stop rosbag player when the script exits
