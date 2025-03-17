#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os

# Initialize ROS Node
rospy.init_node('bone_segmentation_node', anonymous=True)
rospy.loginfo("Starting Bone Segmentation Node")

# Try to locate PyCATMAUS
paths_to_try = [
    os.path.dirname(os.path.abspath(__file__)), 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "PyCATMAUS")
]

for path in paths_to_try:
    if os.path.exists(path) and path not in sys.path:
        sys.path.append(path)
        rospy.loginfo(f"Added PyCATMAUS path: {path}")

# Try importing PyCATMAUS
try:
    from PyCATMAUS.SegBone import RunBoneSeg
    rospy.loginfo("PyCATMAUS successfully loaded!")
except ModuleNotFoundError as e:
    rospy.logwarn(f"PyCATMAUS import failed: {e}")

class BoneSegmentationNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribe to the ultrasound image topic
        self.image_sub = rospy.Subscriber("/us_image", Image, self.image_callback)

        # Publisher for segmented images
        self.seg_image_pub = rospy.Publisher("/seg_bone/image", Image, queue_size=1)

        rospy.loginfo("Bone Segmentation Node Started!")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (Grayscale)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

            # Apply Bone Segmentation
            segmented_coords = RunBoneSeg(cv_image, F0=5, F1=3, Bth=0.05, JC=3)

            # Get segmentation size and color from ROS parameters
            seg_size = rospy.get_param("/segmentation/size", 2)
            color_r = rospy.get_param("/segmentation/color_r", 0)
            color_g = rospy.get_param("/segmentation/color_g", 0)
            color_b = rospy.get_param("/segmentation/color_b", 255)

            segmentation_color = (color_b, color_g, color_r)  # OpenCV uses (B, G, R)

            # Convert grayscale to BGR for color overlay
            seg_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            # Extract segmentation points
            x_coords = segmented_coords[0]
            y_coords = segmented_coords[1]

            # Draw segmentation with adjustable thickness and color
            for i in range(len(x_coords)):
                x = int(x_coords[i])
                y = int(y_coords[i])

                if 0 <= x < seg_image.shape[1] and 0 <= y < seg_image.shape[0]:
                    cv2.circle(seg_image, (x, y), radius=seg_size, color=segmentation_color, thickness=-1)

            # Convert back to ROS Image format and publish
            seg_msg = self.bridge.cv2_to_imgmsg(seg_image, "bgr8")
            self.seg_image_pub.publish(seg_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    node = BoneSegmentationNode()
    rospy.spin()
