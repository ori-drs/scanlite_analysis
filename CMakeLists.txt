cmake_minimum_required(VERSION 3.0.2)
project(scanlite_analysis)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/segmentation_node.py
  scripts/LiveDemoTool.py
  scripts/DataAcqTool.py
  scripts/PyCATMAUS/SegBone.py
  scripts/PyCATMAUS/TransFunction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
