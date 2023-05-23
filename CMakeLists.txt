cmake_minimum_required(VERSION 3.0.2)
project(camera_utils_ros)


find_package(catkin REQUIRED COMPONENTS
    rospy
    cv_bridge
    sensor_msgs
    std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/camera_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

