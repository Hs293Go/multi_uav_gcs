cmake_minimum_required(VERSION 3.20.0)
project(rqt_multi_uav_gcs)

find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py)

catkin_package()

catkin_python_setup()

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
