cmake_minimum_required(VERSION 2.8.3)
project(imu_vicon_arm_calibration)
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when necessary:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(Qt5Core_DIR /opt/Qt/5.5/gcc_64/lib/cmake/Qt5Core)
set(Qt5PrintSupport_DIR /opt/Qt/5.5/gcc_64/lib/cmake/Qt5PrintSupport)
set(Qt5Gui_DIR /opt/Qt/5.5/gcc_64/lib/cmake/Qt5Gui)
set(Qt5Network_DIR /opt/Qt/5.5/gcc_64/lib/cmake/Qt5Network)
set(Qt5Widgets_DIR /opt/Qt/5.5/gcc_64/lib/cmake/Qt5Widgets)
set(QT_USE_QTGUI TRUE)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rosbag
  roscpp
  rospy
  std_msgs
  message_filters
  message_generation
  geometry_msgs
tf
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Qt5Core 5.5.1 EXACT REQUIRED)
find_package(Qt5PrintSupport 5.5.1 EXACT REQUIRED)
find_package(Qt5Gui 5.5.1 EXACT REQUIRED)
find_package(Qt5Widgets 5.5.1 EXACT REQUIRED)
find_package(Qt5Network 5.5.1 EXACT REQUIRED)

#add_service_files(FILES
    #Bag_info.srv
    #ref_info.srv
#)
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS roscpp rospy std_msgs rosbag message_filters geometry_msgs tf
  INCLUDE_DIRS include 
  LIBRARIES 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Painter_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5PrintSupport}
)
MESSAGE(STATUS,"variables is: "${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${Qt5Network_DEFINITIONS} ${Qt5Core_DEFINITIONS} ${Qt5Widgets_DEFINITIONS} ${Qt5Gui_DEFINITIONS} ${Qt5PrintSupport_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Network_EXECUTABLE_COMPILE_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS} ${Qt5PrintSupport_EXECUTABLE_COMPILE_FLAGS}")
#qt5_generate_moc(src/main.cpp main.moc)

#set(qt_srcs src/mainwindow.cpp src/main.cpp src/qcustomplot.cpp src/data_log_Widget.ui src/rosbag_worker.cpp src/refsiggenerator.cpp src/bag_worker_main.cpp)
#set(qt_hdrs include/data_collection_gui/mainwindow.h include/data_collection_gui/qcustomplot.h include/data_collection_gui/rosbag_worker.h include/data_collection_gui/refsiggenerator.h include/data_collection_gui/Emg.h include/data_collection_gui/EmgPoints.h include/data_collection_gui/EmgStamped.h include/data_collection_gui/MyoArm.h)
set(qt_srcs src/imu_vicon_arm_calibration_Widget_test.ui src/mainwindow.cpp)
set(qt_hdrs include/imu_vicon_arm_calibration_gui/mainwindow.h)

qt5_wrap_cpp(qt_moc_srcs ${qt_hdrs})
qt5_wrap_ui(uihdr src/imu_vicon_arm_calibration_Widget_test.ui)

add_executable(test_1 src/test.cpp)
target_link_libraries(test_1 ${Boost_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets Qt5::Network Qt5::Gui Qt5::PrintSupport)

add_executable(imu_vicon_arm_calibration_gui ${qt_srcs} ${qt_hdrs} src/main.cpp)
target_link_libraries(imu_vicon_arm_calibration_gui ${Boost_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets Qt5::Network Qt5::Gui Qt5::PrintSupport)


#add_executable(imu_sync_pub_node src/imu_sync_data_pub.cpp)
#target_link_libraries(imu_sync_pub_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets Qt5::Network Qt5::Gui Qt5::PrintSupport)

#add_executable(fake_sub1 src/test_sub1.cpp)
#target_link_libraries(fake_sub1 ${Boost_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets Qt5::Network Qt5::Gui Qt5::PrintSupport)
#add_executable(fake_sub2 src/test_sub2.cpp)
#target_link_libraries(fake_sub2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets Qt5::Network Qt5::Gui Qt5::PrintSupport)

