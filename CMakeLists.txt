cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(i3dr_ur10)

find_package(catkin REQUIRED COMPONENTS
                roscpp std_msgs tf rospy std_srvs cv_bridge image_transport message_filters message_generation
            ) 
find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    #INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs
	LIBRARIES ${PROJECT_NAME}
)

add_executable(
	i3dr_hand_eye src/i3dr_hand_eye.cpp
)
target_link_libraries(
	i3dr_hand_eye
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)