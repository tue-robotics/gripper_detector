cmake_minimum_required(VERSION 2.8.3)
project(gripper_detector)

SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )

find_package(catkin REQUIRED COMPONENTS
roscpp
cv_bridge
rgbd
tf
visp_bridge
aruco
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#Library gathering libauto_tracker used by all nodes.
#add_library(visp_auto_tracker_cmd_line
#  include/cmd_line/cmd_line.cpp
#  include/cmd_line/cmd_line.h
#)
#target_link_libraries(visp_auto_tracker_cmd_line
#  ${resource_retriever_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES})

#add_library(auto_tracker
#  include/libauto_tracker/states.hpp
#  include/libauto_tracker/events.h
#  include/libauto_tracker/tracking.h
#  include/libauto_tracker/tracking.cpp
#  include/libauto_tracker/logfilewriter.hpp
#  include/libauto_tracker/threading.h
#  include/libauto_tracker/threading.cpp
#)

#target_link_libraries(auto_tracker
#  visp_auto_tracker_cmd_line
#  ${resource_retriever_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

file(GLOB_RECURSE HEADER_FILES include/*.h)

add_executable( gripper_detector
    src/gripper_detector.cpp
    ${HEADER_FILES}
)
target_link_libraries(gripper_detector ${catkin_LIBRARIES})


