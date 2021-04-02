cmake_minimum_required(VERSION 3.0.2)
project(inrol_iq)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(Threads REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/CANOpen
  src/iq
)

add_executable(switch_listener src/switch_listener.cpp)
target_link_libraries(switch_listener ${catkin_LIBRARIES})

add_executable(iq_controller
	src/iq_controller.cpp
  src/iq/iq.cpp
	src/CANOpen/CANOpen.c
	src/CANOpen/CANOpen_hw_appl.c
)
target_link_libraries(iq_controller ${catkin_LIBRARIES})
target_link_libraries(iq_controller ${CMAKE_THREAD_LIBS_INIT})

add_executable(joy_controller src/joy_controller.cpp)
target_link_libraries(joy_controller ${catkin_LIBRARIES})