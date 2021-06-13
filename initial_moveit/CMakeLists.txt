cmake_minimum_required(VERSION 3.0.2)
project(initial_moveit)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
)


# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES initial_moveit
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(bit_move src/bit_move.cpp)
target_link_libraries(bit_move
  ${catkin_LIBRARIES}
)

add_executable(trajectory_move src/trajectory_move.cpp)
target_link_libraries(trajectory_move
  ${catkin_LIBRARIES}
)