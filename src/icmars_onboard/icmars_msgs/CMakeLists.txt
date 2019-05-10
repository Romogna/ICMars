cmake_minimum_required(VERSION 2.8.3)
project(icmars_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
   FILES
   NamedImage.msg
   TwistDuration.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icmars_msgs
   CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
