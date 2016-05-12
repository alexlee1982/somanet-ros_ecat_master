cmake_minimum_required(VERSION 2.8.3)
project(ros_ecat_master)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
FILES
    MoveJoint.msg
    MotorState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES sncn_ctrlproto
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

#specify link directories
link_directories(
/opt/etherlab/lib/
)

#specify include directories
include_directories( include
  include/config_motor_master
  include/inc_ctrlproto-common
  include/lib_linux_ctrlproto
  include/lib_linux_motor_drive
  include/module_ethercat/include
  include/module_misc/include
  include/module_motorcontrol/include
  include/module_profile/include
  include/module_pwm_symmetrical/common
  include/module_gpio/include
  include/module_hall/include
  include/module_ams_rotary_sensor/include
  include/module_biss/include
  include/module_qei/include
  /opt/etherlab/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#Add ctrlproto as library
add_library(sncn_ctrlproto
   include/lib_linux_ctrlproto/ctrlproto_m.h
   include/lib_linux_ctrlproto/motor_define.h
   include/lib_linux_ctrlproto/xccompat.h
   include/module_ethercat/include/canod.h
   src/module_ethercat/src/canod_datatypes.h
   src/lib_linux_ctrlproto/ctrlproto_m.c
)

#Add motor drive library
add_library(linux_motor_drive
   include/lib_linux_motor_drive/drive_function.h
   src/lib_linux_motor_drive/drive_function.c
)

#Add profile generators as a library
add_library(profilers
   src/module_profile/profile_linear_mode/profile_linear.c
   src/module_profile/profile_velocity_mode/profile_velocity.c
   src/module_profile/profile_position_mode/profile_position.c
   src/module_profile/quick_stop_mode/profile_position_quick-stop.c
   src/module_profile/quick_stop_mode/profile_velocity_quick-stop.c
)

#Link ethercat to ctrlproto
target_link_libraries(sncn_ctrlproto ethercat)

#Define node
add_executable(ros_master_app src/main.cpp)
add_executable(control_base src/youbot_joy.cpp)

#Link libraries to app
target_link_libraries(ros_master_app 
ethercat 
sncn_ctrlproto
linux_motor_drive
profilers
${catkin_LIBRARIES}
)

target_link_libraries(control_base 
${catkin_LIBRARIES}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(ros_ecat_master
#   src/${PROJECT_NAME}/ros_ecat_master.cpp
# )

## Declare a cpp executable
# add_executable(ros_ecat_master_node src/ros_ecat_master_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ros_ecat_master_node ros_ecat_master_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ros_ecat_master_node
#   ${catkin_LIBRARIES}
# )




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ros_ecat_master ros_ecat_master_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_ecat_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
