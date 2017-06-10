cmake_minimum_required(VERSION 2.8.3)
project(tangible_programming)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rapid_perception
  ar_track_alvar_msgs
  roscpp
  message_generation
  geometry_msgs
  sensor_msgs
  moveit_msgs
  tangible_msgs
  std_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
#add_message_files(
#  FILES
#  Message1.msg
#  Message2.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  Service1.srv
#  Service2.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  Pckages containing msgs e.g.
#  std_msgs 
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES tangible_main
  CATKIN_DEPENDS rapid_perception roscpp message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Library files
## ----------------
## Declare a C++ library
## Add cmake target dependencies of the library
##    as an example, code may need to be generated before libraries
##    either from message generation or dynamic reconfigure

add_library(tangible_tag_extractor src/tag_extractor.cpp)
add_dependencies(tangible_tag_extractor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_tag_extractor 
  ${catkin_LIBRARIES}
)

add_library(tangible_scene_parser src/scene_parser.cpp)
add_dependencies(tangible_scene_parser
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_scene_parser ${catkin_LIBRARIES})

add_library(tangible_compiler src/compiler.cpp)
add_dependencies(tangible_compiler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_compiler ${catkin_LIBRARIES})

add_library(tangible_gripper_marker src/gripper_marker.cpp)
add_dependencies(tangible_gripper_marker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_gripper_marker
  ${catkin_LIBRARIES}
)

add_library(tangible_grasp_generator src/grasp_generator.cpp)
add_dependencies(tangible_grasp_generator
  tangible_gripper_marker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_grasp_generator 
  tangible_gripper_marker
  ${catkin_LIBRARIES}
)

add_library(tangible_object_matching src/object_matching.cpp)
add_dependencies(tangible_object_matching
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_object_matching ${catkin_LIBRARIES})

add_library(tangible_grasp_test src/grasp_test.cpp)
add_dependencies(tangible_grasp_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_grasp_test ${catkin_LIBRARIES})

add_library(tangible_operation src/operation.cpp)
add_dependencies(tangible_operation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_operation ${catkin_LIBRARIES})

add_library(tangible_pick_and_place src/pick_and_place.cpp)
add_dependencies(tangible_pick_and_place
  tangible_operation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_pick_and_place 
  tangible_operation
  ${catkin_LIBRARIES}
)

add_library(tangible_execution_manager src/execution_manager.cpp)
add_dependencies(tangible_execution_manager
  tangible_operation
  tangible_pick_and_place
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_execution_manager
  tangible_operation
  tangible_pick_and_place  
  ${catkin_LIBRARIES}
)



## Executable files
## ----------------
## Declare a C++ executable
## Add cmake target dependencies of the executable (same as for the library above)
## Specify libraries to link a library or executable target against

add_executable(object_processing_node src/object_processing_node.cpp)
add_dependencies(object_processing_node
  tangible_scene_parser
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_processing_node 
  tangible_scene_parser
  ${catkin_LIBRARIES}
)

add_executable(block_parsing_node src/block_parsing_node.cpp)
add_dependencies(block_parsing_node
  tangible_tag_extractor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(block_parsing_node 
  tangible_tag_extractor
  ${catkin_LIBRARIES}
)

add_executable(program_execution_node src/execution_manager_node.cpp)
add_dependencies(program_execution_node
  tangible_operation
  tangible_pick_and_place
  tangible_execution_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(program_execution_node
  tangible_operation
  tangible_pick_and_place 
  tangible_execution_manager
  ${catkin_LIBRARIES}
)

add_executable(compiler_node src/compiler_node.cpp)
add_dependencies(compiler_node
  tangible_compiler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(compiler_node 
  tangible_compiler
  ${catkin_LIBRARIES}
)

add_executable(object_matching_node src/object_matching_node.cpp)
add_dependencies(object_matching_node
  tangible_object_matching
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_matching_node 
  tangible_object_matching
  ${catkin_LIBRARIES}
)

add_executable(grasp_test_node src/grasp_test_node.cpp)
add_dependencies(grasp_test_node
  tangible_grasp_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(grasp_test_node 
  tangible_grasp_test
)

add_executable(grasp_generator_node src/grasp_generator_node.cpp)
add_dependencies(grasp_generator_node
  tangible_grasp_generator
  tangible_gripper_marker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(grasp_generator_node 
  tangible_gripper_marker
  tangible_grasp_generator
  ${catkin_LIBRARIES}
)

add_executable(fake_compilation_node src/program_srv.cpp)
add_dependencies(fake_compilation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fake_compilation_node 
  ${catkin_LIBRARIES}
)

add_executable(fake_scene_processing_node src/scene_srv.cpp)
add_dependencies(fake_scene_processing_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fake_scene_processing_node 
  ${catkin_LIBRARIES}
)

add_executable(fake_instruction_matching_node src/match_srv.cpp)
add_dependencies(fake_instruction_matching_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fake_instruction_matching_node 
  ${catkin_LIBRARIES}
)

add_executable(fake_grasp_generation_node src/grasp_srv.cpp)
add_dependencies(fake_grasp_generation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fake_grasp_generation_node 
  ${catkin_LIBRARIES}
)

add_executable(fake_release_generation_node src/release_srv.cpp)
add_dependencies(fake_release_generation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fake_release_generation_node 
  ${catkin_LIBRARIES}
)

add_executable(fake_move_arm_node src/move_srv.cpp)
add_dependencies(fake_move_arm_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fake_move_arm_node 
  ${catkin_LIBRARIES}
)

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
# install(TARGETS tangible_main tangible_main_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tangible_main.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
