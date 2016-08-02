cmake_minimum_required(VERSION 2.8.3)
project(tangible)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rapid_perception
  ar_track_alvar_msgs
  roscpp
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
#  INCLUDE_DIRS include
#  LIBRARIES tangible_main
#  CATKIN_DEPENDS rapid_perception roscpp
#  DEPENDS system_lib
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

add_library(tangible_frame_transformer src/frame_transformer.cpp)
add_dependencies(tangible_frame_transformer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_frame_transformer ${catkin_LIBRARIES})

add_library(tangible_tag src/tag.cpp)
add_dependencies(tangible_tag
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_tag ${catkin_LIBRARIES})

add_library(tangible_tag_extractor src/tag_extractor.cpp)
add_dependencies(tangible_tag_extractor
  tangible_tag
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_tag_extractor 
  tangible_tag
  ${catkin_LIBRARIES}
)

add_library(tangible_scene_parser src/scene_parser.cpp)
add_dependencies(tangible_scene_parser
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_scene_parser ${catkin_LIBRARIES})

add_library(tangible_utils src/utils.cpp)
add_dependencies(tangible_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_utils ${catkin_LIBRARIES})

add_library(tangible_program src/program.cpp)
add_dependencies(tangible_program
  tangible_tag
  tangible_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_program
  tangible_tag
  tangible_utils
  ${catkin_LIBRARIES}
)

add_library(tangible_visualizer src/visualizer.cpp)
add_dependencies(tangible_visualizer
  tangible_program
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tangible_visualizer
  tangible_program
  ${catkin_LIBRARIES}
)

## Executable files
## ----------------
## Declare a C++ executable
## Add cmake target dependencies of the executable (same as for the library above)
## Specify libraries to link a library or executable target against

add_executable(system_main src/system_main.cpp)
add_dependencies(system_main
  tangible_frame_transformer
  tangible_tag
  tangible_tag_extractor
  tangible_scene_parser
  tangible_utils
  tangible_program
  tangible_visualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(system_main
  tangible_frame_transformer
  tangible_tag
  tangible_tag_extractor
  tangible_scene_parser
  tangible_utils
  tangible_program
  tangible_visualizer
  ${catkin_LIBRARIES}
)

add_executable(system_test_grouping src/system_test_grouping.cpp)
add_dependencies(system_test_grouping
  tangible_tag
  tangible_utils
  tangible_program
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(system_test_grouping
  tangible_tag
  tangible_utils
  tangible_program
  ${catkin_LIBRARIES}
)

add_executable(system_test_matching src/system_test_matching.cpp)
add_dependencies(system_test_matching
  tangible_tag
  tangible_scene_parser
  tangible_utils
  tangible_program
  tangible_visualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(system_test_matching
  tangible_tag
  tangible_scene_parser
  tangible_utils
  tangible_program
  tangible_visualizer
  ${catkin_LIBRARIES}
)

add_executable(scene_visualization_main src/scene_visualization_main.cpp)
add_dependencies(scene_visualization_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scene_visualization_main ${catkin_LIBRARIES})

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
