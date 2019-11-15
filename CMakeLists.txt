cmake_minimum_required(VERSION 2.8.3)
project(tmapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
#    qt_build
    message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost
        REQUIRED
        COMPONENTS
        system
        )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


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
add_message_files(
    FILES
#    NewNodeMsg.msg
#    LeaveNode.msg
#    ImageExract.msg
)
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
    FILES
#    SaveMap.srv
#    GetMaps.srv
#    PathPlanning.srv
#    NextStepOfPath.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

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
#  LIBRARIES topology_map
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
#rosbuild_prepare_qt4 (QtCore QtGui) # Add the appropriate components to the component list here

#set(QT Core Gui Widgets Network DBus Sql)
#find_package(Qt5 REQUIRED ${QT})
#find_package(Qt5Core)
#find_package(Qt5Widgets)
#find_package(Qt5Gui)
#include_directories(${Qt5Widgets_INCLUDE_DIRS})
#add_definitions(${Qt5Widgets_DEFINITIONS})

find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/topology_map.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

file(GLOB Topo_UI_FORM RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    include/tmapping/UI/*.ui)

file(GLOB Topo_UI_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    include/tmapping/UI/*.h include/tmapping/UI/*.cpp)

file(GLOB_RECURSE TopoFILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    include/tmapping/*.cpp include/tmapping/*.h)

file(GLOB JsonFILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/json/*.cpp include/json/*.h)

#add_library(topoLayer_UI  ${Topo_UI_SOURCES})
#add_library(topoLayer ${TopoFILES})
add_library(jsonCpp ${JsonFILES})

#add_executable(main src/main.cpp src/MapROSNode.hpp)
#add_executable(TopoUI src/UI_main.cpp)
#add_executable(FakeOB src/FakeOB.cpp)
#add_executable(localController
#    src/grounding/localController.cpp
#    src/grounding/localController.h)

#target_link_libraries(topoLayer jsonCpp)
#target_link_libraries(topoLayer_UI topoLayer ${QT_LIBRARIES})
#target_link_libraries(main topoLayer)
#target_link_libraries(localController topoLayer)
#target_link_libraries(TopoUI topoLayer_UI)
#target_link_libraries(FakeOB topoLayer)
#
#target_link_libraries(main ${catkin_LIBRARIES})
#target_link_libraries(TopoUI ${catkin_LIBRARIES})
#target_link_libraries(FakeOB ${catkin_LIBRARIES})
#target_link_libraries(localController ${catkin_LIBRARIES})
#
#add_dependencies(topoLayer jsonCpp)
#add_dependencies(topoLayer_UI topoLayer)
#add_dependencies(TopoUI topoLayer_UI)
#
#add_dependencies(topoLayer ${PROJECT_NAME}_gencpp)
#add_dependencies(main ${PROJECT_NAME}_gencpp)
#add_dependencies(FakeOB ${PROJECT_NAME}_gencpp)
#add_dependencies(localController ${PROJECT_NAME}_gencpp)

add_executable(main src/main.cpp ${TopoFILES})

#target_link_libraries(main topoLayer)
#target_link_libraries(main topoLayer)