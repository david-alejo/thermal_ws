cmake_minimum_required(VERSION 2.8.3)
project(glider_planner)

SET(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mavros
)

find_package(Boost REQUIRED COMPONENTS thread)


SET (LIB boost_thread sparser simulator functions UAVFlightPlan boost_regex functions glider_planner)
SET(LIBXML xml++-2.6)
SET(LIBKML kmldom kmlconvenience kmlengine)
SET(LIBS ${LIB} ${LIBXML} ${LIBKML})
message(${LIBS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES glider_planner
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs mavros
  DEPENDS ${LIBS}
)

###########
## Build ##
###########




## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

set (XML_INCLUDE /usr/include/libxml2 /usr/include/glibmm-2.4 /usr/lib/x86_64-linux-gnu/glibmm-2.4/include /usr/include/sigc++-2.0 /usr/lib/x86_64-linux-gnu/sigc++-2.0/include /usr/include/glib-2.0 /usr/lib/x86_64-linux-gnu/glib-2.0/include /usr/include/libxml++-2.6 /usr/lib/libxml++-2.6/include)  


include_directories(
  ${catkin_INCLUDE_DIRS} include ${XML_INCLUDE}
)

## Declare a cpp library
add_library(glider_planner
  src/URM.cpp src/BasicState.cpp src/Planner.cpp
        src/TimeSlot.cpp src/CompleteSystem.cpp src/UAV.cpp src/SoaringPlanner.cpp src/SoaringPlannerFactory.cpp
        src/ExpandingTreePlanner.cpp src/TreeState.cpp src/ThreadUAV.cpp src/UAVFactory.cpp src/UAVSimulation.cpp src/UAVReal.cpp src/UAVROS.cpp
        src/ThermalDetector.cpp
)
target_link_libraries(glider_planner ${catkin_LIBRARIES} ${LIBS})

## Declare a cpp executable
add_executable(glider_planner_node src/exec/multiuav_main.cpp)
## Specify libraries to link a library or executable target against
message(${catkin_LIBRARIES} )
target_link_libraries(glider_planner_node  ${LIBS} ${catkin_LIBRARIES} glider_planner)

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
# install(TARGETS glider_planner glider_planner_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_glider_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
