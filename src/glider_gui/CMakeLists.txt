cmake_minimum_required(VERSION 2.8.3)
project(glider_gui)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  glider_planner
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
 LIBRARIES glider_gui
 CATKIN_DEPENDS geometry_msgs glider_planner roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


SET ( QWT_INCLUDE /usr/include/qwt/)
 SET ( QT_MOBILITY_INCLUDE /usr/include/QtMobility/ )
 SET (LIBS qwt boost_thread sparser functions boost_regex simulator UAVFlightPlan marblewidget )

SET( GLIDER_UIS
       ./ui/glider_planner.ui
       ./ui/about_glider.ui
       ./ui/execute_dialog.ui
       ./ui/multi_uav.ui
       ./ui/scenario_dialog.ui
       ./ui/axis_dialog.ui
       ./ui/latex_dialog.ui
       ./ui/uas_widget.ui
  )
  
set(glider_gui_SRCS src/ScenarioLayer.cpp src/CompleteSystemGui.cpp src/LatexDialog.cpp src/AxisDialog.cpp src/ScenarioDialog.cpp src/ExecuteDialog.cpp src/About.cpp src/glider_gui.cpp src/UAS_Widget.cpp)
set(glider_gui_HEAD include/glider_gui/ScenarioLayer.h include/glider_gui/ScenarioDialog.h include/glider_gui/ExecuteDialog.h include/glider_gui/About.h include/glider_gui/glider_gui.h include/glider_gui/UAS_Widget.h)

set (XML_INCLUDE /usr/include/libxml2 /usr/include/glibmm-2.4 /usr/lib/x86_64-linux-gnu/glibmm-2.4/include /usr/include/sigc++-2.0 /usr/lib/x86_64-linux-gnu/sigc++-2.0/include /usr/include/glib-2.0 /usr/lib/x86_64-linux-gnu/glib-2.0/include /usr/include/libxml++-2.6 /usr/lib/libxml++-2.6/include)  



QT4_WRAP_UI( GLIDER_UI_HDRS ${GLIDER_UIS} )
QT4_WRAP_CPP( GEOMAP_SRCS include/glider_gui/ScenarioLayer.h include/glider_gui/ScenarioDialog.h include/glider_gui/glider_gui.h include/glider_gui/UAS_Widget.h include/glider_gui/CompleteSystemGui.h)

qt4_automoc(${glider_gui_SRCS} ${glider_gui_HEAD})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR} ${QWT_INCLUDE} ${QT_MOBILITY_INCLUDE} include/glider_gui ${CMAKE_CURRENT_BINARY_DIR} include ${XML_INCLUDE}
)

## Declare a cpp library
add_library(glider_gui
  ${glider_gui_SRCS} ${GEOMAP_SRCS} ${GLIDER_UI_HDRS}
)


## Declare a cpp executable
add_executable(glider_gui_node src/exec/main.cpp)
target_link_libraries(glider_gui_node glider_gui  ${catkin_LIBRARIES} ${LIBS} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTTEST_LIBRARY} 
)
