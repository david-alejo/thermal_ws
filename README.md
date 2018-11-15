# thermal_ws

Source code related to developments done during the PhD. Thesis of David Alejo

These are related to the Chapter 6:  A Distributed System for Cooperative Static Soaring

In order to make it work, some dependencies have to be met:

 - Ros indigo or jade should be installed
 - Mavros package of ROS
 - It is meant to be a catkin workspace which should be initialized: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
 - External libraries:
      - Qt 4
      - Marble https://marble.kde.org/ 
      - Libraries functions, simulator, sparser, UAVFlightPlan and graph of my "resolution" repository

Configuring the marble maps:

- Marble is both a GUI for maps visualization and a library for including a map visualization in an external application. Several maps can be installed by cloning the repositories inside https://gitlab.com/groups/marble-restricted-maps into ~/.local/share/marble/maps/earth/

For example:

> cd ~/.local/share/marble/maps/earth/

> git clone https://gitlab.com/marble-restricted-maps/virtualearth.git

Please test that the maps are available in the Marble Application and displaying ok.
