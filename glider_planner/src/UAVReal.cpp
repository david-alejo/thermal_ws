/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "glider_planner/UAVReal.h"
#include "UAVFlightPlan/UAVFlightPlan.h"
#include <iostream>
#include <boost/function.hpp>

using namespace std;
using UAVFlightPlan::EarthLocation;

namespace glider_planner {

UAV* UAVReal::clone() const
{
  UAV *new_uav = new UAVReal();
  UAVReal &uav_real = dynamic_cast<UAVReal &>(*new_uav);
  
  UAV::copy_parameters(*new_uav);
  
  ros::NodeHandle n;
  
  uav_real.flight_plan_filename = flight_plan_filename;
  uav_real.position_filename = position_filename;
  uav_real.center = center;
  uav_real.flight_plan_publisher = n.advertise<std_msgs::String>(flight_plan_filename, 1);
  boost::function<void (const geometry_msgs::PoseStampedConstPtr &)> fun = 
			      boost::bind(&UAVReal::stateCallback, dynamic_cast<UAVReal*>(new_uav), _1);
  uav_real.state_subscriber = n.subscribe<geometry_msgs::PoseStampedConstPtr &>(position_filename, 1, fun);
  uav_real.qgc_version = qgc_version;
  
  return new_uav;
}

UAV* UAVReal::createFromBlock(ParseBlock& block)
{
  if (!ros::isStarted()) {
    cerr << "UAVReal::createFromBlock --> ROS must be initialized before creating a UAVReal\n";
  }
  if ( init(block)) {
    return clone();
  } else {
    cerr << "UAVReal::createFromBlock --> Could not initialize the UAV from file\n";
    return NULL;
  }
}

void UAVReal::actualize()
{
  ros::spinOnce();
  if (received_data) {
    EarthLocation e(current_pose.pose.position.x,
                  current_pose.pose.position.y,
		 current_pose.pose.position.z);
    current_location = e.toRelative(center);
  
    cout << "Debug: Obtained location: " << e.toMatlab() << "\tRelative: " << current_location.toString() << endl;
    functions::FormattedTime t;
    t.getTime();
    current_location.push_back(t - init_time);
  
    trajectory.push_back(current_location); 
    curr_time.getTime();
  }
}

void UAVReal::setFlightPlan(const simulator::FlightPlan& new_plan)
{
  simulator::FlightPlan old_plan = active_plan;
  UAV::setFlightPlan(new_plan);
  UAVFlightPlan::UAVFlightPlan uav_plan(new_plan, center);

  // Check if the plan is different 
  if (new_plan.isDifferent(old_plan)) {
    // Publish it!
    std_msgs::String s;
    s.data = uav_plan.toQGCString(qgc_version);
    flight_plan_publisher.publish(s);
  }
}

Checker* UAVReal::getChecker()
{
  Checker *ret = UAV::getChecker();
  
  ret->addProperty("position_filename", new NTimes(1));
  ret->addProperty("flight_plan_filename", new NTimes(1));
  ret->addProperty("center", new NTimes(1));
  
  return ret;
}

bool UAVReal::init(ParseBlock& block)
{
  bool ret_val;
    
  try {
    ret_val = UAV::init(block);
    ros::NodeHandle n;
    
    if (ret_val) {
      position_filename = block("position_filename").as<string>();
      flight_plan_filename = block("flight_plan_filename").as<string>();
      qgc_version = 120;
      if (block.hasProperty("qgc_version")) {
	qgc_version = block("qgc_version").as<int>();
      }
      
      flight_plan_publisher = n.advertise<std_msgs::String>(flight_plan_filename, 1);
      boost::function<void (const geometry_msgs::PoseStampedConstPtr &)> fun = 
			      boost::bind(&UAVReal::stateCallback, this, _1);
      state_subscriber = n.subscribe<geometry_msgs::PoseStampedConstPtr &>(position_filename, 1, fun);
      
      std::vector<double> vec;
      vec = block("center").as<vector<double> >();
      EarthLocation e(vec.at(0),vec.at(1),vec.at(2));
      center = e;
      received_data = false;
    }
    
  } catch (exception &e) {
	  cerr << "UAVReal::init --> Error loading data. Exception: " << e.what() << "\n";
    ret_val = false;
  }
  
  return ret_val;
}

void UAVReal::stateCallback(const geometry_msgs::PoseStampedConstPtr& pose)
{
  received_data = true;
  if (pose.get() != NULL) {
    const geometry_msgs::Pose &aux = pose.get()->pose;
//     current_pose.header = pose.get()->header;
//     string s;
//     s.copy(pose.get()->header.frame_id.c_str());
//     current_pose.header.frame_id = s;
    current_pose.header.seq = pose.get()->header.seq;
    current_pose.header.stamp = pose.get()->header.stamp;
    current_pose.pose.orientation.x = aux.orientation.x;
    current_pose.pose.orientation.y = aux.orientation.y;
    current_pose.pose.orientation.z = aux.orientation.z;
    current_pose.pose.orientation.w = aux.orientation.w;
    current_pose.pose.position.x = aux.position.x;
    current_pose.pose.position.y = aux.position.y;
    current_pose.pose.position.z = aux.position.z;
//     current_pose = *(pose.get());
  }
}


}