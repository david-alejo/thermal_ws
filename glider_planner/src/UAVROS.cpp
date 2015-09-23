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


#include "glider_planner/UAVROS.h"
#include <mavros/Waypoint.h>
#include "UAVFlightPlan/UAVFlightPlan.h"
#include <iostream>
#include <boost/function.hpp>
#include <mavlink/v1.0/ardupilotmega/mavlink.h>
#include <functions/functions.h>

using namespace std;
using UAVFlightPlan::EarthLocation;

namespace glider_planner {

UAV* UAVROS::clone() const
{
  UAV *new_uav = new UAVROS();
  UAVROS &uav_real = dynamic_cast<UAVROS &>(*new_uav);
  
  UAV::copy_parameters(*new_uav);
  
  ros::NodeHandle n;
  
  uav_real.flight_plan_filename = flight_plan_filename;
  uav_real.cur_wp_service = cur_wp_service;
  uav_real.param_service = param_service;
  uav_real.position_filename = position_filename;
  uav_real.battery_topic = battery_topic;
  uav_real.VFR_topic = VFR_topic;
  uav_real.center = center;
  uav_real.waypoint_client = n.serviceClient<mavros::WaypointPush>(flight_plan_filename);
  uav_real.cur_wp_client = n.serviceClient<mavros::WaypointSetCurrent &>(cur_wp_service);
  uav_real.param_client = n.serviceClient<mavros::ParamSet &>(param_service);
  boost::function<void (const sensor_msgs::NavSatFix &)> fun = 
			      boost::bind(&UAVROS::stateCallback, dynamic_cast<UAVROS*>(new_uav), _1);
  uav_real.state_subscriber = n.subscribe<sensor_msgs::NavSatFix &>(position_filename, 1, fun);
  boost::function<void (const mavros::BatteryStatus &)> fun_bat = 
			      boost::bind(&UAVROS::batteryCallback, dynamic_cast<UAVROS*>(new_uav), _1);
  boost::function<void (const mavros::VFR_HUD &)> fun_vfr = 
			      boost::bind(&UAVROS::VFRCallback, dynamic_cast<UAVROS*>(new_uav), _1);
  uav_real.battery_subscriber = n.subscribe<mavros::BatteryStatus &>(battery_topic, 1, fun_bat);
  uav_real.VFR_subscriber =n.subscribe<mavros::VFR_HUD &>(VFR_topic,1,fun_vfr);
  
  uav_real.qgc_version = qgc_version;
  
  uav_real.min_dist = min_dist;
  uav_real.thr_thermal = thr_thermal;
  uav_real.thr_gliding = thr_gliding;
  uav_real.thr_approach = thr_approach;
  uav_real.throttle_param_name = throttle_param_name;
  uav_real.throttle_percentage = throttle_percentage;
  uav_real.max_retries = max_retries;
  
  return new_uav;
}

UAV* UAVROS::createFromBlock(ParseBlock& block)
{
  if (!ros::isStarted()) {
    cerr << "UAVROS::createFromBlock --> ROS must be initialized before creating a UAVROS\n";
  }
  if ( init(block)) {
    return clone();
  } else {
    cerr << "UAVROS::createFromBlock --> Could not initialize the UAV from file\n";
    return NULL;
  }
}

void UAVROS::actualize()
{
  ros::spinOnce();
}

void UAVROS::setFlightPlan(const simulator::FlightPlan& new_plan)
{
  simulator::FlightPlan old_plan = active_plan;
  UAV::setFlightPlan(new_plan);
  UAVFlightPlan::UAVFlightPlan uav_plan(new_plan, center, true,  new_plan.getReachAltitude());
  
  UAVFlightPlan::UAVFlightPlan::iterator it = uav_plan.begin();
  for (uint i = 0; i < new_plan.size() && it != uav_plan.end(); it++, i++) {
    it->setClimbRate(new_plan.getClimbRate(i));
    cout << "Climb rate " << i  << " = " << new_plan.getClimbRate(i) << "\t";
  }
  cout << endl;

  // Check if the plan is different 
  bool in_updraft = false;
  
  if (new_plan.isDifferent(old_plan, 1, min_dist) ) {
    bool actualize = true;
    for (uint i = 0; i < old_plan.size() && actualize; i++) {
      if (old_plan.getClimbRate(i) > 0.1 && old_plan.at(i).z * 0.85 < current_location.at(2)) {
	actualize = false;
      }
    }
    
    if (actualize) {
      
      // Publish the plan to the mavros module!
      mavros::WaypointPush srv;
      srv.request.waypoints = toMavRos(uav_plan, in_updraft).waypoints;
      mavros::WaypointSetCurrent srv_current;
      srv_current.request.wp_seq = 0;
      uint max_req = max_retries;
      while ( (!waypoint_client.call(srv) || !srv.response.success) && max_req > 0) {
	ROS_ERROR("UAVROS::setFlightPlan --> Could not change the flight plan retrying.\n");
	max_req--;
      } 
      if (srv.response.success !=0 ) {
	active_plan = new_plan;
      } else {
	ROS_ERROR("UAVROS::setFlightPlan --> Could not change the flight plan.\n");
      }
      ROS_INFO("Waypoint service call. Result: %d. Ways transferred:  %d", srv.response.success, srv.response.wp_transfered);
      max_req = max_retries;
      while ( srv.response.success != 0 && (!cur_wp_client.call(srv_current) || srv_current.response.success == 0) && max_req > 0) {
	ROS_ERROR("UAVROS::setFlightPlan --> Could not change the current waypoint. Retrying");
	max_req--;
      }
      if (srv_current.response.success != 0) {
	ROS_INFO("Waypoint set current call. Switched to waypoint 1.");
      } else {
	ROS_ERROR("UAVROS::setFlightPlan --> Could not change the current waypoint.");
      }
      
      if (emergency) {
	setThrMaxRetry(thr_thermal);
      } else if (in_updraft) {
	setThrMaxRetry(thr_thermal);
      } else {
	setThrMaxRetry(thr_gliding);
      }
    }
    
    // TODO: change flight mode? Verify in simulation
  }
}

bool UAVROS::setThrMaxRetry(double thr)
{
  uint max_req = max_retries;
  
  if (throttle_percentage != thr) {
    ROS_INFO("Changing THR_MAX to %f", thr);
    mavros::ParamSet srv_param;
    srv_param.request.param_id = throttle_param_name;
    srv_param.request.value.integer = (uint64_t)thr;
    srv_param.request.value.real = thr;
    
    while ( (!param_client.call(srv_param) || ! srv_param.response.success != 0) && max_req > 0) {
      ROS_ERROR("UAVROS::setThrMaxRetry --> Could not change the throttle max. Retrying");
      max_req--;
    }
    if (max_req != 0) {
      ROS_INFO("UAVROS::setThrMaxRetry --> Throttle changed to: %f", thr);
      throttle_percentage = thr;
    }
  }
  
  
  return max_req != 0;
}


Checker* UAVROS::getChecker()
{
  Checker *ret = UAV::getChecker();
  
  
  ret->addProperty("position_filename", new NTimes(1));
  ret->addProperty("flight_plan_filename", new NTimes(1));
  ret->addProperty("cur_wp_service", new NTimes(1));
  ret->addProperty("center", new NTimes(1));
  ret->addProperty("param_service", new NTimes(1));
  ret->addProperty("battery_topic", new NTimes(1));
  ret->addProperty("VFR_topic",new NTimes(1));
  return ret;
}

bool UAVROS::init(ParseBlock& block)
{
  bool ret_val;
    
  try {
    ret_val = UAV::init(block);
    ros::NodeHandle n;
    
    if (ret_val) {
      position_filename = block("position_filename").value;
      flight_plan_filename = block("flight_plan_filename").value;
      cur_wp_service = block("cur_wp_service").value;
      param_service = block("param_service").value;
      qgc_version = 120;
      battery_topic = block("battery_topic").value;
      VFR_topic = block("VFR_topic").value;
      
      if (block.hasProperty("qgc_version")) {
	qgc_version = block("qgc_version").as<int>();
      }
      
      if (block.hasProperty("throttle_param_name")) {
	throttle_param_name = block("throttle_param_name").value;
      } else {
	throttle_param_name = "THR_MAX";
      }
      if (block.hasProperty("max_retries")) {
	max_retries = block("max_retries").as<uint>();
      } else {
	max_retries = 3;
      }
      
      // Throttle parameters
      if (block.hasProperty("throttle_approach")) {
	thr_approach = block("throttle_approach").as<double>();
      } else {
	thr_approach = 20.0;
      }
      if (block.hasProperty("throttle_thermal")) {
	thr_thermal = block("throttle_thermal").as<double>();
      } else {
	thr_thermal = 50.0;
      }
      if (block.hasProperty("throttle_gliding")) {
	thr_gliding = block("throttle_gliding").as<double>();
      } else {
	thr_gliding = 0.0;
      }
      throttle_percentage = 50.0; // At first it is in its normal value
      
      if (block.hasProperty("min_dist")) {
	min_dist = block("min_dist").as<double>();
      } else {
	min_dist = 50.0;
      }
      
      // ROS connections
      waypoint_client = n.serviceClient<mavros::WaypointPush>(flight_plan_filename);
      cur_wp_client = n.serviceClient<mavros::WaypointSetCurrent>(cur_wp_service);
      param_client = n.serviceClient<mavros::ParamSet &>(param_service);
      boost::function<void (const sensor_msgs::NavSatFix &)> fun = 
			      boost::bind(&UAVROS::stateCallback, this, _1);
      boost::function<void (const mavros::BatteryStatus &)> fun_bat = 
			      boost::bind(&UAVROS::batteryCallback, this, _1);
      boost::function<void (const mavros::VFR_HUD &)> fun_vfr = 
			      boost::bind(&UAVROS::VFRCallback, this, _1);
      state_subscriber = n.subscribe<sensor_msgs::NavSatFix &>(position_filename, 1, fun);
      battery_subscriber = n.subscribe<mavros::BatteryStatus &>(battery_topic, 1, fun_bat);
      VFR_subscriber =n.subscribe<mavros::VFR_HUD &>(VFR_topic,1,fun_vfr);
      std::vector<double> vec;
      vec = block("center").as<vector<double> >();
      EarthLocation e(vec.at(0),vec.at(1),vec.at(2));
      center = e;
      received_data = false;
    }
    
  } catch (exception &e) {
	  cerr << "UAVROS::init --> Error loading data. Exception: " << e.what() << "\n";
    ret_val = false;
  }
  
  return ret_val;
}

void UAVROS::stateCallback(const sensor_msgs::NavSatFix& state)
{
  received_data = true;
  EarthLocation e(state.latitude,
                  state.longitude,
		 state.altitude);
  current_location = e.toRelative(center);
  functions::FormattedTime t;
  t.getTime();
  current_location.push_back(t - init_time);
   trajectory.push_back(current_location);
}

void UAVROS::batteryCallback(const mavros::BatteryStatus& battery)
{
  this->battery = battery;
}
void UAVROS::VFRCallback(const mavros::VFR_HUD& state)
{
  this->status=state;
}



mavros::WaypointList UAVROS::toMavRos(const UAVFlightPlan::UAVFlightPlan &fp, bool &in_updraft) const
{
  mavros::WaypointList ret;
  in_updraft = false;
  UAVFlightPlan::UAVFlightPlan::const_iterator it = fp.begin();
  mavros::Waypoint w_mav;
  for (unsigned int way_id = 0; it != fp.end(); way_id++, it++) {
    const UAVFlightPlan::UAVWaypoint &w = *it;
  
    w_mav.frame = 0; // GLOBAL FRAME RELATIVE ALTITUDE
    w_mav.x_lat = w.getLatitude();
    w_mav.y_long = w.getLongitude();
    w_mav.z_alt = w.getAltitude();
    w_mav.param1 = w_mav.param2 = w_mav.param3 = w_mav.param4 = 0.0;
    w_mav.autocontinue = 1;
    if (!w.getReachAltitude()) {
      w_mav.command = MAV_CMD_NAV_WAYPOINT;
    } else {
      in_updraft = true;
      // First the condition
      w_mav.param1 = w.getClimbRate() * 100.0;
      w_mav.command = MAV_CMD_CONDITION_CHANGE_ALT;
      ret.waypoints.push_back(w_mav);
      way_id++;
      // Then the jump
      mavros::Waypoint w_mav_jump = jumpCommand(way_id + 2, 10);
      ret.waypoints.push_back(w_mav_jump);
      way_id++;
      // Last the loiter
//       w_mav.param1 = it->getClimbRate();
      w_mav.command = MAV_CMD_NAV_LOITER_UNLIM;
    }
    ret.waypoints.push_back(w_mav);
  }
  
  if (fp.getLoop() && fp.size() > 0) {
    mavros::Waypoint w_mav_jump = jumpCommand(0, 10);
    ret.waypoints.push_back(w_mav_jump);
    
    mavros::Waypoint w_mav_finish = w_mav; // Include a dummy waypoint
    ret.waypoints.push_back(w_mav_finish);
  }
  
  return ret;
}

mavros::Waypoint UAVROS::jumpCommand(uint target_id, uint n_times) const
{
  mavros::Waypoint w;
  
  w.command = UAVFlightPlan::UAVWaypoint::MAV_DO_JUMP;
  w.param1 = target_id;
  w.param2 = n_times;
  w.param3 = w.param4 = w.x_lat = w.y_long = w.z_alt = 0.0;
  w.autocontinue = 1;
  
  return w;
}


}