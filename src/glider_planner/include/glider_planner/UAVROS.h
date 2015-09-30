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

// TODO: toBlock

#ifndef UAVROS_H
#define UAVROS_H

#include "UAV.h"
#include <UAVFlightPlan/earthlocation.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/VFR_HUD.h>

namespace glider_planner {

class UAVROS : public UAV
{

public:
    virtual UAV* clone() const;
    virtual UAV* createFromBlock(ParseBlock& block);
    virtual void actualize();
    virtual void setFlightPlan(const simulator::FlightPlan& new_plan); // Will send a goto message with the second WP!
    virtual Checker* getChecker();
    virtual double getVoltage()const{
      return  battery.voltage ;
    };
    virtual double getPercent()const{
      return battery.remaining*100.0; 
    };
    virtual double getAirSpeed()const{
      return status.airspeed;
    };
    virtual double getAscRate()const{
      return status.climb;
    };
    virtual double getThrottle()const{
      return status.throttle*100.0;
    };
    virtual double getAltitude()const{
      return status.altitude;
    };
    virtual std::string getType() const {return "ROS";}
    
    bool setThrMaxRetry(double thr);

protected:
    bool received_data;
  
    virtual bool init(ParseBlock& block);
    
    void stateCallback(const sensor_msgs::NavSatFix& pose);
    void batteryCallback(const mavros_msgs::BatteryStatus& battery);
    void VFRCallback(const mavros_msgs::VFR_HUD& state);
    
    mavros_msgs::WaypointList toMavRos(const UAVFlightPlan::UAVFlightPlan &fp, bool &in_updraft) const;
    
    mavros_msgs::Waypoint jumpCommand(uint target_id, uint n_times) const;
    
    
    
    // Middleware ROS
    ros::Subscriber state_subscriber;
    ros::ServiceClient waypoint_client;
    ros::ServiceClient cur_wp_client;
    ros::ServiceClient param_client;
    ros::Subscriber battery_subscriber;
    ros::Subscriber VFR_subscriber;
//     ros::ServiceClient flight_mode_client;
    
    std::string flight_plan_filename; // Used for flight plan topic
    std::string position_filename; // Used for state topic
    std::string cur_wp_service; // 
    std::string param_service; // Used for changing the max_throttle
    std::string throttle_param_name;
    std::string battery_topic;
    std::string VFR_topic;
//     std::string flight_mode_change_service;  TODO: IS IT NECESSARY TO CHANGE THE FLIGHT MODE?
    int qgc_version; // Version fo the QGC plan file that is generated
    UAVFlightPlan::EarthLocation center;
    ros::NodeHandle n;
    double throttle_percentage, thr_thermal, thr_gliding, thr_approach;
    double min_dist;
    uint max_retries;
    mavros_msgs::BatteryStatus battery; // Last received battery status
    mavros_msgs::VFR_HUD status;
};

}

#endif // UAVROS_H
