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

#ifndef UAVREAL_H
#define UAVREAL_H

#include "UAV.h"
#include <UAVFlightPlan/earthlocation.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

namespace glider_planner {

class UAVReal : public UAV
{

public:
    virtual UAV* clone() const;
    virtual UAV* createFromBlock(ParseBlock& block);
    virtual void actualize();
    virtual void setFlightPlan(const simulator::FlightPlan& new_plan);
    virtual Checker* getChecker();
    
    virtual std::string getType() const {return "Real";}

protected:
    bool received_data;
  
    virtual bool init(ParseBlock& block);
    
    void stateCallback(const geometry_msgs::PoseStampedConstPtr& pose);
    
    geometry_msgs::PoseStamped current_pose;
    
    ros::Subscriber state_subscriber;
    ros::Publisher flight_plan_publisher;
    
    std::string flight_plan_filename; // Used for flight plan topic
    std::string position_filename; // Used for state topic
    int qgc_version; // Version fo the QGC plan file that is generated
    UAVFlightPlan::EarthLocation center;
    ros::NodeHandle n;
};

}

#endif // UAVREAL_H
