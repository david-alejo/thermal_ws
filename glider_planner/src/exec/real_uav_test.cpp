// This file will test the functionality defined in UAVReal.h

#include <simulator/ParticleFactory.h>
#include <functions/functions.h>
#include <functions/RealVector.h>
#include <sparser/all.h>
#include <iostream>
#include <UAVFlightPlan/earthlocation.h>
#include <UAVFlightPlan/UAVFlightPlan.h>
#include <boost/thread.hpp>
#include "src/UAV.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

using namespace simulator;
using namespace std;
using namespace functions;
using UAVFlightPlan::EarthLocation;
using namespace glider_planner;

void planCallback(const std_msgs::String &p);

simulator::FlightPlan fp;
EarthLocation center;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <input filename>\n";
    return -1;
  }
  
  ros::init(argc, argv, "RealUAVTest");
  ros::NodeHandle n;
  
  Checker *check = new Checker;
  
  check->addProperty("flight_plan_file", new NTimes(1));
  check->addProperty("state_file", new NTimes(1));
  check->addProperty("center", new NTimes(1)); // This center has to be the same of the center inidicated for UAVReal
  check->addProperty("T", new NTimes(1));
  check->addBlock("particle", new NTimes(1));
  
  string flight_plan_file;
  string state_file;
  double T;
  Particle *p = NULL;
  
  // Loading data
  try {
    ParseBlock data;
    data.load(argv[1]);
    
    data.checkUsing(check);
    T = data("T").as<double>();
    flight_plan_file = data("flight_plan_file").as<string>();
    state_file = data("state_file").as<string>();
    functions::RealVector v(data("center").as<vector<double> >());
    EarthLocation e(v.at(0), v.at(1), v.at(2));
    center = e;
    ParticleFactory fac;
    p = fac.createFromBlock(data["particle"]);
  } catch (exception &e) {
    cerr << "Exception catched. Content: " << e.what() << endl;
    return -1;
  }
  
  cout << "Center: " << center.toString() << endl;
  cout << "T: " << T << endl;
  
  fp = p->getController()->getFlightPlan();
  
  // Configure ROS Comms
  ros::Subscriber plan_subs = n.subscribe(flight_plan_file, 1, &planCallback);
  ros::Publisher state_pub = n.advertise<geometry_msgs::PoseStamped>(state_file, 1);
  
  // All the data has been loaded, lets go with the main loop
  bool exit = p == NULL;
  UAVFlightPlan::UAVFlightPlan fp_abs;
  int cont_pose = 0;
  ros::Rate r(1/T);
  while (!exit && ros::ok()) {
    p->getController()->setFlightPlan(fp);
    cout << "Flight plan: " << fp.toString() << endl;
    cout << "Location: " << functions::printVector(p->getModel()->getState()) << endl;
    p->runOneStep();
    
    std::vector<double> state = p->getState();
    EarthLocation e(center);
    e.shift(state.at(0), state.at(1), state.at(2));
    e.setRepresentTime(false);
    
    geometry_msgs::PoseStamped p;
    p.header.seq = cont_pose++;
    p.header.frame_id = "global";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = e.getLatitude();
    p.pose.position.y = e.getLongitude();
    p.pose.position.z = e.getAltitude();
    state_pub.publish(p);
    ros::spinOnce();
    r.sleep();
  };
  
  
  return 0;
}

void planCallback(const std_msgs::String& p)
{
  UAVFlightPlan::UAVFlightPlan fp_abs;
  fp_abs.fromQGCString(p.data);
  fp = fp_abs.toRelativeFlightPlan(center);
}

