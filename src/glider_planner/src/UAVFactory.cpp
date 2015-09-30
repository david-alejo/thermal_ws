#include "glider_planner/UAVFactory.h"

#include <iostream>

using namespace std;

namespace glider_planner {

UAV* UAVFactory::create_from_block(ParseBlock& block)
{
  UAV *uav = NULL;
  try {
    std::string type;
    if (block.hasProperty("type")) {
      type = block("type").as<string>();
    }
    if (type == "Simulation") {
      UAVSimulation sim;
      uav = sim.createFromBlock(block);
    } else if (type == "Real") {
      UAVReal real;
      uav = real.createFromBlock(block);
    } else if (type == "ROS") {
      UAVROS ros;
      uav = ros.createFromBlock(block);
    }else {
      cerr << "UAVFactory::create_from_block unrecognized UAV type.\n";
    }
  }catch (exception &e) {
    cerr << "UAVFactory::create_from_block error while loading data.\n";
  }
  
  return uav;
}

UAV* UAVFactory::create_from_file(const string& fileName)
{
  UAV *uav = NULL;
  
  try {
    ParseBlock data;
    data.load(fileName.c_str());
    uav = create_from_block(data);
  } catch (exception &e) {
    cerr << "UAVFactory::create_from_file error while loading data.\n";
  }
    
    
    
  
  return uav;
}


}