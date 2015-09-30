#include "glider_planner/UAV.h"
#include <functions/functions.h>
#include <sstream>

using namespace std;

namespace glider_planner {

bool UAV::init(ParseBlock &uav_block) {
  Checker *check = getChecker();
  bool ret_val = true;
  emergency = false;

  try {
    uav_block.checkUsing(check);
    gamma = uav_block("gamma").as<double>();
    initial_location = uav_block("initial_location").as<vector<double> >();
    current_location = initial_location;
    v_ref = uav_block("v_ref").as<double>();
    min_alt = uav_block("minimum_altitude").as<double>();
    max_alt = uav_block("maximum_altitude").as<double>();
    simulator::FlightPlan fp(uav_block["flight_plan"]);
    active_plan = fp;
    descending_ratio = tan(gamma);
    
    if (uav_block.hasProperty("minimum_altitude_thermal")) {
      min_alt_thermal  = uav_block("minimum_altitude_thermal").as<double>();
    } else {
      min_alt_thermal = min_alt;
    }
    
    if (uav_block.hasProperty("home_location")) {
      home_location = uav_block("home_location").as<vector<double> >();
    } else {
      home_location.clear();
      home_location.push_back(initial_location.at(0));
      home_location.push_back(initial_location.at(1));
    }
    
    if (uav_block.hasProperty("minimum_altitude_coefficient")) {
      min_alt_coeff = uav_block("minimum_altitude_coefficient").as<double>();
    } else {
      min_alt_coeff = 0.5;
    }

    curr_time.getTime();
  } catch (exception &e) {
    cerr << "UAV::init --> Error while loading data. Content:" << e.what() << endl;

    ret_val = false;
  }
  delete check;

  return ret_val;
}

ParseBlock* UAV::toBlock() const
{
  ParseBlock *block = new ParseBlock;
  
  block->setProperty("home_location", home_location.toMatlabString());
  block->setProperty("current_location", current_location.toMatlabString());
  block->setProperty("initial_location", initial_location.toMatlabString());
  block->setProperty("v_ref", functions::numberToString(v_ref));
  block->setProperty("gamma", functions::numberToString(gamma));
  block->setProperty("minimum_altitude", functions::numberToString(min_alt));
  block->setProperty("maximum_altitude", functions::numberToString(max_alt));
  block->setBlock("flight_plan", active_plan.toBlock());
  
  return block;
}


Checker *UAV::getChecker() {
  Checker *check = new Checker;

  check->addProperty("v_ref", new NTimes(1));
  check->addProperty("gamma", new NTimes(1));
  check->addProperty("initial_location", new NTimes(1));
  check->addProperty("minimum_altitude", new NTimes(1));
  check->addProperty("maximum_altitude", new NTimes(1));
  check->addBlock("flight_plan", new NTimes(1));

  return check;
}

string UAV::toString() const {
  ostringstream os;

  os << "V_ref = " << v_ref << "\t";
  os << "Gamma = " << gamma << "\t";
  os << "Initial Location = " << initial_location.toString() << "\t";
  os << "Current Location = " << current_location.toString() << "\t";
  os << "Flight plan = " << active_plan.toString() << "\t";
  os << "Minimum Altitude = " << min_alt << "\t";
  os << "Maximum Altitude = " << max_alt << "\t";
  

  return os.str();
}

double UAV::getAscendingRate(double wind_speed) const {
  return wind_speed - v_ref * tan(gamma);
}

bool UAV::exportTrajectory(const string& filename) const
{
  ostringstream os;
  
  for (unsigned int i = 0; i < trajectory.size(); i++) {
    os << trajectory.at(i).toMatlabString() << endl;
  }
  
  return functions::writeStringToFile(filename, os.str());
}

UAV::~UAV()
{
  trajectory.clear();
  current_location.clear();
  initial_location.clear();
  
}

functions::Point3D UAV::getArrivalTimeAltitude(const functions::RealVector& pos_2d, const functions::RealVector& destination, const double& h0, functions::FormattedTime &t0) const
{
  functions::Point3D p;
  double distance = pos_2d.distance(destination);
  p.x = destination.at(0);
  p.y = destination.at(1);
  p.z = h0 - distance * descending_ratio;
  t0 = t0 + distance/v_ref;
  
  return p;
}

void UAV::copy_parameters(UAV& uav) const
{
  uav.active_plan = active_plan;
  uav.curr_time = curr_time;
  uav.current_location = current_location;
  uav.descending_ratio = descending_ratio;
  uav.gamma = gamma;
  uav.id = id;
  uav.initial_location = initial_location;
  uav.max_alt = max_alt;
  uav.min_alt = min_alt;
  uav.v_ref = v_ref;
  uav.home_location = home_location;
  uav.min_alt_coeff = min_alt_coeff;
}

void UAV::setFlightPlan(const simulator::FlightPlan& new_plan)
{
  simulator::FlightPlan old_plan = active_plan;
  active_plan = new_plan;
}

bool UAV::exportCumulativePlan(const string& filename) const
{
  bool ret_val = true;
  
  ret_val = result.toQGCFile(filename);
  
  return ret_val;
}


}