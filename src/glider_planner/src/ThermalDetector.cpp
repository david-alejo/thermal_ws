/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  sinosuke <email>

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


#include "glider_planner/ThermalDetector.h"
#include <functions/functions.h>

using namespace glider_planner;
using namespace std;
using simulator::Updraft;
using functions::RealVector;

ThermalDetector::ThermalDetector(ParseBlock& b, int uav_id, URM *urm, UAV *parent)
{
  init(b);
  this->uav_id = uav_id;
  this->urm = urm;
  this->parent = parent;
  
}


void ThermalDetector::init(ParseBlock& block)
{
  Checker *check = getChecker();

  init();
  
  try {
    block.checkUsing(check);
    t_consecutive = block("t_consecutive").as<double>();
    sample_time = block("sample_time").as<double>();
    min_height = block("min_height").as<double>();
    max_height = block("max_height").as<double>();
    log_filename = block("log_filename").as<std::string>();
    usual_radius = block("usual_radius").as<double>();
    if (block.hasProperty("dead_thermal_threshold")) {
      dead_thermal_threshold = block("dead_thermal_threshold").as<double>();
    } else {
      dead_thermal_threshold = -5.0;
    }
    
  } catch (exception &e) {
    std::cerr << "ThermalDetector::init --> error while loading data from block. Content: " << e.what() << "\n";
    delete check;
    throw(e);
  }

  delete check;
}

void ThermalDetector::init()
{
  ascent_detected = false;
  current_detection = NULL;
  parent = NULL;
  dead_thermal_threshold = -5.0;
  last_state.clear();
}


Checker* ThermalDetector::getChecker() const
{
  Checker *check = new Checker;
  
  check->addProperty("min_height", new NTimes(1));
  check->addProperty("t_consecutive", new NTimes(1));
  check->addProperty("sample_time", new NTimes(1));
  check->addProperty("log_filename", new NTimes(1));
  check->addProperty("max_height", new NTimes(1));
  check->addProperty("usual_radius", new NTimes(1));
  
  return check;
}

ThermalDetector::DetectionData::~DetectionData()
{
  states.clear();
  center.clear();
}


void ThermalDetector::thermalDetectionStep(functions::RealVector& vec, bool second_exploration)
{
  // See if the UAV is inside a thermal or inside a possible thermal
  int n_therm = urm->getUpdraftFromLocation(vec);
  int n_poss = urm->isPossibleUpdraft(vec, usual_radius);
  
  if (n_therm != last_therm && n_therm >= 0) {
    descending_in_thermal = false;
    init_therm = vec; // Store the entry state!
  } else if (n_therm == last_therm && last_therm >= 0) {
    if (init_therm.size() > 2 && vec.at(2) - init_therm.at(2) < dead_thermal_threshold) {
      // The thermal has to be considered as dead_thermal_threshold
      urm->deleteUpdraft(n_therm);
      cout << "ThermalDetector::thermalDetectionStep --> deleting updraft in pos: ";
      cout << urm->getUpdrafts().at(n_therm).getLocation().toString() << "(dead). \n";
    } else if (last_state.at(2) - vec.at(2) && n_therm == last_therm && !descending_in_thermal) {
      descending_in_thermal = true;
      init_therm = vec;
    }
  }
  
  if (n_poss != last_poss && n_poss >= 0) {
    init_pos.getTime();
  } else if (n_poss >= 0) {
    functions::FormattedTime t;
    t.getTime();
    if (t - init_pos > 3.0 * usual_radius * parent->v_ref && last_state.at(2) - vec.at(2) < 0.0) {
      cout << "ThermalDetector::thermalDetectionStep --> deleting possible updraft (dead). \n";
      urm->deletePossibleUpdraft(n_poss);
    }
  }
  
  if (last_state.size() < 2) {
    last_state = vec;
    return;
  }
  if (last_state.at(2) <= vec.at(2)) {
    // The glider is ascending
    if (!ascent_detected) {
      // First ascension detected;
      delete current_detection; // Delete the last ascension
      current_detection = new DetectionData;
      current_detection->states.push_back(vec);
      current_detection->first_time.getTime();
      ascent_detected = true;
    } else {
      if (current_detection != NULL) {
	current_detection->states.push_back(vec);
      }
    }
  } else {
    if(ascent_detected) {
      ascent_detected = false;
      
      functions::RealVector &first_state = current_detection->states.at(0);
      // Complete the detection data 
      current_detection->last_time.getTime();
      current_detection->calculateCenter();
      
      // Check if the ascension exceeds a threshold. If not --> ignore it!
      // Deleted the condition of max_height in order to actualize the parameters of an already detected thermal
      if (last_state.at(2) - first_state.at(2) > min_height) { 
	thermalAction(second_exploration); // Do an action if necessary (thermal is still not registered in URM)
      } 
      // append it to the log
      detection_log.push_back(current_detection);
      current_detection = NULL;
    }
  }
  
  // Actualize the last height
  last_state = vec;
}

void ThermalDetector::thermalAction(bool second_exploration)
{
  // Calculate the fields of the new thermal with the data contained in current detection
  if (current_detection != NULL ) { // Make sure that the detection actually exists
    // Get two points that are equidistant to the center in the direction of the perpendicular line
    std::vector<functions::RealVector> &st = current_detection->states; // Shortcut to current_detection->states
    functions::RealVector &pos_thermal = current_detection->center;
    functions::RealVector detection_line_director = st.at(0) - st.at(st.size() - 1);
    detection_line_director.normalize();
    functions::RealVector perpendicular_line_director(2);
    perpendicular_line_director[0] = detection_line_director[1];
    perpendicular_line_director[1] = -detection_line_director[0];
    
    functions::RealVector p1(2);
    functions::RealVector p2(2);
    p1 = pos_thermal + perpendicular_line_director * usual_radius;
    p2 = pos_thermal - perpendicular_line_director * usual_radius;
    
    int n_up = urm->getUpdraftFromLocation(current_detection->center);
    
    if (st.size() < 3) {
      return; // Discard cases with little samples TODO: Magic constant
    }
    
    double delta_t = st.size() * sample_time;
    double wind_speed =parent->descending_ratio * parent->v_ref + 
		      current_detection->altitude_gain / delta_t ; 
    
    if (!second_exploration &&  n_up < 0) {
      sendPossibleThermal(p1, p2);
    } else if (second_exploration) {
      // Possible Updraft is converted into an Updraft and its resources are shared
      simulator::Updraft up(current_detection->center, wind_speed, urm->getMaxHeight(), 80.0);
      int pos_up = urm->isPossibleUpdraft(current_detection->center, 100.0);
      
      if (pos_up >= 0) {
	PossibleUpdraft pu;
	
	if (urm->getPossibleUpdraft(pos_up, pu)) {
// 	  up.drift = (current_detection->center - pu.center) / (current_detection->last_time - pu.detection_time); // TODO: A the drift
	  up.drift.clear(); // TODO: better estimate the drift
	  up.drift.push_back(-0.5);
	up.drift.push_back(0.0);
	  urm->deletePossibleUpdraft(urm->getPossibleUpdrafts().at(pos_up).center);
	}
      } else {
	up.drift.clear();
	up.drift.push_back(0.0);
	up.drift.push_back(0.0);
      }
      urm->addThermal(up);
      
    } else {
      // Actualize parameters of the thermal with the new measures!!
      Updraft old_therm = urm->getUpdrafts().at(n_up);
      RealVector new_drift = (current_detection->center - old_therm.getLocation()) / delta_t + old_therm.drift;
      urm->actualizeThermalParameters(current_detection->center, wind_speed, new_drift, n_up); 
    }
  }
}

bool ThermalDetector::writeLog() const {
  return functions::writeStringToFile(log_filename, getLog());
}

std::string ThermalDetector::getLog() const
{
  std::ostringstream os;
  
  for (unsigned int i = 0; i < detection_log.size(); i++) {
    os << detection_log.at(i)->toString() << endl;
  }
  
  return os.str();
}

void ThermalDetector::dispose()
{
  for (unsigned int i = 0; i < detection_log.size(); i++) {
    delete detection_log.at(i);
  }
}

ThermalDetector::~ThermalDetector()
{
  dispose();
}

string ThermalDetector::DetectionData::toString() const
{
  ostringstream os;
  
  os << states.at(0).toMatlabString() << " " << first_time.getFormattedTime(false) << " ";
  os << states.at(states.size() - 1).toMatlabString() << " " << last_time.getFormattedTime(false) << "\t";
  os << center.toMatlabString() << " " << altitude_gain << endl;
  
  return os.str();
}

void ThermalDetector::DetectionData::calculateCenter()
{
  double delta_z = 0.0;
  double sum_delta_z = 0.0;
  center.resize(2);
  
  double sum_x = 0.0; // Pondered sum x
  double sum_y = 0.0; // Pondered sum y
  for (unsigned int i = 1; i < states.size(); i++) {
    delta_z = states[i][2] - states[i - 1][2];
    sum_delta_z += delta_z;
    sum_x += states[i][0] *delta_z;
    sum_y += states[i][1] *delta_z;
  }
  
  center[0] = sum_x / sum_delta_z;
  center[1] = sum_y / sum_delta_z;
  altitude_gain = sum_delta_z;
}


functions::RealVector ThermalDetector::getPerpendicularLine(const functions::RealVector& p1_, const functions::RealVector& p2_) const
{
  functions::RealVector ret(2), p1(2), p2(2);
  
  p1[0] = p1_[0];
  p1[1] = p1_[1];
  p2[0] = p2_[0];
  p2[1] = p2_[1];
  
  functions::RealVector director = p2 - p1; // Vector (xd, yd)
  
  // First the slope:
  if (director[1] != 0.0) {
    ret[0] = -director[0]/director[1]; // slope director = m = yd / xd . Slope perpend = m'. Relationship: m * m' = -1 --> m' = - xd / yd
    if (ret[0] == 0.0) {
      ret[0] = 1e-100;
    }
  } else {
    ret[0] = 1e100;
  }
  
  // Then the independent therm: y1 = m * x + n --> n = y1 - m * x1 = y1 + x1 / m'
  ret[1] = p1[1] + p1[0] / ret[0];
  
  return ret;
}

bool ThermalDetector::sendPossibleThermal(const functions::RealVector &p1, const functions::RealVector &p2) const {
  PossibleUpdraft sent_message;
  bool ret = true;
  sent_message.p1 = p1;
  sent_message.p2 = p2;
  if (current_detection) {
    sent_message.center = current_detection->center;
  }
  
  sent_message.detection_time.getTime();
  urm->addPossibleUpdraft(sent_message);
  
  return ret;
}

ParseBlock* ThermalDetector::toBlock() const
{
  ParseBlock *ret = new ParseBlock;
 
  ret->setProperty("min_height", functions::numberToString(min_height));
  ret->setProperty("max_height", functions::numberToString(max_height));
  ret->setProperty("t_consecutive", functions::numberToString(t_consecutive));
  ret->setProperty("sample_time", functions::numberToString(sample_time));
  ret->setProperty("log_filename", log_filename);
  ret->setProperty("usual_radius", functions::numberToString(usual_radius));
  
  return ret;
}

