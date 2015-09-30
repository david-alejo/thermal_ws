/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#include "glider_planner/URM.h"
#include "functions/functions.h"
#include <iomanip>
#ifdef USE_KML
#include <kml/convenience/convenience.h>
#include <kml/dom.h>
#include <kml/engine.h>
#endif
#include <UAVFlightPlan/general.h>

#define KML_SEGMENTS_URM 1500

using namespace std;
using namespace functions;
using simulator::Updraft;

#ifdef USE_KML
using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::PointPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
#endif


using namespace UAVFlightPlan;
using namespace std;

namespace glider_planner {
  
string PossibleUpdraft::toString() const
{
  ostringstream os;
  
  os << "p1 = " << p1.toString() << " ";
  os << "p2 = " << p2.toString() << " ";
  os << "center = " << center.toString() << " ";
  os << "Detection time: " << detection_time.getFormattedTime(false);
  
  return os.str();
}


URM::URM(ParseBlock& block)
{
  init();
  
  data_loaded = init(block);
  
}

URM::URM(const std::vector< Updraft >& updraft_vec)
{
  init();
  updraft_vector = updraft_vec;
}

bool URM::init(ParseBlock& block)
{
  bool ret_val = true;
  
  Checker *check = getChecker();
  try {
    block.checkUsing(check);

    min_dist = block("min_dist").as<double>();
    waypoint_arrival_tolerance  = block("waypoint_tolerance").as<double>();
    
    // Optional parameters
    if (block.hasProperty("debug")) {
      debug = block("debug").as<bool>();
    }
    
    // Get updraft info
    ParseBlock::Blocks::iterator it;
    if (block.hasBlock("updraft")) {
      ParseBlock::Blocks *updrafts = block.getBlocks("updraft");
	
      updraft_vector.clear();
      it = updrafts->begin();
      for ( ; it != updrafts->end(); it++) {
	Updraft aux(**it);
	updraft_vector.push_back(aux);
	if (debug) {
	  cout << "URM::init --> Updraft loaded: " << aux.toString() << endl;
	}
      }
    }
    
    // Get temporal constrain info that already exists
    if (block.hasBlock("constraint")) {
      ParseBlock::Blocks *constraints = block.getBlocks("constraint");
	
      constraint_list.clear();
      it = constraints->begin();
      for ( ; it != constraints->end(); it++) {
        TimeSlot constraint_(**it);
        addConstraint(constraint_);
	if (debug) {
	  cout << "URM::init --> Constraint loaded: " << constraint_.toString() << endl;
	}
      }
    }

    // Last, but not least retrieve the waypoint information
    ParseBlock::Properties *waypoints = block.getProperties("waypoint");
    ParseBlock::Properties::iterator w_it = waypoints->begin();
    waypoint_list.clear();
    for ( ; w_it != waypoints->end(); w_it++) {
      RealVector aux( (*w_it)->as<vector<double> >());
      waypoint_list.push_back(aux);
      if (debug) {
         cout << "URM::init --> Loaded a waypoint: " << aux.toString() << endl;
      }
      
    }
    if (block.hasProperty("waypoint_file")) {
      waypoint_file = block("waypoint_file").value;
      waypoint_list.clear();
      getWaypointsFromFile(waypoint_file);
      cout << "Using waypoint file: " << waypoint_file << endl;
    }
    
    // Initialize the assigned uav map
    vector<RealVector>::iterator w_it2 = waypoint_list.begin();
    for (;w_it2 != waypoint_list. end(); w_it2++) {
      assigned_uav[*w_it2] = OBJECTIVE_NOT_ASSIGNED;
    }
    
    if (block.hasProperty("updraft_output")) {
      updraft_data_output_file = block("updraft_output").value;
    }
    
    if (block.hasProperty("possible_log_file")) {
      possible_log_file = block("possible_log_file").value;
    }
    
     if (block.hasProperty("urm_file")) {
      urm_file = block("urm_file").value;
    } else {
      urm_file = "urm.tex";
    }
    
    if (block.hasProperty("poi_file")) {
      poi_file = block("poi_file").value;
    } else {
      poi_file = "poi.tex";
    }
    
    if (block.hasProperty("max_height")) {
      max_height = block("max_height").as<double>();
    }
    
  } catch (exception &e) { 
    cerr << "URM::init --> Error while loading data. Content:" << e.what() << endl;
    ret_val = false;
    throw e;
  }
  data_loaded = ret_val;
  
  delete check;
  
  return ret_val;
}

void URM::init()
{
  debug = false;
  constraint_list.clear();
  updraft_vector.clear();
  waypoint_reached.clear();
  waypoint_cost.clear();
  ETA.clear();
  plans.clear();
  waypoint_arrival_tolerance = 10.0;
  possible_log_file = "";
  possible_log.clear();
  possible_updrafts.clear();
  max_height = 1000;
}

URM::~URM() {
  init();
}

bool URM::assignUAV(const RealVector& waypoint, int uav, double cost, FormattedTime eta)
{
   
   int curr_assignment = getAssignedUAV(waypoint);
   
   bool ret = false;
   try {
     if (curr_assignment != OBJECTIVE_NOT_IN_THE_LIST) {
       if (curr_assignment == OBJECTIVE_NOT_ASSIGNED || (curr_assignment == uav && !isWaypointReached(waypoint))||
              (waypoint_cost.at(waypoint) > cost)) {
	 boost::unique_lock<boost::shared_mutex> lock3(wp_mutex);
         assigned_uav[waypoint] = uav;
         waypoint_cost[waypoint] = cost;
	 ETA[waypoint] = eta;
         ret = true;
       
	functions::FormattedTime t;
	if (curr_assignment != uav) {
	  cout << "URM::assignUAV --> Waypoint " << waypoint.toString() << " assigned to UAV " << uav;
	  cout << ". New cost = " << cost << " . New ETA: " << eta << "\t Current time: " << t << endl;
	}
      }
    }
  } catch (std::exception &e) {
    std::cerr << "assignUAV:: Catched unexpected exception." << std::endl;
  }
  return ret;
 }
 
void URM::unassignUAV(const RealVector& waypoint, int uav)
{
  int curr_assignment = getAssignedUAV(waypoint);
  try {
    
     if (curr_assignment == uav && !isWaypointReached(waypoint)) {
       boost::unique_lock<boost::shared_mutex> lock2(wp_mutex);
       assigned_uav[waypoint] = OBJECTIVE_NOT_ASSIGNED;
     }
  } catch (std::exception &e) {
    std::cerr << "URM --> unassignUAV:: Catched unexpected exception." << std::endl;
  }
   
}


void URM::addConstraint(const TimeSlot& c)
{
  boost::unique_lock<boost::shared_mutex> lock_2(slot_mutex);
  
  list<TimeSlot>::iterator c_it;
  bool found = false;
  for (c_it = constraint_list.begin(); !found && c_it != constraint_list.end();) {
    if (c_it->updraft_id == c.updraft_id && c_it->uav_id == c.uav_id) {
//       cout << "URM::addConstrains --> removing constraint: " << c_it->toString() << "\t";
      c_it = constraint_list.erase(c_it);
      found = true;
    } else {
		c_it++;
	}
  }
//   cout << "URM::addConstrains --> adding constraint: " << c.toString() << endl;
  constraint_list.push_back(c);
}

Checker* URM::getChecker() const
{
  Checker *check = new Checker;
//   check->addBlock("updraft", new OneOrMore());
//   check->addChecker("updraft", Updraft::getChecker());
  check->addProperty("min_dist", new NTimes(1));
  check->addProperty("waypoint", new OneOrMore());
  check->addProperty("waypoint_tolerance", new NTimes(1));

  return check;
}

// Only registers the constraints related to the first thermal
void URM::getConstraints(const simulator::FlightPlan& fp, const UAV *uav)
{
  int updraft_id;
  bool thermal_found = false;
  
  TimeSlot curr_constraint;
  
//   boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  
  for (unsigned int cont = 0; cont < fp.size() && uav != NULL && !thermal_found; cont++) {
    updraft_id = getUpdraftFromLocation(fp.at(cont));
    // Not in an updraft
    if (updraft_id >= 0) {
      // Entering an updraft
      curr_constraint.updraft_id = updraft_id;
      curr_constraint.min_time = fp.getETA(cont);
//       cout << "Debug: ETA of fp: " << fp.getETA(cont).getFormattedTime() << endl;
      curr_constraint.initial_altitude = fp.at(cont).z;
      curr_constraint.uav_id = uav->id;
      curr_constraint.ascending_rate = uav->getAscendingRate(updraft_vector.at(updraft_id).getWindSpeed());
      
      addConstraint(curr_constraint);
      thermal_found = true;
      
    }
  }
//   plans.push_back(fp); // TODO: better management of the plans (allow an actualization, and many more things) (change to map)
}

bool URM::checkConstraints(const simulator::FlightPlan& fp, TimeSlotList& constraint, const UAV &uav)
{
  bool ret_val = true;
  
  constraint.clear();
  
  int curr_updraft = -1;
  int next_updraft = -1;
  
  FormattedTime init_time;
  double initial_altitude;
  
  for (unsigned int cont = 0; cont < fp.size(); cont++, curr_updraft = next_updraft) {
    next_updraft = getUpdraftFromLocation(fp.at(cont));
    
    if (curr_updraft < 0) {
      // Not in an updraft
      if (next_updraft >= 0) {
	// Entering an updraft
	init_time = fp.getETA(cont);
	initial_altitude = fp.at(cont).z;
      }
    } else {
      if (next_updraft != curr_updraft && cont > 0) {
	// Exiting updraft
	ret_val &= checkConstraints(init_time, initial_altitude, curr_updraft, constraint, uav);
	if (next_updraft != curr_updraft) {
	  init_time = fp.getETA(cont);
	}
      }
    }
    
  }
  
  
  return ret_val;
}

int URM::getUpdraftFromLocation(const functions::Point3D &p) const
{
  int updraft_id = -1;
  
  
  for (uint cont_up = 0; cont_up < updraft_vector.size() && updraft_id == -1;cont_up++) {
    const Updraft &u = updraft_vector[cont_up];
    functions::RealVector v;
    v.push_back(p.x);
    v.push_back(p.y);
    
    if (v.distance(u.getLocation()) <= u.radius * 1.01) {
      updraft_id = cont_up;
    }
  }
  return updraft_id;
}

std::string URM::toString() const
{
  ostringstream os;
  
  // Insert updraft list
  os << "----------------------- URM content ---------------- " << endl;
  os << "Updraft vector content: " << endl;
  vector<Updraft>::const_iterator u_it = updraft_vector.begin();
  for (;u_it != updraft_vector.end(); u_it++) {
    os << u_it->toString() << endl;
  }
  
  // At last, temporal constraints
  os << "Constraint list content: " << endl;
  list<TimeSlot>::const_iterator c_it = constraint_list.begin();
  for (;c_it != constraint_list.end(); c_it++) {
    os << c_it->toString() << endl;
  }
  os << "--------------------------- end -----------------------" << endl;
  
  return os.str();
}

ParseBlock* URM::toBlock() const
{
  ParseBlock *ret = new ParseBlock;
  
  ret->setProperty("min_dist", functions::numberToString(min_dist));
  ret->setProperty("waypoint_tolerance", functions::numberToString(waypoint_arrival_tolerance));
  ret->setProperty("debug", functions::boolToString(debug));
  ret->setProperty("poi_file", poi_file);
  ret->setProperty("urm_file", urm_file);
  ret->setProperty("updraft_output", updraft_data_output_file);
  ret->setProperty("possible_log_file", possible_log_file);
  ret->setProperty("max_height", functions::numberToString(max_height));
  
  // Insert updraft list
  vector<Updraft>::const_iterator u_it = updraft_vector.begin();
  for (;u_it != updraft_vector.end(); u_it++) {
    ret->setBlock("updraft", u_it->toBlock());
  }
  
  // Then the waypoint list
  for (unsigned int i = 0; i < waypoint_list.size(); i++) {
    ret->setProperty("waypoint", waypoint_list.at(i).toMatlabString());
  }
  if (waypoint_file != "") {
    ret->setProperty("waypoint_file", waypoint_file);
  }
  
  // At last, temporal constraints
  TimeSlotList::const_iterator c_it = constraint_list.begin();
  for (;c_it != constraint_list.end(); c_it++) {
    ret->setBlock("constraint", c_it->toBlock());
  }
  
  return ret;
}


bool URM::checkConstraints(FormattedTime init_time, double initial_altitude, int updraft_id, TimeSlotList& constrain
                           , const UAV &uav)
{
  bool ret_val = true;
  boost::shared_lock<boost::shared_mutex> lock(slot_mutex);
  boost::shared_lock<boost::shared_mutex> lock_1(updraft_mutex);
  
  list<TimeSlot>::const_iterator it = constraint_list.begin();
  
  for (;it != constraint_list.end(); it++) {
    const TimeSlot &curr = *it;
    // Check for altitude minimum separation violations in all constraints
    if (it->updraft_id == updraft_id && !curr.check(updraft_id, init_time, initial_altitude,
                                                   uav.getAscendingRate(updraft_vector.at(updraft_id).getWindSpeed()),
                                                   min_dist, updraft_vector.at(updraft_id).max_height, uav.id)) {
      // An overlap occured
      ret_val = false;
    
      // Log it! TODO: change the log type
//       cout << "URM::checkConstraints --> UAV: " << uav.toString() << " violates the constrain: ";
//       cout << it->toString() << endl;
      
      // Check if the constrain was already violated
      TimeSlotList::const_iterator it2 = constrain.begin();
      bool found = false;
      for (; it2 != constrain.end() && !found; it2++) {
	if ( (*it2) == (*it)) {
	  found = true;
	}
      }
      if (!found) {
	// If not --> add it
	if (lock.owns_lock()) {
	  lock.unlock();}
	boost::unique_lock<boost::shared_mutex> lock_2(slot_mutex);
	constrain.push_back(*it);
      }
    }
  }
  
  
  return ret_val;
}

string URM::toLatex(const string &caption, const string &label, int precision, double scale) const {
    ostringstream os;

    if (precision >= 0) {
        os << fixed << setprecision(precision);
      }

    os << "\\begin{table} %[h!]" << endl;
    os << "\\caption{" << caption << "}" << endl;
    os << "\\centering" << endl;
    os << "\\label{" << label << "}" << endl;
    os << "\\scalebox{" << scale << "} {                % change table size" << endl;
    os << "\\begin{tabular}{*{4}{|c}|c|} \\hline" << endl;
    os << "UAV & Updraft & Entry time $(s)$ & Altitude $(m)$ Climb rate$(m/s)$ \\\\ \\hline" << endl;

    TimeSlotList::const_iterator c_it = constraint_list.begin();


    for (; c_it != constraint_list.end(); c_it++) {
      os << c_it->uav_id + 1 << "& " << c_it->updraft_id + 1 << " & ";
      os << c_it->min_time << " & " << c_it->initial_altitude << " & ";
      os << c_it->ascending_rate;
      os << " \\\\ \\hline" << endl;
    }

    os << "\\end{tabular}" << endl;
    os << "} % Scalebox" << endl;
    os << "\\end{table}" << endl;
    
    

    return os.str();
}

string URM::eventsToLatex(const string &caption, const string &label, int precision, double scale) const {
    ostringstream os;

    if (precision >= 0) {
        os << fixed << setprecision(precision);
      }

    os << "\\begin{table} %[h!]" << endl;
    os << "\\caption{" << caption << "}" << endl;
    os << "\\centering" << endl;
    os << "\\label{" << label << "}" << endl;
    os << "\\scalebox{" << scale << "} {                % change table size" << endl;
    os << "\\begin{tabular}{*{6}{|c}|c|} \\hline" << endl;
    os << "UAV & Updraft & Entry time  & Altitude $(m)$ & Exit time  & Altitude $(m)$  \\\\ \\hline" << endl;

    std::list<URMEvent>::const_iterator c_it = events.begin();


    for (; c_it != events.end(); c_it++) {
      if (!c_it->exit) {
	os << c_it->uav + 1<< "& " << c_it->thermal + 1 << " & ";
	os << c_it->time.getFormattedTime(false) << " & " << c_it->altitude << " & ";
	
	std::list<URMEvent>::const_iterator c_it2 = c_it;
	bool found = false;
	for (;c_it2 != events.end() && !found; c_it2++) {
	  if (c_it2->uav == c_it->uav && c_it2->thermal == c_it->thermal && c_it2->exit) {
	    found = true;
	    os << c_it2->time.getFormattedTime(false) << " & " << c_it2->altitude ;
	  }
	}
	
	if (!found) {
	  os << c_it->time.getFormattedTime(false) << " & " << c_it->altitude;
	}
	
      os << " \\\\ \\hline" << endl;
     }
   }

    os << "\\end{tabular}" << endl;
    os << "} % Scalebox" << endl;
    os << "\\end{table}" << endl;
    
    

    return os.str();
}

string URM::waypointsToLatex(const string& caption, const string& label, int precision, double scale)
{
  ostringstream os;
  boost::shared_lock<boost::shared_mutex> lock_2(wp_mutex);
  
  if (precision >= 0) {
    os << fixed << setprecision(precision);
  }
  
  os << "\\begin{table} %[h!]" << endl;
  os << "\\caption{" << caption << "}" << endl;
  os << "\\centering" << endl;
  os << "\\label{" << label << "}" << endl;
  os << "\\scalebox{" << scale << "} {                % change table size" << endl;
  os << "\\begin{tabular}{*{4}{|c}|c|} \\hline" << endl;
  os << "Waypoint & ETA & Assigned UAV & Altitude & Visited? \\\\ \\hline" << endl;

  for (unsigned int i = 0; i < waypoint_list.size(); i++) {
    functions::RealVector curr = waypoint_list.at(i);
    os << curr.toString() << "& " << ETA[curr].getFormattedTime(false) << " & ";
    os << assigned_uav[curr] + 1 << " & ";
    os << waypoint_cost[curr] << " & ";
    os << functions::boolToString(waypoint_reached[curr]);
    
    os << " \\\\ \\hline" << endl;
  }

  os << "\\end{tabular}" << endl;
  os << "} % Scalebox" << endl;
  os << "\\end{table}" << endl;
   
  return os.str();
}

const TimeSlotList &URM::getConstraints() {
  boost::shared_lock<boost::shared_mutex> lock(slot_mutex);
  return constraint_list;
}

const std::vector<Updraft> &URM::getUpdrafts() {
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  return updraft_vector;
}

bool URM::allWaypointsVisited()
{
  bool ret_val = true;
  
  // Mutex!
//   boost::shared_lock<boost::shared_mutex> lock(wp_mutex);
  
  vector<RealVector>::const_iterator way_it = waypoint_list.begin();
  
  for (;way_it != waypoint_list.end() && ret_val; way_it++) {
    ret_val = isWaypointReached(*way_it);
  } 
  boost::shared_lock<boost::shared_mutex> lock(possible_mutex);
  for (unsigned i = 0; i < possible_updrafts.size(); i++) {
    
    ret_val = isWaypointReached(possible_updrafts.at(i).center);
  }
  
  return ret_val;
}

bool URM::waypointReached(const RealVector &v, uint uav_id, const UAV *uav)
{
//   boost::shared_lock<boost::shared_mutex> lock(wp_mutex);
  
  bool ret_val = false;
  
  RealVector v_2d;
  v_2d.push_back(v.at(0));
  v_2d.push_back(v.at(1));
  
  vector<RealVector>::const_iterator way_it = waypoint_list.begin();
  
//   lock.unlock();
  for (;way_it != waypoint_list.end() && !ret_val; way_it++) {
    if ( isWaypointReached(*way_it)) {
      continue; // Do not take into account the objectives already reached
    }
    
    ret_val = way_it->distance(v_2d) < waypoint_arrival_tolerance;
    
    if (ret_val) {
      boost::unique_lock<boost::shared_mutex> lock_3(wp_mutex);
    
      waypoint_reached[*way_it] = true;
      functions::FormattedTime t;
      t.getTime();
      ETA[*way_it] = t;
      waypoint_cost[*way_it] = v.at(2);
      
      appendPlan(v, uav_id, false, uav);
      
    }
  }
  
  return ret_val;
}

int URM::getAssignedUAV(const RealVector& waypoint) {
  int ret = OBJECTIVE_NOT_IN_THE_LIST;
  boost::shared_lock<boost::shared_mutex> lock(wp_mutex);
  
  if (assigned_uav.find(waypoint) == assigned_uav.end()) {
    std::vector<functions::RealVector>::const_iterator it = waypoint_list.begin();
    bool found = false;
    for (;it != waypoint_list.end() && !found; it++) {
      found = *it == waypoint;
     }
     if (found) {
       ret = OBJECTIVE_NOT_ASSIGNED; // Not assigned yet
     }
  } else {
    ret = assigned_uav.at(waypoint);
  }
  
  return ret;
}

bool URM::isWaypointReached(const RealVector &waypoint) {
  boost::shared_lock<boost::shared_mutex> lock(wp_mutex);
  
  if ( waypoint_reached.find(waypoint) == waypoint_reached.end() ) {
    return false;
  } else {
    return waypoint_reached[waypoint];
  }    
}

vector< RealVector > URM::getWaypoints()
{
  boost::shared_lock<boost::shared_mutex> lock_2(wp_mutex);
  
  return waypoint_list;
}

// TODO: not thread safe
bool URM::exportKMLFile(const std::string &filename)
{
  #ifdef USE_KML
  kmldom::KmlFactory *fac = kmldom::KmlFactory::GetFactory();
  kmldom::DocumentPtr doc = fac->CreateDocument();
  for (unsigned int i = 0; i < waypoint_list.size(); i++) {
    const RealVector &cwp(waypoint_list.at(i));
    ostringstream name_;
    name_ << "Waypoint " << i;
    EarthLocation e(center);
    e.shift(cwp.at(0), cwp.at(1));
    PlacemarkPtr place = kmlconvenience::CreatePointPlacemark(name_.str(), e.getLatitude(), e.getLongitude());
    doc->add_feature(place);
  }
  
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
    Updraft &u(updraft_vector.at(i));
    ostringstream name_;
    name_ << "Updraft" << i;
    EarthLocation e(center);
    e.shift(u.getLocation().at(0), u.getLocation().at(1));
    
    kmldom::CoordinatesPtr coord = kmlconvenience::CreateCoordinatesCircle(e.getLatitude(), e.getLongitude(), u.radius, KML_SEGMENTS_URM);
    kmldom::LinearRingPtr ring = fac->CreateLinearRing();
    ring->set_coordinates(coord);
    kmldom::PlacemarkPtr place = kmlconvenience::CreateBasicPolygonPlacemark(ring);
    place->set_name(name_.str());
    doc->add_feature(place);
  }


  kmldom::KmlPtr kml = fac->CreateKml();
  kml->set_feature(doc);
	
  return writeKMLToFile(filename, kml);
#else
  return false;
#endif
}

bool URM::updateEvent(const RealVector& position, int uav, const UAV *uav_)
{
  bool ret = false;
  
  int new_pos = getUpdraftFromLocation(position);
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  
  // First initialize map if necessary
  if (curr_updraft.find(uav) == curr_updraft.end()) {
    lock.unlock();
    boost::unique_lock<boost::shared_mutex> lock2(updraft_mutex);
    curr_updraft[uav] = new_pos;
  } else {
    if (curr_updraft[uav] != new_pos) {
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> lock2(updraft_mutex);
      ret = true;
      // An event has ocurred
      URMEvent event;
      event.uav = uav;
      event.altitude = position.at(2);
      event.exit = new_pos == -1;
      if (event.exit) {
	event.thermal = curr_updraft[uav];
      } else {
	event.thermal = new_pos;
      }
      FormattedTime t;
      t.getTime();
      event.time = t; 
      events.push_back(event);
      
      if (new_pos < 0) {
	RealVector v = updraft_vector.at(curr_updraft.at(uav)).getLocation();
	v.push_back(updraft_vector.at(curr_updraft.at(uav)).getMaxHeight());
	appendPlan(v, uav, true, uav_);
      } else {
	appendPlan(position, uav, false, uav_);
      }
       
      curr_updraft[uav] = new_pos;
      
    }
  }
  
  return ret;
}

simulator::FlightPlan URM::getAlternativePlan(const UAV &uav)
{
  simulator::FlightPlan output_plan;
  RealVector pos;
  pos.push_back(uav.current_location.at(0));
  pos.push_back(uav.current_location.at(1));
  double altitude = uav.current_location.at(2); 
  
  int nt = getNearestThermal(pos); // Reach the nearest thermal
  
  // If the alternative is called, we have to unassign all WPs pending to visit for this UAV
  for (unsigned int i = 0; i < waypoint_list.size(); i++) {
    unassignUAV(waypoint_list.at(i), uav.id);
  }
  
  
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  functions::Point3D p(uav.current_location);
  output_plan.push_back4d(p, uav.curr_time);
  
  
  
  if (nt >= 0) {
    Updraft &n_up = updraft_vector.at(nt);
    
    RealVector to_up = n_up.getLocation() - pos;
//     p.z = altitude - n_up.radius * &uav;
    
    
    // First we have to check if we can enter the nearest updraft. If so, we enter in it
    double expended_time = n_up.getLocation().distance(pos) / uav.v_ref;
    FormattedTime entry_time = uav.curr_time + expended_time;
    double initial_altitude = altitude - n_up.getLocation().distance(pos) * uav.descending_ratio;
    FormattedTime t;
    
    TimeSlotList slots_;
    if (initial_altitude < uav.min_alt) {
      // Critical situation!!!!!!!!!!!!!!!!!
      // Houston we got a problem --> we cannot enter the nearest thermal, so we go home and hope we can land safely
      cout << "Could not reach the nearest thermal --> going home. " << nt + 1 ;
      p = uav.getArrivalTimeAltitude(pos, uav.home_location, initial_altitude, t);
      p.z = uav.initial_location[2];
      output_plan.push_back4d(p, t);
    } else if ( checkConstraints(entry_time, initial_altitude, nt, slots_, uav) ) {
      
      cout << "Going inside the nearest thermal. " << nt + 1 ;
      // Get the new plan: go to the thermal and then to home
      p.x = n_up.getLocation().at(0);
      p.y = n_up.getLocation().at(1);
      p.z = initial_altitude;
      output_plan.push_back4d(p, entry_time);
      
      // TODO: move this calculation into an URM method
      FormattedTime exit_time = entry_time;
      p = getFinalUpdraftTime(n_up.getLocation(), initial_altitude, nt, exit_time, uav);
      output_plan.push_back4d(p, exit_time);
      
      p = uav.getArrivalTimeAltitude(n_up.getLocation(), uav.home_location, n_up.max_height, exit_time);
      output_plan.push_back4d(p, exit_time);
      
      output_plan.setReachAltitude(output_plan.size() - 1, true);
      output_plan.setClimbRate(output_plan.size() - 1, uav.getAscendingRate(n_up.getWindSpeed()));
      
      getConstraints(output_plan, &uav);
    } else if (n_up.getLocation().distance(pos) < n_up.radius * 5.0) {
      // If the thermal is too close --> border with a greater radius
      // TODO: decide the direction of the turn taking into account nearest vehicles
      RealVector norm_up;
      norm_up.push_back(-to_up.at(1));
      norm_up.push_back(to_up.at(0));
      norm_up.normalize();
	  
      RealVector new_loc = pos + norm_up * n_up.radius;
      p = uav.getArrivalTimeAltitude(pos, new_loc, altitude, t);
      cout << "Circling near the thermal. " << nt << " ";
      output_plan.push_back4d(p, t);
    } else {
      // If the thermal is not too close --> go for it!
      to_up.normalize();
      RealVector new_pos = pos + to_up * n_up.radius;
       p = uav.getArrivalTimeAltitude(pos, new_pos, altitude, t);
      
      cout << "Going towards the thermal but not entering inside. " << nt;
      output_plan.push_back4d(p, t);
    }
	
    cout << " Alternative plan: " << output_plan.toString() << endl;
  }
  
  return output_plan;
}

int URM::getNearestThermal(const RealVector& pos)
{
  double min_dist = 1e30;
  functions::RealVector pos2d;
  pos2d.push_back(pos.at(0));
  pos2d.push_back(pos.at(1));
  int min = -1;
  
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
     if (updraft_vector.at(i).getWindSpeed() < 1.0) {
      continue; // Discard thermals already dead
    }
    
    double curr_dist = updraft_vector.at(i).getLocation().distance(pos2d);
    if (curr_dist < min_dist) {
      min = i;
      min_dist = curr_dist;
    }
    
  }
  
  return min;
}

Point3D URM::getFinalUpdraftTime(RealVector pos2d, double h0, int n_up, FormattedTime& t, const UAV &uav)
{
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  Point3D p(updraft_vector.at(n_up).getLocation());
  p.z = updraft_vector.at(n_up).max_height;
  
  t = t + (p.z - h0) * (uav.getAscendingRate(updraft_vector.at(n_up).getWindSpeed()));
  
  return p;
}

bool URM::addThermal(const Updraft& new_updraft)
{
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  bool add = true;
  
  const RealVector &new_loc = new_updraft.getLocation();
  
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
    Updraft &curr = updraft_vector.at(i);
    if (new_loc.distance(curr.getLocation()) < curr.radius * 1.2) {
      add = false; // Too near an existent thermal --> discard it
    }
  }
  
  if (add) {
    lock.unlock();
    boost::unique_lock<boost::shared_mutex> lock_2(updraft_mutex);
    updraft_vector.push_back(new_updraft);
    cout << "URM --> Adding updraft: " << new_updraft.toString() <<endl;
  }
  
  return add;
}

bool URM::addPossibleUpdraft(const glider_planner::PossibleUpdraft& p_up)
{
  
  boost::shared_lock<boost::shared_mutex> lock(updraft_mutex);
  bool add = true;
  
  RealVector new_loc(2);
  new_loc[0] = p_up.center.at(0);
  new_loc[1] = p_up.center.at(1);
  
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
       Updraft &curr = updraft_vector.at(i);
    if (new_loc.distance(curr.getLocation()) < curr.radius * 1.2 && curr.getWindSpeed() > 0.5) { // TODO: isAlive() --> in Updraft function
      add = false; // Too near an existent thermal --> discard it
    }
  }
  
  lock.unlock();
  boost::shared_lock<boost::shared_mutex> lock_2(possible_mutex);
  for (unsigned int i = 0; i < possible_updrafts.size(); i++) {
       PossibleUpdraft &curr = possible_updrafts.at(i);
    if (new_loc.distance(curr.center) < 100.0) {
      add = false; // Too near an existent thermal --> discard it
    }
  }
  lock_2.unlock();
  
  if (add) {
    cout << "URM: Adding possible updraft. Content: " << p_up.toString() << endl;
    boost::unique_lock<boost::shared_mutex> lock_2(possible_mutex);
    possible_updrafts.push_back(p_up);
    possible_log.push_back(p_up);
    lock_2.unlock();
    boost::unique_lock<boost::shared_mutex> lock_3(slot_mutex);
    assigned_uav[p_up.center] = OBJECTIVE_NOT_ASSIGNED;
  }
  
  return add;
}

int URM::isPossibleUpdraft(const RealVector& new_possible, double min_dist)
{
  int ret_val = -1;
  boost::shared_lock<boost::shared_mutex> lock(possible_mutex);
  
  for (unsigned int i = 0; i < possible_updrafts.size() && ret_val < 0; i++) {
    PossibleUpdraft &curr = possible_updrafts.at(i);
    if (new_possible.distance(curr.center) < min_dist) {
      ret_val = i; // Too near an existent thermal --> discard it
    }
  }
  
  return ret_val;
}

bool URM::deletePossibleUpdraft(const RealVector& pos_up)
{
  int i = isPossibleUpdraft(pos_up, 100.0);
  bool ret = false;
  
  if (i >= 0 && (int)possible_updrafts.size() > i) {
    cout << "URM --> Deleting possible updraft: " << possible_updrafts.at(i).toString() << " ";
  }
  
  if (i >= 0) {
    cout << "Calling delete...";
    boost::unique_lock<boost::shared_mutex> lock_2(possible_mutex);
    
    possible_updrafts.erase(possible_updrafts.begin() + i);
  }
  cout << endl;

  return ret;
}
  
bool URM::getWaypointsFromFile(const string& filename)
{
  std::vector<vector<double> > v;
  bool ret_val = functions::getMatrixFromFile(filename, v);
  
  for (unsigned int i = 0; i < v.size(); i++) {
    RealVector aux(v.at(i));
    if (aux.size() == 2) {
      waypoint_list.push_back(aux);
    }
  }
  
  return ret_val;
}

bool URM::writeUpdrafts() const
{
  ostringstream os;
  if (updraft_vector.size() > 0) {
    os << functions::printToStringVector(updraft_vector, true) ;
    return functions::writeStringToFile(updraft_data_output_file, os.str());
  }
  
  return functions::writeStringToFile(updraft_data_output_file, " ");
}

bool URM::writePossibleLog() const
{
  if (possible_log_file.size() == 0) {
    return false;
  }
  
  ostringstream os;
  
  if (possible_log.size() > 0) {
    os << functions::printToStringVector(possible_log, true);
  }
  
  return functions::writeStringToFile(possible_log_file, os.str());
}

// TODO: make it thread safe (still not necessary)
void URM::setStartingTime(FormattedTime start_time)
{
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
    updraft_vector.at(i).initial_time = start_time;
  }
}

void URM::addWaypoint(const RealVector& new_way)
{
  boost::unique_lock<boost::shared_mutex> lock_2(wp_mutex);
  waypoint_list.push_back(new_way);
}


void URM::appendPlan(const RealVector& v, uint UAVid, bool reach_altitude, const UAV *uav)
{
  for (uint a = plans.size(); a <= UAVid; a++) {
    plans.push_back(simulator::FlightPlan());
  }
  
  cerr << "Plans size = " << plans.size() << ". UAVid = " << UAVid << ". v.size() " << v.size() <<  std::endl;
  
  if (v.size() > 2) {
    simulator::FlightPlan &p = plans.at(UAVid);
    p.push_back(Point3D(v.at(0), v.at(1), v.at(2)));
    p.setReachAltitude(p.size() - 1, reach_altitude);
    int updraft_id = getUpdraftFromLocation(v);
    if (updraft_id > 0 && updraft_id < updraft_vector.size()) {
      p.setClimbRate(p.size() - 1, uav->getAscendingRate(updraft_vector.at(updraft_id).getWindSpeed()));
    }
  }
}


}