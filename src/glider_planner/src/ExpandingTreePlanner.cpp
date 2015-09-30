#include "glider_planner/ExpandingTreePlanner.h"
#include <functions/Point3D.h>
#include <functions/functions.h>

using namespace functions;
using namespace std;
using simulator::Updraft;

namespace glider_planner {

std::string ExpandingTreePlanner::type = "Tree";

const double ExpandingTreePlanner::MIN_DISTANCE = 30.0; // Distance when the planner skips the waypoint and goes to the next one.

ExpandingTreePlanner::ExpandingTreePlanner(): consec_descending(0), max_consec_descending(10),
dist_go_updraft(100), home()
{
}

ExpandingTreePlanner::~ExpandingTreePlanner()
{
  dispose();
}

void ExpandingTreePlanner::dispose()
{
  waypoint_list.clear();
  updraft_vector.clear();
  last_plan.clear();
  time_slots.clear();
}

SoaringPlanner *ExpandingTreePlanner::createFromBlock(ParseBlock &block) const {
  ExpandingTreePlanner *p = new ExpandingTreePlanner;
  SoaringPlanner &sp = *p;

  Checker *check = getChecker();
  
  p->inupdraft = false;

  try {
    sp.init(block);
    block.checkUsing(check);
    p->depth = block("depth").as<int>();
    
    if (block.hasProperty("safety_coefficient")) {
      p->safety_coefficient = block("safety_coefficient").as<double>();
    } else {
      p->safety_coefficient = 1.1;
    }
    
    if (block.hasProperty("dist_go_updraft")) {
      p->dist_go_updraft = block("dist_go_updraft").as<double>();
    } else {
      p->dist_go_updraft = 100.0;
    }
  } catch (exception &e) {
    delete p;
    p = NULL;
  }

  delete check; // Free the checker
  return p;
}

ParseBlock* ExpandingTreePlanner::toBlock() const
{
    ParseBlock *block = SoaringPlanner::toBlock();
    
    block->setProperty("depth", functions::numberToString(depth));
    block->setProperty("type", type);
    
    return block;
}


bool ExpandingTreePlanner::execute(simulator::FlightPlan &output_plan) {
  bool ret_val = true;
  
  // Actualize
  waypoint_list = urm->getWaypoints();
  output_plan.clear();
  updraft_vector = urm->getUpdrafts();
  time_slots = urm->getConstraints();
  possible_updrafts = urm->getPossibleUpdrafts();

  TreeState initial_state (uav);
  TreeState initial (uav);
  TreeState best;
  best.value = 1e100;
  
  //! Check if we are already in an updraft
  int curr_updraft = inUpdraft(initial_state); // Check if we are already in an updraft
  int near_updraft = getCloseThermal(initial_state); near_updraft = -1;
  if (curr_updraft >= 0 && initial_state.altitude < updraft_vector.at(curr_updraft).max_height * 0.9 && consec_descending < max_consec_descending) {
    // 
    if (last_node.altitude >= initial_state.altitude) {
      // Dead thermal??
      consec_descending++;
    } else {
      consec_descending = 0;
    }
    
//     initial_state.altitude = updraft_vector.at(curr_updraft).max_height;
    RealVector v3d;
    v3d = updraft_vector.at(curr_updraft).getLocation();
    v3d.push_back(initial_state.altitude);
    initial_state.by.push_back(v3d);
  } else if (near_updraft >= 0) {
    cout << "Close of an updraft --> redirecting there!" << endl;
    RealVector v3d;
    v3d = updraft_vector.at(near_updraft).getLocation();
    v3d.push_back(initial_state.altitude);
    initial_state.by.push_back(v3d);
    consec_descending = 0;
  } else if (consec_descending >= max_consec_descending) {
    cout << "Dead termal detected!!" << endl;
  }
  
  ret_val = informedSearch(initial_state, depth, best);
  // Get the flight plan
  if (debug && ret_val) {
//       cout << "ExpandingTreePlanner::execute --> By content: " << printToStringVector(best.by) << endl;
  }	

  ret_val = getPlanFromBy(best.by, output_plan, initial_state);
  // Reserve the waypoints in the plan and the slt time!!
  if (ret_val && best.by.size() > 0) {
    if (!getWaypoints(best) && !getPossibleUpdrafts(best)) {
      cerr << "ExpandingTreePlanner: URM did not assign the UAV as requested. This should not happen.\n";
    }
    urm->getConstraints(output_plan, uav);
    last_plan = output_plan; // Actualize the last successfully generated plan
  } else {
    // No feasible plan has been calculated --> Stay go to the nearest thermal!
    cout << "ExpandingTreePlanner: Could not get a feasible plan. Calling URM for alternative plans.\n" ;
    output_plan = urm->getAlternativePlan(*uav);
    ret_val = false;
  }
  
  last_node = initial;
  
  return ret_val;
}

Checker *ExpandingTreePlanner::getChecker() {
  Checker *ret = new Checker;

  ret->addProperty("depth", new NTimes(1));

  return ret;
}



bool ExpandingTreePlanner::informedSearch(TreeState node, int curr_depth, TreeState &best) {
  //variables:
  bool ret = false;
//   successor_vector.clear();


  // Algorithm
  if(curr_depth <= 0) {
    return false;
  }
  ret = true;
  
  int curr_updraft = inUpdraft(node);
  if (curr_updraft >= 0 && node.altitude < updraft_vector.at(curr_updraft).getMaxHeight() * 0.95) {
    node.altitude = updraft_vector.at(curr_updraft).max_height;
    RealVector v3d;
    v3d = updraft_vector.at(curr_updraft).getLocation();
    v3d.push_back(node.altitude);
    node.by.push_back(v3d);
  }
  
  SuccessorVector successor_vector = getSuccessors(node);
     
  for(uint i = 0; i < successor_vector.size(); i++) {
    TreeState candidate;
    candidate.value = 1e100;
    bool ret_2 = true;
    
    if (isFinalState(successor_vector.at(i)) || depth == 1) {
      candidate = evaluation(successor_vector.at(i));
    } else {
      // If not final state and the maximum depth has been not reached --> keep going!
      ret_2 = informedSearch(successor_vector.at(i), curr_depth - 1, candidate);
    }
    
    if (ret_2 && candidate.value < best.value) {
      ret = true;
      best = candidate;
    }
  }
    
  return ret;
}

bool ExpandingTreePlanner::isFinalState(TreeState node) {
  bool ret = false;

  for (unsigned int i = 0; i < waypoint_list.size() && !ret; i++) {
    ret = waypoint_list.at(i) == node.pos;
//     ret = waypoint_list.at(i).distance(node.pos) < 0.01;
  }
  // TODO: is it worth to make the possible thermals as final states? We'll try it
  for (unsigned int i = 0; i < possible_updrafts.size() && !ret; i++) {
    ret = possible_updrafts.at(i).p1 == node.pos || possible_updrafts.at(i).p2 == node.pos;
  }

  return ret;
}

TreeState ExpandingTreePlanner::evaluation(TreeState nodo) {
  TreeState ret(nodo);
  if(isFinalState(nodo)) {
      ret.value = nodo.dist;
  }
  else {
//       ret.by.push_back(ret.pos);
      //Busca la distancia del destino (libre) más cercano (Para converger a el)
      ret.dist = nodo.dist + distToClosestWaypoint(nodo.pos);
      ret.value = ret.dist;
  }
  return ret;
}

double ExpandingTreePlanner::distToClosestWaypoint(const functions::RealVector &pos) const {
  double dist = 1e30;

  // Searches for the closest unassigned waypoint
  for( uint i = 0; i < waypoint_list.size(); i++) {
    if(urm->getAssignedUAV(pos) == URM::OBJECTIVE_NOT_ASSIGNED) {
      dist = min(pos.distance(waypoint_list.at(i)), dist);
    }
  }

  return dist;
}

ExpandingTreePlanner::SuccessorVector ExpandingTreePlanner::getSuccessors(TreeState nodo, bool first) {
  TreeState aux;
  double radio = (nodo.altitude - uav->min_alt) / uav->getAltitudeRatio(); // Calculate the action ratio
  const double radio_th = (nodo.altitude - uav->min_alt_thermal) / uav->getAltitudeRatio(); // Calculate the action ratio
  double distance; // In m (like all distances)
  FormattedTime tInTermica;
  ExpandingTreePlanner::SuccessorVector successor_vector;

  // First check the waypoints
  for (unsigned int x = 0; x < waypoint_list.size(); x++) {
    RealVector &way = waypoint_list.at(x);
    distance = nodo.pos.distance(way);
    
    double distance_near_thermal = 0.0;
    int near_therm = getNearestThermal(way);
    if (near_therm >= 0 ) {
      distance_near_thermal = way.distance(updraft_vector.at(near_therm).getLocation());
    } else {
      distance_near_thermal = way.distance(home.pos); // If there are no thermals in the system --> check if we can go back home!!
    }

    if (safety_coefficient *(distance_near_thermal + distance) <= radio && distance > MIN_DISTANCE && !urm->isWaypointReached(way)) {
      // I have the energy necessary to go to the target and then go back --> I can reach the target

      if(urm->getAssignedUAV(way) == uav->id || urm->getAssignedUAV(way) == URM::OBJECTIVE_NOT_ASSIGNED) {
        // Get it!!
        aux = actualizeNode(nodo, way, distance, first);
        // Add it to successor vector
        successor_vector.push_back(aux);
      } else if (urm->getAssignedUAV(way) >= 0) {
        // Another UAV has it!
        // When will I get that point?
        tInTermica.getTime();
        tInTermica = tInTermica + distance / uav->v_ref;

        // Will I arrive before the other UAV? (With a configurable hysteresis)
        if ((tInTermica + t_hysteresis) < urm->getETA(way)) {
          // Get the waypoint
          aux = actualizeNode(nodo, way, distance, first);
          successor_vector.push_back(aux);
        }
      }
    } // end if
  }// end for

  // Then the updrafts
  for (unsigned int x = 0; x < updraft_vector.size(); x++) {
    Updraft &curr_updraft = updraft_vector.at(x);
    
    if (curr_updraft.getWindSpeed() < uav->descending_ratio * uav->v_ref *0.5) {
      // Discard dead thermals!!
      continue;
    }
    
    distance = nodo.pos.distance(updraft_vector.at(x).getLocation());
    
    if (distance < curr_updraft.radius && curr_updraft.max_height > nodo.altitude + 10) {
      // Do no take into account this thermal
      continue;
    }
    
    if (distance <= radio) {
      // Calculate the ETA to the thermal
      // Hora actual + distancia:
      tInTermica = nodo.time + distance / uav->v_ref;
      aux = actualizeThermalNode(nodo, x, distance, first);
      double init_alt;
      if (aux.by.size() >= 2) {
	init_alt = aux.by.at(aux.by.size() - 2).at(2);
      } else {
	init_alt = aux.by.at(aux.by.size() - 1).at(2);
      }
      
      // Add to successor list if the constraints have been met
      TimeSlotList aux_list;
      
      if (urm->checkConstraints(tInTermica, init_alt , x, aux_list, *uav)) {
	// Check that we can actually enter the updraft. If not, this successor is not valid.
	successor_vector.push_back(aux);
      } else {
	cout << "ExpandingTreePlanner::getSuccessors --> could not reach the thermal: " << updraft_vector.at(x).toString() << ".\n";
	cout << "Because of the constraints: " << functions::printToStringList(aux_list);
      }
    }
  } // for updrafts
  
  // Finally go for the possible updrafts
  for (unsigned int x = 0; x < possible_updrafts.size(); x++) {
    if (!possible_updrafts.at(x).isAlive()) {
      continue;
    }
    
    RealVector &way = possible_updrafts.at(x).center;
    distance = nodo.pos.distance(way);
    int near_index = getNearestThermal(way);
    double distance_near_thermal = 0.0;
    if (near_index >= 0 && near_index < (int)updraft_vector.size()) {
      distance_near_thermal = way.distance(updraft_vector.at(near_index).getLocation());
    }

    if (safety_coefficient *(distance_near_thermal + distance) <= radio && distance > MIN_DISTANCE && !urm->isWaypointReached(way)) {
      // I have the energy necessary to go to the target and then go back --> I can reach the target

      if(urm->getAssignedUAV(way) == uav->id || urm->getAssignedUAV(way) == URM::OBJECTIVE_NOT_ASSIGNED) {
        // Get it!!
        aux = actualizePossibleThermalNode(nodo, x, first);
        // Add it to successor vector
        successor_vector.push_back(aux);
      } else if (urm->getAssignedUAV(way) >= 0) {
        // Another UAV has it!
        // When will I get that point?
        tInTermica = tInTermica + distance / uav->v_ref;

        // Will I arrive before the other UAV? (With a configurable hysteresis)
        if ((tInTermica + t_hysteresis) < urm->getETA(way)) {
          // Get the waypoint
          aux = actualizePossibleThermalNode(nodo, x, first);
          successor_vector.push_back(aux);
        }
      }
    } // end if
  }// end for
  return successor_vector;
}

TreeState ExpandingTreePlanner::actualizeNode(const TreeState& nodo, const RealVector& pos, double distance, bool first)
{
  TreeState aux;
  aux.pos = pos;

  //Necessary alt. en m expended in going to the target
  aux.altitude = nodo.altitude - (distance * uav->getAltitudeRatio());
  aux.dist = nodo.dist + distance;
  aux.value = aux.dist;
  aux.time = nodo.time + distance / uav->v_ref;
  
  functions::RealVector pos3d(aux.pos);
  pos3d.push_back(aux.altitude);
  
  if (first) {
    aux.by.clear();
    aux.by.push_back(pos3d);
  } else {
    aux.by = nodo.by;
    aux.by.push_back(pos3d);
  }
  return aux;
}

TreeState ExpandingTreePlanner::actualizeThermalNode(const TreeState& nodo, int x, double distance, bool first)
{
  const RealVector &thermal_pos = updraft_vector.at(x).getLocation();
  
  // Add the node with the estimated arrival altitude to the thermal
  TreeState new_node = actualizeNode(nodo, thermal_pos, distance, first);
    
  if (nodo.pos.distance(thermal_pos) < updraft_vector.at(x).radius) {
    // If we are inside the thermal --> erase the generated by clause in actualizeNode
    new_node.by.erase(new_node.by.end() - 1);
  }
  
  // Then add a by clause with the maximum thermal height TODO: Check whether if the maximum height is imposed by the thermal or by the aircraft instead
  new_node.pos = thermal_pos;
  
  Point3D p = urm->getFinalUpdraftTime(thermal_pos, new_node.altitude, x, new_node.time, *uav);
  
  functions::RealVector pos3d(p);
  new_node.by.push_back(pos3d);
  new_node.altitude = p.z;
  
  return new_node;
}

TreeState ExpandingTreePlanner::actualizePossibleThermalNode(const TreeState& nodo, int x, bool first)
{
  PossibleUpdraft &pos_up = possible_updrafts.at(x);
  RealVector p1, p2;
  RealVector pm = pos_up.center;
  
  double dist_1 = pos_up.p1.distance(nodo.pos);
  double dist_2 = pos_up.p2.distance(nodo.pos);
//   double distance;
  
  if (dist_1 <= dist_2) {
    p1 = pos_up.p1;
    p2 = pos_up.p2;
//     distance = dist_1;
  } else {
    // Interchange points and distance
    p1 = pos_up.p2;
    p2 = pos_up.p1;
//     distance = dist_2;
  }
  
  // Add the node with the estimated arrival altitude to the thermal
  TreeState new_node_1 = actualizeNode(nodo, p1, nodo.pos.distance(p1), first);
  TreeState new_node_m = actualizeNode(new_node_1, pm, p1.distance(pm));
  TreeState new_node_2 = actualizeNode(new_node_m, p2, p2.distance(pm));
  
  new_node_2.value = new_node_1.dist;
  new_node_2.dist = new_node_1.dist; // Prefer exploring new thermals to explore new waypoints
  
   // TODO: checks if the position is inside the possible updraft 
//   if (nodo.pos.distance(thermal_pos) < updraft_vector.at(x).radius) {
    // If we are inside the thermal --> erase the generated by clause in actualizeNode
//     new_node.by.erase(new_node.by.end() - 1);
//   }
  
  // Then add a by clause with the maximum thermal height TODO: Check whether if the maximum height is imposed by the thermal or by the aircraft instead
//   new_node.pos = thermal_pos;
  
//   Point3D p = urm->getFinalUpdraftTime(thermal_pos, new_node.altitude, x, new_node.time, *uav);
//   
//   functions::RealVector pos3d(p);
//   new_node.by.push_back(pos3d);
//   new_node.altitude = p.z;
  
  return new_node_2;
}


string ExpandingTreePlanner::toString() const {
  ostringstream os;

  os << "Soaring planner type: Expanding Tree Planner.";

  return os.str();
}

bool ExpandingTreePlanner::isUpdraftIncludedInPlan(unsigned int x)
{
  bool ret = false;
  for (unsigned int i = 0; i < last_plan.size() && !ret; i++) {
    RealVector v;
    v.push_back(last_plan.at(i).x);
    v.push_back(last_plan.at(i).y);
    if (updraft_vector.at(x).getLocation().distance(v) < updraft_vector.at(x).radius) {
      
      ret = true;
    }
  }
  return ret;
} 

int ExpandingTreePlanner::inUpdraft(TreeState& st) const {
  int ret = -1;
  for (unsigned int i = 0; ret < 0 && i < updraft_vector.size(); i++) {
    if (st.pos.distance(updraft_vector.at(i).getLocation()) < updraft_vector.at(i).radius ) {
      ret = i;
    }
  }
  
  return ret;
}

int ExpandingTreePlanner::getCloseThermal(const TreeState& st) const
{
  int ret = -1;
  for (unsigned int i = 0; ret < 0 && i < updraft_vector.size(); i++) {
    if (st.pos.distance(updraft_vector.at(i).getLocation()) < dist_go_updraft &&
        st.altitude < 0.8 * updraft_vector.at(i).getMaxHeight())
    {
      ret = i;
    }
  }
  
  return ret;
}


bool ExpandingTreePlanner::getWaypoints(const TreeState& st) const
{
  bool ret_val = true;
  
  // First for the waypoints
  for (unsigned int way_it = 0; way_it < waypoint_list.size() && ret_val; way_it++) {
    bool way_used = false;
    for (uint i = 0; i < st.by.size() && ret_val; i++) {
      RealVector aux;
      // Convert to 2D
      aux.push_back(st.by.at(i).at(0));
      aux.push_back(st.by.at(i).at(1));
      
      if (waypoint_list[way_it] == aux) {
	double value = 0.0;
	functions::FormattedTime time;
	if (i == st.by.size() - 1) {
	  time = st.time;
	  value = st.value;
	} else {
	  // Get the time and value till this point
	  for (uint j = 0; j < i; j++) {
	    time = time + st.by.at(j).distance(st.by.at(j + 1)) / uav->v_ref;
	    value += st.by.at(j).distance(st.by.at(j + 1));
	  }
	}
	ret_val = urm->assignUAV(aux, uav->id, value, time);
	way_used = true;
      } 
      
    }
    if (!way_used) {
      // If the way is not in the by vector--> unassign it!
      urm->unassignUAV(way_it, uav->id);
    }
  }
  
  return ret_val;
}

bool ExpandingTreePlanner::getPossibleUpdrafts(const TreeState& st) const {
  bool ret_val = true;
  
  for (unsigned int i = 0; i < possible_updrafts.size() && ret_val; i++) {
    const RealVector &way = possible_updrafts.at(i).center;
    bool way_used = false;
    for (uint i = 0; i < st.by.size() && ret_val; i++) {
      RealVector aux;
      // Convert to 2D
      aux.push_back(st.by.at(i).at(0));
      aux.push_back(st.by.at(i).at(1));
      
      if (way == aux) {
	double value = 0.0;
	functions::FormattedTime time;
	if (i == st.by.size() - 1) {
	  time = st.time;
	  value = st.value;
	} else {
	  // Get the time and value till this point
	  for (uint j = 0; j < i; j++) {
	    time = time + st.by.at(j).distance(st.by.at(j + 1)) / uav->v_ref;
	    value += st.by.at(j).distance(st.by.at(j + 1));
	  }
	}
	ret_val = urm->assignUAV(aux, uav->id, value, time);
	way_used = true;
      } 
      
    }
    if (!way_used) {
      // If the way is not in the by vector--> unassign it!
      urm->unassignUAV(way, uav->id);
    }
  }
  
  return ret_val;
  
}


int ExpandingTreePlanner::getNearestThermal(const functions::RealVector &pos) const
{
  int ret = urm->getNearestThermal(pos);
  
  if (ret >= (int)updraft_vector.size()) {
    ret = -1;
  }
  
  return ret;
}

bool ExpandingTreePlanner::getPlanFromBy(const vector< RealVector >& by, simulator::FlightPlan& output_plan, 
					 const TreeState& initial_state)
{
 //  output_plan.clear(); NOTICE: the plan has been already cleared in the calling function
  Point3D p(initial_state.pos);
  p.z = initial_state.altitude;
  FormattedTime t = uav->curr_time;
  output_plan.push_back4d(p, t);
  bool ret_val = true;
  double altitude = initial_state.altitude;
  RealVector ant_pos = initial_state.pos;
  int ant_updraft = urm->getUpdraftFromLocation(ant_pos);
  
  for (uint i = 0; i < by.size() && ret_val; i++) {
    RealVector pos2d;
    pos2d.push_back(by.at(i).at(0));
    pos2d.push_back(by.at(i).at(1));
    int curr_up = urm->getUpdraftFromLocation(pos2d);
    bool final_up_flag = false;
    
    if (curr_up > -1 && ant_updraft > -1 && i > 0) {
      p = urm->getFinalUpdraftTime(ant_pos, altitude, ant_updraft, t, *uav);
      altitude = p.z;
      final_up_flag = true;
    } else {
      p = uav->getArrivalTimeAltitude(ant_pos, pos2d, altitude, t);
      altitude = p.z;
    }
    
    output_plan.push_back4d(p, t);
    output_plan.setReachAltitude(i + 1, final_up_flag);
    if (final_up_flag) {
      output_plan.setClimbRate(i + 1, uav->getAscendingRate(urm->getUpdrafts().at(curr_up).getWindSpeed()));
    }
    
    ant_updraft = curr_up;
    ant_pos = pos2d;
  }
  
  cout << "CR: " ;
  for (uint i = 0; i < output_plan.size(); i++) {
    
    cout << " " << output_plan.getClimbRate(i) << endl;
  }
  cout << endl;
  
  return ret_val;
}

void ExpandingTreePlanner::setUpdrafts(vector< Updraft > new_ups)
{
  updraft_vector = new_ups;
}

int ExpandingTreePlanner::isInPossibleUpdraft(const RealVector& v, const simulator::FlightPlan& fp)
{
  int ret = -1;
  RealVector pos;
  pos.push_back(v.at(0));
  pos.push_back(v.at(1));
  
  if (fp.size() > 3) {
    for (unsigned int i = 0; i < possible_updrafts.size() && ret < 0; i++) {
      PossibleUpdraft &p = possible_updrafts.at(i);
      double dist_1 = p.p1.distance(pos);
      double dist_2 = p.p2.distance(pos);
      double dist_m = p.center.distance(pos);
      double dist = p.p1.distance(p.center);
      
      if ( dist_m < dist && (dist_1 < dist * 1.1 || dist_2 < dist * 1.1 )) {
	ret = i;
      }
    }
  }
  return ret;
}


}