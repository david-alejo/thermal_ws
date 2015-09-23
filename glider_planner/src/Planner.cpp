/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "glider_planner/Planner.h"

#include <set>
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>
#include <functions/functions.h>

using namespace std;
using namespace functions;
using namespace simple_graph;
using namespace simulator;

namespace glider_planner {

const std::string Planner::type = "A*";

Planner::Planner()
{
  init();
}

Planner::~Planner()
{
  init();
}

Planner::Planner(const Planner& other)
{
  init();
  copyObj(other);
}

Planner &Planner::operator= (const Planner &right) {
  if (this != &right) {
    init();
    copyObj(right);
  }
  return *this;
}

SoaringPlanner *Planner::createFromBlock(ParseBlock& block) const
{
  Planner *p = new Planner;
  SoaringPlanner &sp = *p;

  Checker *check = getChecker();

  sp.init(block);
  try {
    block.checkUsing(check);

    p->minimum_altitude_coefficient = block("minimum_altitude_coefficient").as<double>();
    p->grid_length = block("grid_length").as<double>();
    p->alpha = block("alpha").as<double>();
    p->max_turning_angle = block("maximum_turning_angle").as<double>();
    if (block.hasProperty("upper_world")) {
      p->upper_world = block("upper_world").as<vector<double> >();
    }
    if (block.hasProperty("lower_world")) {
      p->lower_world = block("lower_world").as<vector<double> >();
    }

    //    if (uav_block.hasProperty("minimum_edge_length")) {
    //      min_edge_length = uav_block("minimum_edge_length").as<double>();
    //    }



//    // Optional parameters

//    // -------------- UAV parameters ----------------

//    if (waypoints_block_name.length() > 0 ) {
//      uav_block = block[waypoints_block_name.c_str()];
//    }
//    uav_block.checkUsing(check);

//    alpha_exploitation = uav_block("alpha").as<double>();
    
//    ParseBlock::Properties *waypoints = uav_block.getProperties("waypoint");
//    ParseBlock::Properties::iterator w_it = waypoints->begin();
//    waypoint_list.clear();
//    for ( ; w_it != waypoints->end(); w_it++) {
//      RealVector aux( (*w_it)->as<vector<double> >());
//      waypoint_list.push_back(aux);
//      if (debug) {
//	cout << "Loaded a waypoint: " << aux.toString() << endl;
//      }
//    }
    
  } catch (exception &e) {
    cerr << "Planner::createFromBlock --> Error while loading data. Content:" << e.what() << endl;
    delete p;
    p = NULL;
  }
//  generateGraph();
  if (p != NULL) {
    p->data_loaded = true;
  }
  delete check;
  
  return p;
}



Checker* Planner::getChecker() {
  Checker *check = new Checker;

  check->addProperty("alpha", new NTimes(1));
  check->addProperty("grid_length", new NTimes(1));
  check->addProperty("maximum_turning_angle", new NTimes(1));
  check->addProperty("minimum_altitude_coefficient", new NTimes(1));
  
  return check;
}

void Planner::init()
{
  // Default values here!!
  first_updraft_id = 0;
  alpha = 0.97;
  min_edge_length = 150.0;
  max_turning_angle = 2 * M_PI / 3;
  min_dist = 0.0;
  grid_length = 100;
  minimum_altitude_coefficient = 1.5;
  
  // Status and behaviour flags
  debug = false;
  data_loaded = false;
  
  // Clean vectors and lists
  graph.clear();
  grid_black_list.clear();
  upper_world.clear();
  lower_world.clear();
}

// TODO: Updraft connections?
bool Planner::generateGraph(const std::list< RealVector >& waypoint_list,
                            const std::vector< Updraft >& updraft_list)
{
  // Clear previous information
  graph.clear();
  
  // First generate the vertices
  int cont = 0;
  
  // One for each waypoint
  list<RealVector>::const_iterator w_it;
  for ( w_it = waypoint_list.begin(); w_it != waypoint_list.end(); w_it++, cont++) {
    if (debug) {
      cout << " Adding vertex: " << w_it->toString() << endl;
    }
    graph.addVertex(*w_it);
  }
  
  int first_grid_id = cont;
  
  cont = addGrid(updraft_list, waypoint_list);
  first_updraft_id = cont;
  
  unsigned int updraft_cont = 0;
  for ( ; updraft_cont < updraft_list.size(); updraft_cont++) {
    const Updraft &curr_updraft = updraft_list.at(updraft_cont);

//     int current_node_id = getFirstUpdraftNode(updraft_cont, updraft_list);
    // East vertex
    double pi_inc = M_PI / curr_updraft.graph_points * 2;
    RealVector offset;
    offset.push_back(curr_updraft.radius);
    offset.push_back(0.0); 
    graph.addVertex(curr_updraft.getLocation() + offset);
    cont++;
    for (int i = 1; i < curr_updraft.graph_points * curr_updraft.max_loops; i++) {
      // Rotate the vector (counterclockwise)
      double old_x = offset[0];
      offset[0] = offset[0]*cos(pi_inc) - offset[1]*sin(pi_inc);
      offset[1] = offset[1]*cos(pi_inc) + old_x*sin(pi_inc);
      graph.addVertex(curr_updraft.getLocation() + offset); 
      addEdgeWithWeight(updraft_list, cont - 1, cont, updraft_cont);
      cont++;
    }
    
    addEdgeWithWeight(updraft_list,
                      cont - 1, cont - curr_updraft.graph_points * curr_updraft.max_loops, updraft_cont);
  }
  
  // Complete the edges between each waypoint and each updraft. Only one connection between updraft and waypoint
  // TODO: and with ot her updrafts!!
  cont = 0;
  for ( w_it = waypoint_list.begin(); w_it != waypoint_list.end(); w_it++, cont++) {
    uint cont2 = 0;
    
    list<RealVector>::const_iterator w_it2 = w_it;
    w_it2++;
    
    for (cont2 = cont + 1; w_it2 != waypoint_list.end(); w_it2++, cont2++) {
      addEdgeWithWeight(updraft_list, cont, cont2);
    }
    
    // Then connect to each element in the grid
    for (cont2 = first_grid_id; (int)cont2 < first_updraft_id; cont2++) {
      if (grid_black_list.find(cont2) == grid_black_list.end()) {
	// If the grid point is not in the black list -> connect it
	addEdgeWithWeight(updraft_list, cont, cont2, -1, false);
      }
    }
    
    for ( cont2 = 0; cont2 != updraft_list.size(); cont2++) {
      connectToUpdraft(cont, cont2, updraft_list);
    }
  }
  
  // At last connect each point of the updraft with its nearest neighbor in the grid
  for (uint cont2 = 0; cont2 < updraft_list.size(); cont2++) {
    const Updraft &up = updraft_list.at(cont2);
    
    if (debug) {
      cout << "Connecting updrafts with the grid. Updraft id = " << cont2;
      cout << ". First updraft node = " << getFirstUpdraftNode(cont2, updraft_list) << endl;
      cout << "Updraft info: " << up.toString() << endl;
    }
    
    int first_up_id = getFirstUpdraftNode(cont2, updraft_list);
    for (int cont3 = 0; cont3 < up.graph_points; cont3++) {
      for (int cont4 = waypoint_list.size(); cont4 < first_up_id; cont4++) {
	if ( grid_black_list.find(cont4) == grid_black_list.end() ) {
	  connectToUpdraft(cont4, cont2, updraft_list);
	}
      }
    }
  }
  
  // The graph has been succesfully created
  return true;
}

bool Planner::recursivePlan(const functions::RealVector& initial_loc, FlightPlan& output_plan)
{
  output_plan.clear();
  
  RealVector v;
  v.push_back(0.0);
  v.push_back(0.0);

  bool ret_val = true;

  // Make the first call

  if (debug) {
    cout << "Planner::recursivePlan --> Initial call. Alpha = " << alpha << "\t Initial location: " << initial_loc.toString() << endl;
  }
  
  if (ret_val) {
//    ret_val = execute(initial_loc, 0, output_plan, alpha);
  }
  
  // Then begin calling the planner for the next waypoints
//  for (int cont = 1; cont < waypoint_list.size() && ret_val; cont++) {
//    FlightPlan aux_plan;
//    double init_time = output_plan.getETA(output_plan.size() - 1);
    
//    Point3D &last_point = output_plan.at(output_plan.size() -1);
//    Point3D &prelast_point = output_plan.at(output_plan.size() - 2);
//    BasicState state(init_time, last_point.z);
    
//    int pre_last = getNodeID(prelast_point);

//    if (debug) {
//      cout << "Planner::recursivePlan --> Call number " << cont + 1 << ". Alpha = " << alpha;
//      cout << "Initial state: " << state.toString() << endl;
//    }
        
//    ret_val = basic_A_star(cont - 1, state, cont, aux_plan, pre_last, alpha);
    
//    for (int cont2 = 1;cont2 < aux_plan.size(); cont2++) {
      // Add all aux plan nodes but the first
//      output_plan.push_back4d(aux_plan.at(cont2), aux_plan.getETA(cont2));
//    }
//  }
  
  return ret_val;
}


bool Planner::execute(simulator::FlightPlan &output_plan) {
  // Preparatives
  output_plan.clear();
  std::vector<Updraft> updrafts = urm->getUpdrafts();
  TimeSlotList slots = urm->getConstraints();
  int init_id;
  double starting_altitude;
  
  if (!prepareAlgorithm(init_id, starting_altitude, updrafts)) {
    return false;
  }
  
//   double init_time = 0.0;
  
  FormattedTime time;
  time.getTime();
  
  BasicState init_state (time, starting_altitude);
  
  return basic_A_star(init_id, init_state, output_plan, updrafts, -1);
}

bool Planner::basic_A_star (int init_id, const BasicState &starting_state,
                            FlightPlan& output_plan, const vector<Updraft> &updrafts,
                            int prev_node)
{
  bool ret_val = false;
  std::map<int , int> parent_map; // Come from map
  std::map<int, BasicState> state_map; // Keeps track of the altitude in each node
  state_map[init_id] = starting_state;
  std::set<int> closed_set; // The set of nodes already evaluated.
  std::set<int> open_set; // The set of tentative nodes to be evaluated, initially containing the start node
  open_set.insert(init_id);
  BasicState current_state = starting_state;
  int goal = 0; // TODO: get the closest goal

  // Initialization of the A* algorithm
  std::map<int , double> g_score, h_score, f_score; // Score from maps
  g_score[init_id] = 0.0;    // Cost from start along best known path.
  h_score[init_id] = heuristic(init_id, goal);
  // Estimated total cost from start to goal through y.
  f_score[init_id] = g_score[init_id] + h_score[init_id];
  
  parent_map[init_id] = prev_node;
 
  while (!open_set.empty() && ! ret_val) {
     // Search the node with lowest value of f
     int low_cost = 0; // id of the node in the open_list with less g
     double lowest_cost = 1e100;
     set<int>::iterator it = open_set.begin();
     
     for (int cont = 0; it != open_set.end(); it++, cont++) {
       if ( f_score[*it] < lowest_cost) {
	 low_cost = *it;
	 lowest_cost = f_score[*it];
	 current_state = state_map[low_cost];
       }
     }
     
     if (low_cost == goal) {
       if (debug) {
	cout << "Goal node reached successfully\n";
       }
       
       // Get the obtained flight plan
       output_plan = reconstruct_plan(parent_map, state_map, init_id, goal);
       ret_val = true;
     } else {
     
      open_set.erase(low_cost);
      closed_set.insert(low_cost); 
     
      std::list<int> neighbor_list = graph.getEdges(low_cost);
      std::list<int>::iterator n_it = neighbor_list.begin();
     
      // Get the previous edge in order to calculate the turning angle constrains
      RealVector prev_edge;
      double prev_edge_length; 
      if ( parent_map[low_cost] >= 0 && parent_map[low_cost] < graph.nVertices()) {
	prev_edge = graph.getVertexContent(low_cost) - graph.getVertexContent(parent_map[low_cost]); // Previous edge
	prev_edge_length = prev_edge.norm();
      }
      
      for ( ; n_it != neighbor_list.end(); n_it++) {
        // If it is in the closed set --> do not take it into account
	if ( closed_set.find(*n_it) != closed_set.end()) {
	  continue;
	}

	// If we have just entered in an updraft, we cannot exit it immediately
	if ( getUpdraftFromNode(low_cost, updrafts) > -1 &&
	     getUpdraftFromNode(parent_map[low_cost], updrafts) != getUpdraftFromNode(low_cost, updrafts) &&
	     getUpdraftFromNode(low_cost, updrafts) != getUpdraftFromNode(*n_it, updrafts) ) {
	  continue;
	}

	// Get altitude gain
	double altitude_gain;
	graph.getEdgeContent(low_cost, *n_it, altitude_gain);
	double &current_altitude = current_state.altitude;
	
	// If the final altitude in the node is not enough. This altitude must be minimum_altitude_coefficient times the
	// distance to the closest updraft by the tan of gamma
	double alt_const = uav->min_alt + getMinimumDistanceToUpdraft(*n_it, updrafts) *
			   tan(uav->gamma) * minimum_altitude_coefficient;
	
	if (current_altitude - altitude_gain <  alt_const) {
	  if (debug) {
// 	    cout << "Could not reach the node " << *n_it << ". Not enough altitude: " << current_altitude - altitude_gain << " < " << min_alt << endl;
	  }
	  continue;
	}
	
	if ( getUpdraftFromNode(low_cost, updrafts) == getUpdraftFromNode(*n_it, updrafts) &&
	     getUpdraftFromNode(low_cost, updrafts) >= 0) {
	  // We are trying to move inside an updraft --> have we reached max_altitude?
	  vector<Updraft>::const_iterator uit = updrafts.begin();
	  for (int cont = 0; cont < getUpdraftFromNode(low_cost, updrafts); cont++, uit++);
	  if ( current_altitude - altitude_gain > uit->max_height ) {
	    if (debug) {
// 	      cout << "Could not reach the node " << *n_it << ". Max altitude of the updraft " << getUpdraftFromNode(low_cost) << "reached: ";
// 	      cout << current_altitude - altitude_gain << " > " << uit->max_height << endl;
	    }
	    continue;
	  }
	}
	
	// Consider temporal constrains
	// Get altitude gain
	BasicState new_node_state = current_state;
	updateState(low_cost, *n_it, new_node_state);
	TimeSlotList::const_iterator c_it = constraint_list.begin();
	bool checked_ok = true;
	for (; c_it != constraint_list.end() && checked_ok; c_it++) {
	  int up_num = getUpdraftFromNode(*n_it, updrafts);
	  checked_ok = c_it->check(up_num, new_node_state.time,
				   new_node_state.altitude,
				   uav->getAscendingRate(updrafts.at(up_num).getWindSpeed()),
				   min_dist, updrafts.at(up_num).max_height, uav->id);
	}
	
	if(!checked_ok)
	  continue;
	
	// At last, turning angle constrains
	if ( parent_map[low_cost] >= 0 && parent_map[low_cost] < graph.nVertices()) {
	  RealVector curr_edge = graph.getVertexContent(*n_it) - graph.getVertexContent(low_cost);
	  
	  double max_angle = max_turning_angle;
	  
	  if (prev_edge_length < min_edge_length) {
	      max_angle = M_PI / 3; // The condition changes if the prev_edge is too short
	  }

	  if (prev_edge.angle(curr_edge) > max_angle) {
	    if (debug) {
// 	      cout << "Could not reach the nodes " << graph.getVertexContent(*n_it).toString();
// 	      cout << "\tFrom node " << graph.getVertexContent(low_cost).toString() << "and " << graph.getVertexContent(parent_map[low_cost]).toString();
// 	      cout << "\nBecause angle " << prev_edge.angle(curr_edge) << " the edges were: prev = " << prev_edge.toString();
// 	      cout << "and current = " << curr_edge.toString() << endl;
	    }
	    continue;
	  }
	}
	
	double weight;
	graph.getEdgeContent(low_cost, *n_it, weight);
	double tentative_g_score = g_score[low_cost] + weight;
	bool tentative_is_better = false;
	if ( open_set.find(*n_it) == open_set.end()) {
	  open_set.insert(*n_it);
	  tentative_is_better = true;
	} else if (tentative_g_score < g_score[*n_it]) {
	  tentative_is_better = true;
	}
       
	if (tentative_is_better) {
	  g_score[*n_it] = tentative_g_score;
	  h_score[*n_it] = heuristic(*n_it, goal);
	  f_score[*n_it] = alpha * g_score[*n_it] + (1 - alpha) * h_score[*n_it];
	  parent_map[*n_it] = low_cost;
	  
	  // Get altitude gain
	  BasicState new_node_state = current_state;
	  updateState(low_cost, *n_it, new_node_state);
	  // And add the result to the map
	  state_map[*n_it] = new_node_state;
	}
      }
    }
  }
     
  return ret_val;
}

void Planner::updateState(int init, int end, BasicState& state) const
{
  double altitude_gain;
  graph.getEdgeContent(init, end, altitude_gain);
  state.altitude -= altitude_gain;
  state.time = state.time + graph.getVertexContent(init).distance(graph.getVertexContent(end)) / uav->v_ref;
}


bool Planner::prepareAlgorithm(int &init_id, double &starting_altitude,
                               const vector<Updraft> updraft_list)
{
  const functions::RealVector &initial_loc = uav->current_location;
  if (initial_loc.size() < 3 || initial_loc.size() > 4) {
    cerr << "Planner::execute --> Error! Initial location must be a 3-dim or 4-dim vector.\n";
    return false;
  }
  starting_altitude = initial_loc.at(2);
  
  // Add the current point to the graph
  RealVector init_point2d;
  init_point2d.push_back(initial_loc.at(0));
  init_point2d.push_back(initial_loc.at(1));
  init_id = graph.addVertex(init_point2d);
 
  // Connect it with all waypoints
  for (int i = 0; i < first_updraft_id; i++) {
    if ( grid_black_list.find(i) == grid_black_list.end()) {
      addEdgeWithWeight(updraft_list, init_id, i, -1, false);
    }
  }
  
  // Connect it with the updrafts
  for (uint i = 0; i < updraft_list.size(); i++) {
    connectToUpdraft(init_id, i, updraft_list);
  }
  if (debug) {
    cout << "Planner::prepareAlgorithm. Init ID: " << init_id << endl ;
  }
  
  return true;
}


string Planner::toString() const
{
  ostringstream os;
  
  os << " Graph content: " << graph.toString() << endl;
  
  return os.str();
}

ParseBlock* Planner::toBlock() const
{
  ParseBlock *ret = new ParseBlock;
  
  // And the system parameters
  ret->setProperty("alpha", numberToString(alpha));
  ret->setProperty("upper_world", printVector(upper_world));
  ret->setProperty("lower_world", printVector(lower_world));
  
  // Put the waypoints
//  list<RealVector>::const_iterator w_it = waypoint_list.begin();
  
//  for (;w_it != waypoint_list.end();w_it++) {
//    ret->setProperty("waypoint", functions::printVector(*w_it));
//  }
  
  // Insert updraft list
//  vector<Updraft>::const_iterator u_it = updraft_list.begin();
//  for (;u_it != updraft_list.end(); u_it++) {
//    ret->setBlock("updraft", u_it->toBlock());
//  }
  
//  // At last, temporal constrains
//  list<TemporalConstrain>::const_iterator c_it = constraint_list.begin();
//  for (;c_it != constraint_list.end(); c_it++) {
//    ret->setBlock("constrain", c_it->toBlock());
//  }
  
  // Then the flags
  ret->setProperty("debug", boolToString(debug));
  ret->setProperty("grid_length", numberToString(grid_length));
  ret->setProperty("min_edge_length", numberToString(min_edge_length));
  ret->setProperty("minimum_altitude_coefficient", numberToString(minimum_altitude_coefficient));
  
  return ret;
}

int Planner::getFirstUpdraftNode(int updraft_id, const std::vector<Updraft> &updraft_list) const
{
  int id = first_updraft_id;
    
  int cont = 0;
    
  for (; cont < updraft_id ; cont++) {
    id += updraft_list[cont].max_loops * updraft_list[cont].graph_points;
  }
    
  return id;
}

void Planner::connectToUpdraft(int node_id, int updraft_id, const vector<Updraft> &updraft_list)
{
  RealVector node_location = graph.getVertexContent(node_id);
  
  const Updraft &u_it = updraft_list.at(updraft_id);
  
  RealVector to_updraft = node_location - u_it.getLocation();
  double angle = atan2(to_updraft[1], to_updraft[0]);
  double pi_inc = M_PI/u_it.graph_points * 2;
  
  // See which vertex on the updraft has to be connected
  for (int i = 0; i < u_it.graph_points; i++, angle -= pi_inc ) {
    
    if (angle < -M_PI) {
      angle += 2 * M_PI;
    }
  
    // Only one entrance and exit edge
    addEdgeWithWeight(updraft_list, node_id, getFirstUpdraftNode(updraft_id, updraft_list) + i);

    // But it has to exist one exit edge for each loop
    for (int cont = 1; cont < u_it.max_loops; cont++) {
      addEdgeWithWeight(updraft_list, getFirstUpdraftNode(updraft_id, updraft_list) + i + cont * u_it.graph_points, node_id, -1, false);
    }
  }
}

double Planner::heuristic(int init_id, int goal_id)
{
  // Calculate the distance between
  RealVector v = graph.getVertexContent(init_id);
  return (v.distance(graph.getVertexContent(goal_id)));
}

simulator::FlightPlan Planner::reconstruct_plan (const std::map< int, int > parent_map, const std::map<int, BasicState> &state_map, int init_id, int goal_id)
{
  int current_node = goal_id;
  FlightPlan ret;
  Point3D curr_point;
  FormattedTime curr_time;
      
  while (current_node != init_id) {
    curr_point.init(graph.getVertexContent(current_node));
    std::map<int, BasicState>::const_iterator alt_it = state_map.find(current_node);
    if ( alt_it != state_map.end()) {
      curr_point.z = alt_it->second.altitude;
      curr_time = alt_it->second.time;
    }
    ret.insert4d(ret.begin(), curr_point, curr_time);
    // Actualize current node
    current_node = parent_map.find(current_node)->second;
  }
  
  curr_point.init(graph.getVertexContent(init_id));
  std::map<int, BasicState>::const_iterator alt_it = state_map.find(init_id);
    if ( alt_it != state_map.end()) {
      curr_point.z = alt_it->second.altitude;
      curr_time = alt_it->second.time;
    }
  ret.insert4d(ret.begin(), curr_point, curr_time);
  
  return ret;
}

bool Planner::checkGridAddition(const functions::RealVector& vec,
                                const std::list<RealVector> &waypoint_list,
                                const std::vector<Updraft> &updraft_list) const
{
  bool ret_val = true;
  list<RealVector>::const_iterator it = waypoint_list.begin();
  for (; it != waypoint_list.end() && ret_val; it++) {
    if (it->distance(vec) < grid_length / 4) {
      ret_val = false;
    }
  }
  
  for (unsigned int cont = 0; cont < updraft_list.size() && ret_val; cont++) {
    const Updraft &upd = updraft_list.at(cont);
    if ( vec.distance(upd.getLocation()) < upd.radius * sqrt(2.0)) {
      ret_val = false;
    }
  }
  
  return ret_val;
}

bool Planner::getGridBounds(RealVector& min, RealVector& max,
                            const std::list<RealVector> &waypoint_list,
                            const std::vector<Updraft> &updraft_list)
{
  double min_x = 1e100;
  double min_y = 1e100;
  double max_x = -1e100;
  double max_y = -1e100;
  
  min.clear();
  max.clear();
  
  if (upper_world.size() >= 2 && lower_world.size() >= 2) {
    min = lower_world;
    max = upper_world;
    min_x = min.at(2);
    min_y = min.at(1);
    max_x = max.at(2);
    max_y = max.at(1);
  } else {
    min.push_back(0.0);
    min.push_back(0.0);
    max.push_back(0.0);
    max.push_back(0.0);
  }
  
  list<RealVector>::const_iterator it = waypoint_list.begin();
  for (; it != waypoint_list.end(); it++) {
    min_x = minimum(min_x, it->at(0));
    min_y = minimum(min_y, it->at(1));
    max_x = maximum(max_x, it->at(0));
    max_y = maximum(max_y, it->at(1));
  }
  
  for (unsigned int cont = 0; cont < updraft_list.size(); cont++) {
    RealVector vec(updraft_list.at(cont).getLocation());
    min_x = minimum(min_x, vec.at(0));
    min_y = minimum(min_y, vec.at(1));
    max_x = maximum(max_x, vec.at(0));
    max_y = maximum(max_y, vec.at(1));
  }
  
  min[0] = min_x - 1.2 * grid_length;
  min[1] = min_y - 1.2 * grid_length;
  max[0] = max_x + 1.2 * grid_length;
  max[1] = max_y + 1.2 * grid_length;
  
  lower_world = min;
  upper_world = max;
  return true;
}

//int Planner::getNearestGridPoint(const RealVector& p) const
//{
//  double min_dist = 1e100;
//  int best_id = -1;
  
//  for (unsigned int i = waypoint_list.size(); i < first_updraft_id; i++) {
//    if (min_dist > graph.getVertexContent(i).distance(p) && grid_black_list.find(i) == grid_black_list.end()) {
//      min_dist = graph.getVertexContent(i).distance(p);
//      best_id = i;
//    }
//  }
  
//  return best_id;
//}

bool Planner::addEdgeWithWeight(const vector<Updraft> &updraft_list, int index_1, int index_2, int updraft_id,
                                bool double_add)
{
  functions::RealVector pos_1(graph.getVertexContent(index_1));
  functions::RealVector pos_2(graph.getVertexContent(index_2));
    
  double weight = pos_1.distance(pos_2) * tan(uav->gamma); // Regular weight
    
  if (updraft_id > -1) {
    weight -= sqrt(2.0) * updraft_list.at(updraft_id).radius / uav->v_ref * updraft_list.at(updraft_id).getWindSpeed();
  }
    
  bool add_edge = true;
    
  if (updraft_id < 0) {
    for (uint cont = 0; cont < updraft_list.size() && add_edge; cont++) {
      functions::RealVector vec(updraft_list.at(cont).getLocation());
      
      double min_dist = updraft_list.at(cont).radius;
      
      if (getUpdraftFromNode(index_1, updraft_list) >= 0 || getUpdraftFromNode(index_2, updraft_list) >= 0) {
	min_dist *= 0.9;
      } else {
	min_dist *= 1.1;
      }
      
      if (vec.distanceToSegment(pos_1, pos_2) < min_dist) {
	add_edge = false;
      }
    }
  }
    
  if (add_edge) {
    if (debug) {
//       std::cout << "Adding edge from " <<index_1 << " to " << index_2 << " with weight: " << weight << std::endl;
    }
    graph.addEdge(index_1, index_2, weight);
    if (double_add) {
      graph.addEdge(index_2, index_1, weight);
    }
  }
  return add_edge;
}

int Planner::addGrid(const std::vector<Updraft> &updraft_list, const std::list<RealVector> &waypoint_list)
{

  // Add the regular spaced grid to the system
  RealVector min_bounds, max_bounds;
  getGridBounds(min_bounds, max_bounds, waypoint_list, updraft_list);
    
  int x_nodes = (int)floor((max_bounds.at(0) - min_bounds.at(0)) / grid_length);
  int y_nodes = (int)floor((max_bounds.at(1) - min_bounds.at(1)) / grid_length);
    
  if (debug) {
    cout << "Grid bounds: " << min_bounds.toString() << " to " << max_bounds.toString() << endl;
    cout << "X nodes: " << x_nodes << ".\t Y nodes: " << y_nodes << endl;
  }
    
  int cont_y = 0;
  int cont_x = 0;
  int cont = waypoint_list.size();
  grid_black_list.clear();
    
  for (double x_pos = min_bounds.at(0); x_pos < max_bounds.at(0); x_pos += grid_length, cont_x++) {
    for (double y_pos = min_bounds.at(1); y_pos < max_bounds.at(1); y_pos += grid_length, cont_y++, cont++) {
      RealVector pos;
      pos.push_back(x_pos);
      pos.push_back(y_pos);
	
      // Check if the grid point has to be added (is not inside a thermal and 
      graph.addVertex(pos);

      if (debug) {
// 	cout << " Adding grid vertex: " << pos.toString() << endl;
      }
	
      if (checkGridAddition(pos, waypoint_list, updraft_list)) {
	  // Connect with the left grid
	 
	if (cont_x > 0) {
	  if ( grid_black_list.find(cont - y_nodes) == grid_black_list.end() ) {
	    addEdgeWithWeight(updraft_list, cont, cont - y_nodes);
	  }
	}
	// Connect with the upper grid
	if (cont_y > 0) {
	  if ( grid_black_list.find(cont - 1) == grid_black_list.end() ) {
	    addEdgeWithWeight( updraft_list, cont, cont - 1);
	  }
	}
	
	if (cont_x > 0 && cont_y >0) {
	  if ( grid_black_list.find(cont - y_nodes - 1) == grid_black_list.end() ) {
	    addEdgeWithWeight(updraft_list, cont, cont - 1 - y_nodes);
	  }
	}
      } else {
	grid_black_list.insert(cont);
	if (debug) {
	  cout << " Node: " << cont << " to the black list!.\n";
	}
      }
    }
  }
  
  return cont;
}

int Planner::getNodeID(const Point3D& p)
{
  int ret;
  bool found = false;
  
  for (ret = 0; ret < graph.nVertices() && !found; ret++) {
    RealVector v = graph.getVertexContent(ret);
    
    if (v.at(0) == p.x && v.at(1) == p.y) {
      found = true;
    }
  }
  
  return found ? ret - 1 : -1;
}

double Planner::getMinimumDistanceToUpdraft(int node, const std::vector<Updraft> updraft_list) const
{
  double min_dist = 1e100;
  const RealVector &v = graph.getVertexContent(node);
  
  for (unsigned int i = 0; i < updraft_list.size(); i++) {
    min_dist = min(min_dist, updraft_list.at(i).getLocation().distance(v) - updraft_list.at(i).radius);
  }
  
  return max(min_dist,0.0);
}

void Planner::copyObj(const Planner &other) {
  // Complex types
  graph = other.graph;
  grid_black_list = other.grid_black_list;
  upper_world = other.upper_world;
  lower_world = other.lower_world;

  // Simple types
  max_turning_angle = other.max_turning_angle;
  first_updraft_id = other.first_updraft_id;
  alpha = other.alpha;
  min_edge_length = other.min_edge_length;
  max_turning_angle = other.max_turning_angle;

  // Flags and related data
  debug = other.debug;
  minimum_altitude_coefficient = other.minimum_altitude_coefficient;
  grid_length = other.grid_length;
  data_loaded = other.data_loaded;
}


}
