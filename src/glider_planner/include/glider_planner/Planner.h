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


#ifndef PLANNER_H
#define PLANNER_H

#include <list>
#include <vector>
#include <set>
#include <simple_graph/SimpleGraph.h>
#include <functions/RealVector.h>
#include <simulator/Updraft.h>
#include "BasicState.h"
#include "TimeSlot.h"
#include "UAV.h"
#include "URM.h"
#include "SoaringPlanner.h"

#include <simulator/FlightPlan.h>

namespace glider_planner {

typedef simple_graph::SimpleGraph<functions::RealVector, double> Graph;

//! @brief Simple planner based on A* Algorithm
class Planner:public SoaringPlanner
{
public:
  static const std::string type;
  typedef std::vector<simulator::Updraft> updraft_vec;
  
    //! @brief Returns 
    Planner();
    
    //! @brief Copy constructor
    Planner(const Planner& other);

    //! @brief Assignment operator
    Planner &operator= (const Planner &right);
    
    //! @brief Constructor from filename
    //! @param filename the file to be loaded
    //! @brief Initializer from ParseBlock. Can throw exceptions!!
    //! @param block The parse block
    virtual SoaringPlanner *createFromBlock(ParseBlock &block) const;
    
    //! @brief Destructor
    virtual ~Planner();

    //! @brief Makes recursive calls to the planner in order to get the plan to all waypoints
    //! @param initial_loc Initial 3D coordinates of the vehicle. If a 4D is given --> the init time is the fourth coord (0.0 by default)
    //! @param output_plan Output plan 
    //! @param init_time Simulation start time
    bool recursivePlan(const functions::RealVector& initial_loc, simulator::FlightPlan &output_plan);
        
    //! @brief Implementation of the basic_A_star algorithm
    //! @param init_id Initial 3D coordinates of the vehicle 
    //! @param goal ID of the goal in the graph
    //! @param output_plan Output plan 
    //! @param prev_node Previous node of the init_id. Useful in recursive plans
    //! @param alpha Weighting parameter that is used to calculate f scores
    bool basic_A_star(int init_id, const BasicState &starting_state,
                      simulator::FlightPlan &output_plan,
                      const updraft_vec &updrafts, int prev_node = -1);
    
    //! @brief Call to execute, in this case the initial location is already stored in the class
    //! @param output_plan Output plan
    //! @param updrafts The updrafts in the system
    //! @param slots Constrains in the use of the updrafts
    virtual bool execute(simulator::FlightPlan &output_plan);

    
    //! @brief Returns a string that describes the class contents
    virtual std::string toString() const;
    
    //! @brief Transfer the useful info into a Block. This block can be loaded again generating another similar class
    //! @return A loadable block
    virtual ParseBlock *toBlock() const;
    
    //! @brief Returns the number of nodes of the generated graph
    inline int nNodes() const {
      return graph.nVertices();
    }
    
    // --------------------- Function necessary to access to the members of the class -----------------

    //! @brief Gets the generated graph of the system
    //! @return A const reference to the graph
    inline const Graph &getGraph() const {return graph;}
    
    //! @brief Returns true if the data has been successfully loaded
    inline bool isDataLoaded() const {return data_loaded;}
    
    //! @brief Gets the identificator of the node in the graph related to a location
    int getNodeID(const functions::Point3D &p);
    
    //! @brief Adds constrains to the current constrain list
    inline void addConstrains(const TimeSlotList &new_constrains) {
      constraint_list.insert(constraint_list.end(), new_constrains.begin(), new_constrains.end());
    }
    
    //! @brief Sets the constrain list
    inline void setConstrains(const TimeSlotList &new_constrains) {
      constraint_list = new_constrains;
    }
    
    inline const functions::RealVector &getLowerWorld() const {return lower_world;}
    inline const functions::RealVector &getUpperWorld() const {return upper_world;}

private:
  Graph graph;
  int first_updraft_id;
  double alpha; // This value indicates is used to calculate f: f = alpha * g + (1 - alpha) * h
  
  // Turning constants
  double min_edge_length;
  double max_turning_angle;
  
  // If set to true --> debug information is set to std::cout
  double minimum_altitude_coefficient;
  double grid_length;
  
  std::set<int> grid_black_list; // Grid nodes not to be used
  functions::RealVector lower_world;
  functions::RealVector upper_world;

  std::list<TimeSlot> constraint_list;
  
  // Status flag. Set to true if the system data has been successfully loaded
  bool data_loaded;
  
  //! @brief Basic initializer. Stores the default values
  void init();
  
  //! @brief Gets the file checker. \NOTE The return value has to be freed.
  //! @return The file checker.
  static Checker *getChecker();

  //! @brief Generates the graph that will be used in for planning purposes
  //! @brief Recommended constructor
  //! @param waypoints Waypoint list to be achieved (not necessary in order)
  //! @param updrafts Updrafts known in the system
  bool generateGraph(const std::list<functions::RealVector> &waypoint_list,
                     const updraft_vec &updraft_list);
  
  //! @brief Returns the id of the first node that represents an updraft
  //! @param updraft_id The ID of the updraft 
  //! @return The id of the first node that represents an updraft
  int getFirstUpdraftNode(int updraft_id, const updraft_vec &updraft_list) const;

  //! @brief Connects a node with the more convenient node of the updraft
  //! @param init_id The ID of the node
  //! @param updraft_id The ID of the updraft 
  void connectToUpdraft(int node_id, int updraft_id, const updraft_vec &updraft_list);
  
  //! @brief Implements the heuristic function that evaluates the distance between nodes
  //! @param init_id The ID of the initial node
  //! @param goal_id The ID of the goal node 
  //! @return The distance between the two nodes
  double heuristic(int init_id, int goal_id);
  
  //! @brief Reconstructs a generated A* plan.
  //! @param parent_map The map that relates each node with its parent
  //! @param altitude_map The map that relates each node with its altitude
  //! @param init_id The ID of the starting node
  //! @param goal_id ID of the goal node
  simulator::FlightPlan reconstruct_plan( const std::map< int, int > parent_map,
                                          const std::map< int, BasicState>& altitude_map, int init_id, int goal_id);
  
  //! @brief Adds initial location to the graph and performs other operations to be done before A* algorithm
  //! @param initial_location The coordinates of the initial location
  bool prepareAlgorithm(int &init_id,
                        double &starting_altitude, const updraft_vec updraft_list);
  
  inline int getTotalUpdraftPoints(const updraft_vec &updraft_list) const {
    int cont = 0;
    
    
    for (unsigned int i = 0; i < updraft_list.size(); i++) {
      cont += updraft_list.at(i).max_loops * updraft_list.at(i).graph_points;
    }
    
    return cont;
  }
  
  inline int getUpdraftFromNode(int node, const updraft_vec &updraft_list) const {
    int ret_val = -1;
    int cont = node - first_updraft_id;
    updraft_vec::const_iterator u_it = updraft_list.begin();
    
    for(; cont > 0 && ret_val < (int)updraft_list.size(); ret_val++, cont -= u_it->graph_points * u_it->max_loops, u_it++ ) ;
    
    if (cont > 0 || ret_val >= (int)updraft_list.size()) {
      ret_val = -1;
    }
    
    return ret_val;
  }
  
  //! @brief updates the state when performing A* search
  void updateState(int init, int end, BasicState &state) const;
  
  //! @brief Checks if a grid vertex has to be added
  bool checkGridAddition(const functions::RealVector &vec,
                         const std::list<functions::RealVector> &waypoint_list,
                         const updraft_vec &up) const;
    
  //! @brief Gets the bounds of the grid)
  bool getGridBounds(functions::RealVector &min, functions::RealVector &max,
                     const std::list<functions::RealVector> &waypoint_list,
                     const updraft_vec &updraft_list);
  
//  int getNearestGridPoint(const functions::RealVector &p) const;
  
  //! @brief Adds an edge with a determinate weight if it does not cross any thermal being outside a thermal
  bool addEdgeWithWeight(const updraft_vec &updraft_list,
                         int index_1, int index_2, int updraft_id = -1, bool double_add = true);

  //! @brief Adds a regular grid to the graph
  //! @return The number of points that have been added
  int addGrid(const updraft_vec &updraft_list,
              const std::list<functions::RealVector> &waypoint_list);
  
  //! @brief Gets the distance to the closest updraft
  //! @return the distance to the closest updraft
  double getMinimumDistanceToUpdraft(int node, const updraft_vec updraft_list) const;

  void copyObj(const Planner &other);
};

}

#endif // PLANNER_H
