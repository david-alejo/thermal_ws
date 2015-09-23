#ifndef EXPANDINGTREEPLANNER_H
#define EXPANDINGTREEPLANNER_H

#include "SoaringPlanner.h"
#include "TreeState.h"

namespace glider_planner {

//! @brief Planner based on expanding tree algorithm with maximum depth
class ExpandingTreePlanner : public SoaringPlanner
{
public:
  static std::string type;
  
  ExpandingTreePlanner();
  
  ~ExpandingTreePlanner();

  // -------------  Inherited methods ------------
  //! @brief Call to execute, in this case the initial location is already stored in the class
  //! @param output_plan Output plan
  //! @param updrafts The updrafts in the system
  //! @param slots Constrains in the use of the updrafts
  virtual bool execute(simulator::FlightPlan &output_plan);

  //! Pure Virtual Contructor. Controllers are complex enough to be always created from a Parse Block
  virtual SoaringPlanner *createFromBlock(ParseBlock &block) const;
  
  virtual ParseBlock *toBlock() const;

  //! @brief Returns a string that describes the class contents
  virtual std::string toString() const;

  // --------------- End of Inherited methods ------------

  // --------------- Type definitions
  typedef std::vector<TreeState> SuccessorVector;
  
  void setPossibleUpdrafts(const std::vector<PossibleUpdraft> &new_p) {possible_updrafts = new_p;}
    void setUpdrafts(std::vector< simulator::Updraft > getUpdrafts);
    
  int isInPossibleUpdraft(const functions::RealVector &v, const simulator::FlightPlan &fp);

protected:
  int depth; // Maximum depth allowed in DFS
  std::vector<functions::RealVector> waypoint_list;
  std::vector<simulator::Updraft> updraft_vector;
  std::vector<PossibleUpdraft> possible_updrafts;
  std::list<TimeSlot> time_slots;
  simulator::FlightPlan last_plan;
  double safety_coefficient;
  
  double thermal_obligation; // If the UAV gets closer to thermal than this distance --> it has to take it!! (profit resources)

  static const double MIN_DISTANCE; // Distance when the planner skips the waypoint and goes to the next one.
  
  void dispose();

  //! @brief Performs an informed search
  bool informedSearch(TreeState node, int curr_depth, TreeState &best);

  //! @brief Distinguishes if the state is a final state (maximum depth reached or goal state)
  //! @retval true Final state has been reached
  //! @retval false Final state has not been reached
  bool isFinalState(TreeState node);

  //! @brief Evaluates the score of the state so far
  TreeState evaluation(TreeState nodo);
  
  inline void setHome(TreeState n_home) { home = n_home; }

  //! @brief Modifies the list of possible successors
  //! @param node The root node
  //! @param first If true --> it is the first call --> the by field will be the same node. If false --> inherits by from root
    SuccessorVector getSuccessors(glider_planner::TreeState nodo, bool first = false);

  //! @brief Gets the path with lowest cost
  bool minimizer(int curr_depth, TreeState &best_successor);

  //! @brief Gets the file checker. \NOTE The return value has to be freed.
  //! @return The file checker.
  static Checker *getChecker();

  //! @brief Gets the distance to the closest waypoint
  double distToClosestWaypoint(const functions::RealVector &pos) const ;

  //! Actualizes the by field in a node. Designed for waypoint points
  TreeState actualizeNode(const TreeState &nodo, const functions::RealVector &pos,
                                 double distance, bool first = false);
  
  //! Actualizes the by field in a node. Designed for thermal points
  //! @param nodo The node t actualize
  //! @param x The ID of the thermal
  //! @param distance Distance to the thermal
  //! @param first Indicates whether if it is the first expansion or not
  TreeState actualizeThermalNode(const TreeState &nodo, int x,
                                 double distance, bool first = false); 
  
  
  TreeState actualizePossibleThermalNode(const glider_planner::TreeState& nodo, int x, bool first = false);
  
  //! Returns true if the updraft is included in the plan
  //! @param x ID of the updraft
  bool isUpdraftIncludedInPlan(unsigned int x);
  
  int inUpdraft(TreeState &st) const;
  
  //! @brief Reserves the waypoint indicated in the flight plan, if necessary
  bool getWaypoints(const TreeState &st) const;
  
  //! @brief Reserves the possible updrafts indicated in the flight plan, if necessary
  bool getPossibleUpdrafts(const TreeState &st) const;
  
  int getCloseThermal(const TreeState &st) const;
  
private:
    bool inupdraft;
    int consec_descending;
    int max_consec_descending;
    double dist_go_updraft;
    
    TreeState last_node;
    
    TreeState home;
    
    //! @brief Gets the nearest thermal from a position.
    //! @retval -1 No updrafts in vector
    //! @return The ID of the nearest updraft
    int getNearestThermal(const functions::RealVector &pos) const;
    
    bool getPlanFromBy(const std::vector< functions::RealVector >& by, 
		       simulator::FlightPlan& output_plan, const glider_planner::TreeState& initial_state);
};

}

#endif // EXPANDINGTREEPLANNER_H

