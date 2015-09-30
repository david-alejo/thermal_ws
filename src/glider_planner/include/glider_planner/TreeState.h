#ifndef TREESTATE_H
#define TREESTATE_H

#include "BasicState.h"
#include "UAV.h"
#include <functions/RealVector.h>

namespace glider_planner {

class TreeState:public BasicState
{
public:
  //! Default constructor
  TreeState();

  //! Constructor from UAV
  TreeState(const UAV *uav);
  
  //! Copy constructor
  TreeState (const TreeState &s);
  
  void init (const TreeState &s);

  ~TreeState();

  virtual std::string toString() const;

  functions::RealVector pos;
  double dist; // Distance to the state in km
  double value; // Total value of the node
  std::vector<functions::RealVector> by; // The whole node list of the plan in order
//   int control; // Control value (is a final node?, etc.)
  
  TreeState operator = (const TreeState& s);
};

}

#endif // TREESTATE_H
