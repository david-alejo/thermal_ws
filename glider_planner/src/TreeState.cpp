#include "glider_planner/TreeState.h"
#include <functions/functions.h>

using namespace std;

namespace glider_planner {

TreeState::TreeState():pos(2)
{
  by.clear();
}

TreeState::TreeState(const UAV *uav) {
  altitude = uav->current_location.at(2);
  time = uav->curr_time;
  pos.clear();
  pos.push_back(uav->current_location.at(0));
  pos.push_back(uav->current_location.at(1));
  value = 1e30;
//   control = 0; // Initialized
  dist = 0.0;
  by.clear();
}

TreeState::~TreeState() {
  pos.clear();
  by.clear();
}

TreeState::TreeState(const TreeState& s):BasicState(s)
{
  init(s);
}


TreeState TreeState::operator=(const TreeState& s)
{
  if(this != &s){ 
    // Check for self-assignment
    init(s);
  }
  return *this;
}

void TreeState::init(const TreeState& s)
{
  pos = s.pos;
  dist = s.dist;
  value = s.value;
  by = s.by;
//   control = s.control;
  BasicState::init(s);
}



string TreeState::toString() const {
  ostringstream os;

  os << BasicState::toString() << "\t"; // Call to the inherited method

  os << "2D Position = " << pos.toString() << "\t";
  os << "Value = " << value << "\t";
  os << "Dist = " << dist << "\t";
  os << "By = " << functions::printToStringVector(by);

  return os.str();
}

}