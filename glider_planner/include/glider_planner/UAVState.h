#ifndef UAVSTATE_H
#define UAVSTATE_H
#include <boost/concept_check.hpp>

namespace glider_planner {

typedef struct 
{
  double voltage;
  double percent; 
  double airspeed; 
  double ascrate;
  double throttle;
  double altitude;
}UAVState;

}

#endif

