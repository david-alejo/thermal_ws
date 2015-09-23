#ifndef SOARINGPLANNERFACTORY_H
#define SOARINGPLANNERFACTORY_H

#include "SoaringPlanner.h"
#include "Planner.h"

namespace glider_planner {

class SoaringPlannerFactory
{
public:
  SoaringPlannerFactory();

  SoaringPlanner *createFromBlock(ParseBlock &block) const;

};

}

#endif // SOARINGPLANNERFACTORY_H
