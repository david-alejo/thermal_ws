#include "glider_planner/SoaringPlannerFactory.h"
#include "glider_planner/ExpandingTreePlanner.h"

namespace glider_planner {

SoaringPlannerFactory::SoaringPlannerFactory()
{
}

SoaringPlanner *SoaringPlannerFactory::createFromBlock(ParseBlock &block) const {
  SoaringPlanner *ret = NULL;
  try {
    if (block.hasProperty("type") && block("type").as<string>() == "A*") {
      Planner p;
      ret = p.createFromBlock(block);
    } else if (block.hasProperty("type") && block("type").as<string>() == ExpandingTreePlanner::type) {
      ExpandingTreePlanner p;
      ret = p.createFromBlock(block);
    }
  } catch (std::exception &e) {
    std::cerr << "SoaringPlannerFactory::createFromBlock --> Error while loading data. Content:" << e.what() << std::endl;
    ret = NULL;
  }

  return ret;
}

}