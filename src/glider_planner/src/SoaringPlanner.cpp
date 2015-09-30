#include "glider_planner/SoaringPlanner.h"
#include "glider_planner/UAVFactory.h"
#include "functions/functions.h"

using namespace std;

namespace glider_planner {

void SoaringPlanner::init( ParseBlock &block, URM *new_urm) {
  urm = new_urm;
  try {
    if (block.hasProperty("debug")) {
      debug = block("debug").as<bool>();
    }
    if (block.hasProperty("t_hysteresis")) {
      t_hysteresis = block("t_hysteresis").as<double>();
    }
    
  } catch (exception &e) {
    cerr << "SoaringPlanner::init --> Error while loading data. Content:" << e.what() << endl;
  }
}

ParseBlock* SoaringPlanner::toBlock() const
{
  ParseBlock *block = new ParseBlock;
  
  block->setProperty("debug", functions::boolToString(debug));
  block->setProperty("t_hysteresis", functions::numberToString(t_hysteresis));
  
  return block;
}
}