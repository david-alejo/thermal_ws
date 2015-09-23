#ifndef _UAV_FACTORY_H_
#define _UAV_FACTORY_H_
#include "UAVs.h"
#include <sparser/all.h>

namespace glider_planner {

/*!@brief UAV Factory
In order to add another model, add another 'else if' clause to each method
*/
class UAVFactory {
  public:
    
    UAV* create_from_file(const std::string & fileName);
		
    UAV* create_from_block(ParseBlock &block);
  
};

}

#endif //_UAV_FACTORY_H_