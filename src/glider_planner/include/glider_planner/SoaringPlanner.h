#ifndef SOARINGPLANNER_H
#define SOARINGPLANNER_H

#include <simulator/Updraft.h>
#include "UAV.h"
#include "URM.h"
#include "TimeSlot.h"
#include <sparser/all.h>
#include <simulator/FlightPlan.h>
#include <list>
#include <vector>
#include <string>

namespace glider_planner {

class SoaringPlanner
{
public:
  //! @brief Call to execute, in this case the initial location is already stored in the class
  //! @note URM has to be initialized first!!
  //! @param output_plan Output plan
  //! @param updrafts The updrafts in the system
  //! @param slots Constrains in the use of the updrafts
  virtual bool execute(simulator::FlightPlan &output_plan) = 0;

  //! Pure Virtual Contructor. Controllers are complex enough to be always created from a Parse Block
  virtual SoaringPlanner *createFromBlock(ParseBlock &block) const = 0;

  //! @brief Returns a string that describes the class contents
  virtual std::string toString() const = 0;
  
  virtual ParseBlock *toBlock() const;

  void setMinDist(double min_dist) {this->min_dist = min_dist;}

  void setUAV(UAV *uav) { this->uav = uav; }

  void setURM(URM *urm) {
    this->urm = urm;
  }

  //! @brief This function has to be called in createFromBlock method
  virtual void init(ParseBlock &block, URM *new_urm = NULL);
  


protected:
  bool debug;
  bool data_loaded; // Don't forget to turn this flag on in the end of createFromBlock implementation
  double min_dist; // Minimum vertical separation between two aircraft while flying in an updraft
  double t_hysteresis; // Minimum time improvement that has to exist in order to get a waypoint already assigned
  UAV *uav; // Parameters and current location of the UAV
  URM *urm; // Reference to urm, is not freed in the destructor
};

}

#endif // SOARINGPLANNER_H
