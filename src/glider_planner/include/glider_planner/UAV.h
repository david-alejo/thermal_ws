#ifndef UAV_H
#define UAV_H

#include "sparser/all.h"
#include "functions/RealVector.h"
#include <string>
#include <functions/FormattedTime.h>
#include <simulator/FlightPlan.h>
#include <simulator/Particle.h>
#include <UAVFlightPlan/UAVFlightPlan.h>

namespace glider_planner {

//! @brief This will be an abstract class that will give the base class to any UAV (simulated or real)
//!

class UAV
{
public:
  virtual ~UAV();

  inline void setLocation(const functions::RealVector &vec) {
    curr_time.getTime();
    current_location = vec;
  }
  
  virtual ParseBlock *toBlock() const;

  double gamma; // Cruise descent angle
  double descending_ratio; // Tan of the cruise descent angle
  double v_ref; // Cruise airspeed
  double min_alt; // Minimum flight altitude
  double min_alt_thermal; // Minimum flight altitude to fly to a thermal
  double max_alt; // Maximum altitude
  double min_alt_coeff; // If the minimum altitude * coeff is reached --> emergency!!
  int id; // It's initialized by CompleteSystem
  functions::RealVector initial_location; // 3D coordinates and starting time (optional)
  functions::RealVector current_location; // 3D coordinates in current time
  std::vector<functions::RealVector> trajectory;
  functions::FormattedTime curr_time; // Time last modification of the UAV current location
  simulator::FlightPlan active_plan;
  functions::RealVector home_location;
  functions::FormattedTime init_time;
  bool emergency;

  std::string toString() const;

  double getAscendingRate(double wind_speed) const;
  
  double getAltitudeRatio() const {return descending_ratio;}
  
  double getRadius() const { return (current_location.at(2) - min_alt) / descending_ratio;}
  
  //! @BRIEF This function has to be implemented in each class and will actualize the state of the UAV
  virtual void actualize() = 0;
  
  virtual UAV* createFromBlock(ParseBlock &block) = 0;
  
  virtual void setFlightPlan(const simulator::FlightPlan &new_plan);
  
  //! @brief Makes a perfect copy of the UAV (keeping the type)
  virtual UAV* clone() const = 0;
  
  virtual std::string getType() const = 0;
  
  virtual Checker *getChecker();
  
  bool exportTrajectory(const std::string &filename) const;
  
  bool exportCumulativePlan(const std::string &filename) const;
  
  functions::Point3D getArrivalTimeAltitude(const functions::RealVector& pos_2d, const functions::RealVector& destination, const double& h0, functions::FormattedTime& t0) const;
  
  protected:
  virtual bool init(ParseBlock& block);
  
  UAVFlightPlan::UAVFlightPlan result; // Will store the results of an execution
  
  //! @brief Copy the parameters stored in this class into uav
  void copy_parameters(UAV& uav) const;
};

}

#endif // UAV_H
