#ifndef COMPLETESYSTEM_H
#define COMPLETESYSTEM_H

#include<string>
#include <vector>
#include<sparser/all.h>

#include "SoaringPlannerFactory.h"
#include "ThreadUAV.h"
#include <UAVFlightPlan/earthlocation.h>

namespace glider_planner {

class CompleteSystem
{
public:
  CompleteSystem(const std::string &filename);

  ~CompleteSystem();

  //! @brief Performs the execution of the algorithm
  virtual bool execute();
  
  
  //! @brief Verifies if the thread of the UAV is executed and starts it if necessary
  bool start_threads();
  
  //! @brief Sets the stop variable to true and sends stop commands to all UAV threads
  void stop();

  // -------------- Accessors -----------------------
  inline bool isLoaded() const { return is_loaded; }

  UAV *getUAV(int i) { return uav_threads.at(i)->getUAV();}

  URM *getURM() const {return urm;}

  const functions::RealVector &getMinimumBounds() const { return min_bounds; }

  const functions::RealVector &getMaximumBounds() const { return max_bounds; }
  
  UAVFlightPlan::EarthLocation getCenter() const { return center; }
  
    ParseBlock* toBlock(int max_uav = -1) const;
    
  //! @brief Makes a new problem by modifying the initial conditions of all UAVs in such a way that there are no UAVs closer than min_dist
  //! @brief considering only 2D poisition. The altitude remains unmodified.
  void randomizeIICC(double min_dist);
  
  int nUAVs() const {return uav_threads.size();}

protected:
  std::vector<ThreadUAV *> uav_threads;
  URM *urm;
  functions::RealVector min_bounds, max_bounds;
  bool debug;
  bool is_loaded;
  double sleep_time;
  UAVFlightPlan::EarthLocation center;
  std::string urm_export, poi_export; // Filenames to export all data about visited waypoints and updrafts.
  functions::FormattedTime start_time; // Time when the simulation starts
  
  // Thread control
  bool _stop;

  //! @brief Basic initializer
  void init();

  //! @brief Initializer from filename
  void init(const std::string &st);

  void dispose();

  static Checker* getChecker();

  //! @brief Actualizes the bounds taking into account new bounds. The new scenario is the union of
  //! @brief the old scenario.
  //! @param min New minimum bounds
  //! @param max New maximum bounds
  void actualizeBounds(const functions::RealVector &min, const functions::RealVector &max_);
};

}

#endif // COMPLETESYSTEM_H
