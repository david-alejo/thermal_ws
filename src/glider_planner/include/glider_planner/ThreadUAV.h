#ifndef THREADUAV_H
#define THREADUAV_H

#include "SoaringPlanner.h"
#include "URM.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <glider_planner/ThermalDetector.h>

namespace glider_planner {

class ThreadUAV
{
public:
  //! @brief Recommended constructor
  //! @param u --> UAV parameters
  //! @param p --> Pointer to the planner.
  //! @note It will free the content of p it in the destructor
  //! @param
  ThreadUAV(UAV* u, SoaringPlanner* p, double sleep_time, URM* urm, ThermalDetector *t = NULL);

  inline simulator::FlightPlan getActivePlan() {
    boost::shared_lock<boost::shared_mutex> lock(uav_mutex);
    lock.lock();
    simulator::FlightPlan copy(active_plan);
    return copy;
  }

  ~ThreadUAV();

  int main_loop();

  inline UAV *getUAV() {
    boost::shared_lock<boost::shared_mutex> lock(uav_mutex);
//     lock.lock();
    return uav;
  }

  void launch_thread();

  inline void stop_thread() {_stop = true;}
  
  inline bool isStopped() {return _stop;}

  //! @brief Stops the thread and waits until it has already stopped.
  void join();
  
  inline SoaringPlanner *getPlanner() const {return planner;}
  
  inline void execute() {_execute = true;}
  
  inline ThermalDetector *getDetector() const {return thermal_detector;}
    void setStartingTime(functions::FormattedTime start_time);
  
private:
  bool _stop;
  double sleep_time; // in secs
  UAV *uav; // NOTE: Will not be freed in the destructor
  // Note: the urm pointer will not be freed. It has to be the same instance of URM for all UAVs.
  // Complete system class will free it.
  URM *urm;
  SoaringPlanner *planner;
  boost::shared_mutex uav_mutex;
  simulator::FlightPlan active_plan;
  ThermalDetector *thermal_detector;
  bool _execute;
  
  boost::thread *_thread;
};

}

#endif // THREADUAV_H
