#include "glider_planner/ThreadUAV.h"
#include "glider_planner/ExpandingTreePlanner.h"
#include "glider_planner/UAVs.h"
#include <simulator/Thermals.h>
using namespace std;

namespace glider_planner {

ThreadUAV::ThreadUAV(glider_planner::UAV* u, glider_planner::SoaringPlanner* p, double sleep_time, URM* urm, ThermalDetector* t)
{
  uav = u;
  planner = p;
  planner->setMinDist(urm->getMinDist());
  planner->setUAV(u);
  this->sleep_time = sleep_time;
  this->urm = urm;
  thermal_detector = t;
  _stop = false;
  _thread = NULL;
  _execute = false;
}


ThreadUAV::~ThreadUAV() {
  delete planner;
}

int ThreadUAV::main_loop() {
  _stop = false;
  long t =  (sleep_time * (long)1000000);

  ExpandingTreePlanner *ex_planner = dynamic_cast<ExpandingTreePlanner *>(planner);
  bool second_exploration; // Indicates if we are in a possible thermal that has already been explored for the first time
  while (!_stop && planner != NULL) {
    second_exploration = false;
    boost::unique_lock<boost::shared_mutex> lock(uav_mutex);
    uav->actualize();
    lock.unlock();
    
    if (!_execute) {
      continue;
    }
    
    cout << "UAV " << uav->id << "\t Location: " << uav->current_location.toString() << endl;
    simulator::FlightPlan aux_plan;
    
    // Actualize the data from the urm
    
    if (ex_planner != NULL) {
      ex_planner->setPossibleUpdrafts(urm->getPossibleUpdrafts());
      ex_planner->setUpdrafts(urm->getUpdrafts());
    }
    
    if ( urm->waypointReached(uav->current_location, uav->id, uav) ) {
      cout << "URM::main_loop --> Waypoint reached.\n";
    }
    
    if ( urm->updateEvent(uav->current_location, uav->id, uav) ) {
      cout << "URM::main_loop --> Event detected.\n";
    }
    
    int poss_up = ex_planner->isInPossibleUpdraft(uav->current_location, uav->active_plan);
    if ( poss_up >= 0 ){
      cout << "In possible updraft --> do not change the plan. Flight plan: " << uav->active_plan.toString() << "\n";
      second_exploration = true;
    } else {
      functions::FormattedTime t0;
      t0.getTime();
      if (planner->execute(aux_plan)) {
	functions::FormattedTime t1;
	t1.getTime();
	cout << "ThreadUAV::main_loop UAV " << uav->id << "  --> plan generated successfully. Expended time: " << t1 - t0 << endl;
//       cout << "\tPlan: " << aux_plan.toString() << endl;
      } else {
	cout << "ThreadUAV::main_loop --> warning: could not generate a valid plan. UAV ID: " << uav->id << "\n";
	uav->emergency = true;
      }
      if (aux_plan.size() > 0) {
	cout << "Actualizing flight plan. New = " << aux_plan.toString() << endl;
	active_plan = aux_plan;
	uav->setFlightPlan(active_plan);
      }
    }
    
    // Make the detection
    if (thermal_detector != NULL) {
      thermal_detector->thermalDetectionStep(uav->current_location, second_exploration);
    }
    
    
    if (uav->current_location.at(2) < uav->min_alt * uav->min_alt_coeff) {
      // Check for minimum latitude
      cout << "UAV x minimum altitude reached. Stoping!\n";
      uav->emergency = true;
      stop_thread();
    }
    
    boost::this_thread::sleep(boost::posix_time::microseconds(t));

  }
  _execute = false;
  
  return 0;
}

void ThreadUAV::launch_thread() {
  boost::unique_lock<boost::shared_mutex> lock(uav_mutex);
  if (_thread == NULL) {
    _thread = new boost::thread(boost::bind( &ThreadUAV::main_loop, this));
  }
}
void ThreadUAV::join() {
  stop_thread();
  if (_thread != NULL) {
    _thread->join();
  }
  boost::unique_lock<boost::shared_mutex> lock(uav_mutex);
  delete _thread;
  _thread = NULL;
}

void ThreadUAV::setStartingTime(functions::FormattedTime start_time)
{
  UAVSimulation *uav_sim = dynamic_cast<UAVSimulation *>(uav);
  if ( uav_sim != NULL) {
    // Set the iniit time of the wind models
    simulator::ModelSimpleGlider *msg = dynamic_cast<simulator::ModelSimpleGlider *>( uav_sim->getParticle()->getModel());
    if (msg != NULL) {
      simulator::ThermalModelTime *tmt = dynamic_cast<simulator::ThermalModelTime *>(msg->getThermalModel());
      if (tmt != NULL) {
	tmt->setInitialTime(start_time);
      }
    }
    uav->init_time = start_time;
  }
}

}