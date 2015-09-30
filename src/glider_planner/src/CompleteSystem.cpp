#include "glider_planner/CompleteSystem.h"
#include "glider_planner/UAVFactory.h"
#include <sstream>
#include <functions/functions.h>
#include <functions/RandomNumberGenerator.h>

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>

#ifdef WIN32
#include <windows.h>
#define sleep(n) Sleep(1000 * n)
#endif

using namespace std;

namespace glider_planner {

CompleteSystem::CompleteSystem(const std::string &filename)
{
  init(filename);
}

void CompleteSystem::init() {
  uav_threads.clear();
  urm = NULL;
  is_loaded = false;
  min_bounds.clear();
  max_bounds.clear();
  urm_export = "urm.tex";
  poi_export = "poi.tex";
  _stop = false;
}

CompleteSystem::~CompleteSystem() {
  dispose();
}

void CompleteSystem::stop()
{
  _stop = true;
  
  sleep(sleep_time);

  for (unsigned int i = 0; i < uav_threads.size(); i++) {
    uav_threads.at(i)->stop_thread();
    uav_threads.at(i)->join(); 
  }
    
}


void CompleteSystem::init(const std::string &st) {
  Checker *check = getChecker();
  init();
  
  try {
    ParseBlock data;
    data.load(st.c_str());
    
    // Get the parameters of URM manager (updrafts, etc.)
    data.checkUsing(check);
    urm = new URM(data["urm"]);

    sleep_time = 1.0; // Default sleep time
    if (data.hasProperty("uav_sleep")) {
      sleep_time = data("uav_sleep").as<double>();
    }
    
    if (data.hasProperty("minimum_bounds")) {
      min_bounds = data("minimum_bounds").as<vector <double> >();
    }
    
    if (data.hasProperty("maximum_bounds")) {
      max_bounds = data("maximum_bounds").as<vector <double> >();
    }
    
    std::vector<double> cv = data("center").as<vector<double> >();
    UAVFlightPlan::EarthLocation cen(cv.at(0), cv.at(1), cv.at(2));
    center = cen;
    urm->setCenter(center);

    // Get the UAV data
    ParseBlock::Blocks *uav_blocks = data.getBlocks("uav");

    ParseBlock::Blocks::iterator uav_it = uav_blocks->begin();
    int cont = 0;
    bool error = false;
    
    for (;uav_it != uav_blocks->end(); uav_it++, cont++) {
      // Get the UAV parameters
      UAVFactory fac;
      UAV *aux = fac.create_from_block(**uav_it);
      if (aux != NULL) {
	aux->id = cont;
	
	// Get the detector parameters
	ThermalDetector *detector = NULL;
	if ((*uav_it)->hasBlock("detector")) {
	  detector = new ThermalDetector((**uav_it)["detector"], aux->id, urm, aux);
	}

	// Then the planner parameters
      
	SoaringPlanner *aux_planner = NULL;
      
	if ((*uav_it)->hasBlock("planner")) {
	  SoaringPlannerFactory fac_planner;
	  aux_planner = fac_planner.createFromBlock( (**uav_it)["planner"] );
	  aux_planner->setURM(urm);
	}

	if (aux_planner != NULL) {
	  ThreadUAV *aux_thread = new ThreadUAV(aux, aux_planner, sleep_time, urm, detector);

	  uav_threads.push_back(aux_thread);
	} else {
	  cerr << "Could not get the data of the planner. \n";
	  error = true;
	}
      } else {
	cerr << "Could not get the data of the UAV " << cont + 1 << ". \n";
	error = true;
      }
    }
    is_loaded = !error;
  } catch (std::exception &e) {
    cerr << "CompleteSystem::init --> problem while loading data from file. Content: " << e.what() << endl;
    is_loaded = false;
    urm = NULL;
  }
}

Checker* CompleteSystem::getChecker()
{
  Checker *check = new Checker;
  
  check->addBlock("urm", new NTimes(1));
  check->addBlock("uav", new OneOrMore);
  check->addProperty("center", new NTimes(1));
  
  return check;
}

bool CompleteSystem::start_threads() {
  start_time.getTime();
  cout << "Simulation started in time: " << start_time.getFormattedTime(false) << endl;
  
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
    urm->setStartingTime(start_time);
    uav_threads.at(i)->setStartingTime(start_time);
    uav_threads.at(i)->launch_thread();
  }
  _stop = false;
  
  return true;
}

bool CompleteSystem::execute() {
  bool ret_val = is_loaded;

  
  if (ret_val) {
    ret_val = start_threads();
    for (unsigned int i = 0; i < uav_threads.size() && ret_val; i++) {
      uav_threads.at(i)->execute();
    }
  }
  
  bool unassigned_waypoints = true;
  while ( ret_val && unassigned_waypoints) {
    functions::FormattedTime curr_time;
    curr_time.getTime();
    cout << "CompleteSystem: Current time: " << curr_time.getFormattedTimeMS(false);
    cout << "\tSimulation time: " << curr_time - start_time<< endl;
    usleep(sleep_time * 1000000L * 2L);
    unassigned_waypoints = !urm->allWaypointsVisited();
    for (unsigned int i = 0; i < uav_threads.size() && ret_val; i++) {
      unassigned_waypoints &= !uav_threads.at(i)->isStopped();
    }
  }
  
  for (unsigned int i = 0; i < uav_threads.size() && ret_val; i++) {
    ostringstream os;
    os << "uav" << i << ".trj";
    uav_threads.at(i)->join();
    // Export the trajectories of each vehicle
    if (uav_threads.at(i)->getUAV() != NULL) {
      uav_threads.at(i)->getUAV()->exportTrajectory(os.str());
    }
    // And the detection log
    if (uav_threads.at(i)->getDetector() != NULL) {
      uav_threads.at(i)->getDetector()->writeLog();
    }
  }
  
  functions::writeStringToFile(urm->urm_file, urm->eventsToLatex("Urm summary", "urm_table"));
  functions::writeStringToFile(urm->poi_file, urm->waypointsToLatex("PoIA summary", "poi_table"));
  urm->writeUpdrafts();
  urm->writePossibleLog();
  
  return ret_val;
}

void CompleteSystem::actualizeBounds(const functions::RealVector &min_, const functions::RealVector &max_) {
  if (min_bounds.size() < 2) {
    min_bounds = min_;
  } else if(min_.size() >= 2) {
    for (int i = 0; i < 2;i++){
      min_bounds[i] = min(min_[i], min_bounds[i]);
    }
  }
  if (max_bounds.size() < 2) {
    max_bounds = max_;
  } else if(max_.size() >= 2) {
    for (int i = 0; i < 2;i++){
      max_bounds[i] = max(max_[i], max_bounds[i]);
    }
  }
}

void CompleteSystem::dispose() {
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
      delete uav_threads[i];
  }
  uav_threads.clear();
  min_bounds.clear();
  max_bounds.clear();
  delete urm;
  urm = NULL;
}

ParseBlock *CompleteSystem::toBlock(int max_uav) const
{
  ParseBlock *block = new ParseBlock;
  
  block->setProperty("center", center.toMatlab());
  block->setProperty("uav_sleep", functions::numberToString(sleep_time));
  block->setBlock("urm", urm->toBlock());
  
  if (max_uav < 1 || max_uav > (int)uav_threads.size()) {
    max_uav = uav_threads.size();
  }
  
  for (int i = 0; i < max_uav; i++) {
    ParseBlock *uav_block = uav_threads.at(i)->getUAV()->toBlock();
    
    if (uav_threads.at(i)->getDetector() != NULL) {
      uav_block->setBlock("detector", uav_threads.at(i)->getDetector()->toBlock());
    }
    
    uav_block->setBlock("planner", uav_threads.at(i)->getPlanner()->toBlock());
    block->setBlock("uav", uav_block);
  }
  
  return block;
}

void CompleteSystem::randomizeIICC(double min_dist)
{
  functions::RandomNumberGenerator r;
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
    UAV *uav = uav_threads.at(i)->getUAV();
    
    
    if (uav->getType() == "Simulation") {
      // In this case we can randomizeIICC
      bool ok = false;
      while (!ok) {
	ok = true;
	uav->initial_location[0] = r.rnd(min_bounds.at(0), max_bounds.at(0));
	uav->initial_location[1] = r.rnd(min_bounds.at(1), max_bounds.at(1));
	uav->initial_location[2] = r.rnd(min_bounds.at(2), max_bounds.at(2));
	uav->initial_location[3] = r.rnd(-M_PI, M_PI);
	uav->current_location = uav->initial_location;
	
	int n_th = urm->getNearestThermal(uav->initial_location);
	if (n_th >= 0)  {
	  const simulator::Updraft &u = urm->getUpdrafts().at( urm->getNearestThermal(uav->initial_location));
	  functions::RealVector init_loc(u.getLocation().size());
	  
	  for (unsigned int p = 0; p < init_loc.size(); p++) {
	    init_loc[p] = u.getLocation()[p];
	  }
	  
	  ok = init_loc.distance(u.getLocation()) * 1.2 < uav->getRadius();
	}
	for (unsigned k = 0; k < i && ok; k++) {
	  functions::RealVector r1 = uav->initial_location;
	  functions::RealVector r2 = uav_threads.at(k)->getUAV()->initial_location;
	  ok = r1.distance(r2) > min_dist;
	}
      }
      UAVSimulation &s = dynamic_cast<UAVSimulation &>(*uav);
      std::vector<double> i_s = s.getParticle()->getModel()->getInitialState();
      i_s[0] = uav->initial_location[0];
      i_s[1] = uav->initial_location[1];
      i_s[2] = uav->initial_location[2];
      i_s[3] = uav->initial_location[3];
      s.getParticle()->getModel()->setInitialState(i_s);
      
      // New flight plans:
      uav->active_plan.at(0).x = i_s[0];
      uav->active_plan.at(0).y = i_s[1];
      uav->active_plan.at(0).z = i_s[2];
      simulator::FlightPlan fp = s.getParticle()->getController()->getFlightPlan();
      fp.at(0).x = i_s[0];
      fp.at(0).y = i_s[1];
      fp.at(0).z = i_s[2];
      s.getParticle()->getController()->setFlightPlan(fp);
      
    }
  }
}

}