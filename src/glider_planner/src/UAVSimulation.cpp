/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "glider_planner/UAVSimulation.h"
#include "functions/functions.h"

using namespace std;
using namespace simulator;

namespace glider_planner {

void UAVSimulation::actualize()
{
  if (p != NULL) {
  
    for (int i = 0;i < n_steps; i++) {  
      p->setStopped(false);
      p->runOneStep();
      
    }
    current_location =  p->getState(); // Actualize the state
    curr_time.getTime();
    current_location.push_back(curr_time - init_time);
    trajectory.push_back(current_location); // Add the position to the trajectory
  }
}

bool UAVSimulation::init(ParseBlock& block)
{
  bool ret_val = true;
  try {
    ParticleFactory fac;
    
    ret_val = UAV::init(block);
    
    if (ret_val) {
      p = fac.createFromBlock(block["particle"]);
      ret_val = p != NULL;
    }
    
    n_steps = 1;
    if (block.hasProperty("n_steps")) {
      n_steps = block("n_steps").as<int>();
    }
    
  } catch (exception &e) {
    cerr << "UAVSimulation::init --> Error loading data\n";
    ret_val = false;
  }
  
  return ret_val;
}

ParseBlock* UAVSimulation::toBlock() const
{
    ParseBlock *block = UAV::toBlock();
    
    block->setBlock("particle", p->toBlock());
    block->setProperty("n_steps", functions::numberToString(n_steps));
    block->setProperty("type", "Simulation");
    
    return block;
}


void UAVSimulation::init()
{
  p = NULL;
  
}

void UAVSimulation::dispose()
{
  delete p;
  p = NULL;
}


// TODO: Take the common part to the base class UAV
UAV* UAVSimulation::clone() const
{
  UAV* uav = new UAVSimulation;
  UAVSimulation& uav_sim = dynamic_cast<UAVSimulation &>(*uav);

  UAV::copy_parameters(*uav);
  
  if (p != NULL) {
    uav_sim.p = p->clone();
  } else {
    uav_sim.p = NULL;
  }
  uav_sim.n_steps = n_steps;
  
  return uav;
}

UAV* UAVSimulation::createFromBlock(ParseBlock& block)
{ 
  try {
    if (!init(block)) {
      return NULL; // Check for errors
    }
  } catch (exception &e) {
    cerr << "UAVSimulation::createFromBlock --> exception catched. Content: " << e.what() << endl;
    return NULL;
  }
  
  return clone();
}

Checker* UAVSimulation::getChecker()
{
  Checker *check = UAV::getChecker();
  
  check->addBlock("particle", new NTimes(1));
  
  return check;
}

void UAVSimulation::setFlightPlan(const FlightPlan& new_plan)
{
  UAV::setFlightPlan(new_plan);
  p->getController()->setFlightPlan(new_plan);
  p->setStopped(false);
}

double UAVSimulation::getAscendingRate(const functions::RealVector& v) const
{
  double wind_speed = 0.0;
  
  simulator::ThermalModel *th = NULL;
  if (p != NULL) {
    simulator::ModelSimpleGlider *glider_mod = dynamic_cast<simulator::ModelSimpleGlider *>(p->getModel());
    if (glider_mod != NULL) {
      th = glider_mod->getThermalModel();
    }  
    if (th != NULL) {
      wind_speed = th->getVerticalWindSpeed(v);
    }
    
  }
  
  
  return getAscendingRate(wind_speed);
}

}