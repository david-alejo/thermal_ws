#include <iostream>
#include <functions/ArgumentData.h>
#include "src/SoaringPlannerFactory.h"
#include "src/UAVFactory.h"
#include "src/URM.h"

//using namespace functions;
using namespace std;
using functions::ArgumentData;
using namespace glider_planner;

// This example will test the functionality of an independent planner

int main(int argc, char **argv) {
    ArgumentData arg(argc,argv);
    if (argc < 2) {
        cerr << "Use: " << arg.at(0) << "<input_filename>" << endl;
        return 1;
    } else {
      if (arg.isOption("simulation")) {
	UAVFactory fac;
	
	UAV *uav = NULL;
	
	try {
	  ParseBlock block; 
	  block.load(arg.at(1).c_str());
	  uav = fac.create_from_block(block["uav"]);
	} catch (exception &e) {
	  cerr << "main:: Exception while loading data\n";
	}
	
	if (uav != NULL) {
	  cout << uav->toString() << endl;
	  for (int i = 0; i < 100; i++) {
	    uav->actualize();
	  }
	  cout << uav->toString() << endl;
	}
	
	
      } else if (arg.isOption("time-slot")) {
	cout << "Performing time slot testing.\n";
	TimeSlotList list;
	
	double min_dist = 5.0;
	double max_altitude = 250.0;
	try {
	  ParseBlock block; 
	  block.load(arg.at(1).c_str());
	  ParseBlock::Blocks *slots = block.getBlocks("timeslot");
	  
	  if (block.hasProperty("min_dist")) {
	    min_dist = block("min_dist").as<double>();
	  }
	  cout << "Min dist = " << min_dist << endl;
	  
	  if (block.hasProperty("max_altitude")) {
	    max_altitude = block("max_altitude").as<double>();
	  }
	  cout << "Maximum altitude = " << max_altitude << endl;
	  
	  ParseBlock::Blocks::iterator sl = slots->begin();
	  for (;sl != slots->end(); sl++) {
	    TimeSlot s(**sl);
	    cout << "Timeslot loaded: " << s.toString() << endl;
	    list.push_back(s);
	  }
	  
	  
	} catch (exception &e) {
	  cerr << "main:: Exception while loading data\n";
	  exit(1);
	}
	
	
	
	
	// Perform the tests. Check the first with all the other
	TimeSlotList::iterator it = list.begin();
	it++;
	TimeSlot &first = *list.begin();
	cout << "Checking this with all the others. " <<first.toString() << endl;
	for (;it != list.end(); it++) {
	  cout << "With: " << it->toString();
	  if (it->check(first.updraft_id, first.min_time, first.initial_altitude, first.ascending_rate, min_dist, max_altitude, first.uav_id)) {
	    cout << "Passes." << endl;
	  } else {
	   cout << "Fails." << endl; 
	  }
	}
      } else {
	
	URM *urm = NULL;
// 	double sleep_time;
	
	cout << "Performing normal test.\n";
	
	try {
	  ParseBlock data;
	  data.load(arg.at(1).c_str());
    
	  // Get the parameters of URM manager (updrafts, etc.)
	  if (!data.hasBlock("urm")) {
	    cerr << "The data faile has not urm data.\n";
	    exit(1);
	  }
	  urm = new URM(data["urm"]);

// 	  sleep_time = 1.0; // Default sleep time
// 	  if (data.hasProperty("uav_sleep")) {
// 	    sleep_time = data("uav_sleep").as<double>();
// 	  }

	  // Get the UAV data
	  ParseBlock::Blocks *uav_blocks = data.getBlocks("uav");

	  ParseBlock::Blocks::iterator uav_it = uav_blocks->begin();
	  int cont = 0;
// 	  bool error = false;
	  simulator::FlightPlan plan;
	  
	  int n_points = 0;
	  if (arg.isOption("random_wp")) {
	    arg.getOption("random_wp", n_points);
	    
	    for (int i = 0; i < n_points; i++) {
	      functions::RealVector v(2, true);
	      v = v * 1000.0;
	      
	      urm->addWaypoint(v);
	    }
	    
	  }
    
	  for (;uav_it != uav_blocks->end(); uav_it++, cont++) {
	    // Get the UAV parameters
	    UAVFactory fac;
	    UAV *aux = fac.create_from_block(**uav_it);
	    
	    if (!aux) {
	      cerr << "Error while loading UAV " << cont << " data.\n";
	      continue;
	    }
	    
	    aux->id = cont;

	    // Then the planner parameters
      
	    SoaringPlanner *aux_planner = NULL;
      
	    if ((*uav_it)->hasBlock("planner")) {
	      SoaringPlannerFactory fac_planner;
	      aux_planner = fac_planner.createFromBlock( (**uav_it)["planner"] );
	      if (aux_planner != NULL) {
		aux_planner->setURM(urm);
		aux_planner->setUAV(aux);
	      }
	      functions::FormattedTime t0;
	      t0.getTime();
	      if (aux_planner != NULL && aux_planner->execute(plan)) {
		functions::FormattedTime t1;
		t1.getTime();
		cout << "Output flight plan: " << plan.toString() << endl;
		cout << "Expended time: " << t1 - t0 << endl;
		
	      } else {
		cout << "The planner failed.\n";
	      }
	    }
      
	  }
	} catch (exception &e) {
	  cerr << "main:: Exception while loading data\n";
	}
      
	
      }
    }
    
    return 0;
}
