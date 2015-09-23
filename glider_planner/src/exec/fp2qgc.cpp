// This executable loads a flight plan from file and 
// prints its content to stdout.
// Opcionally it can export its content to a kml file

#include "UAVFlightPlan/UAVFlightPlan.h"
#include "functions/ArgumentData.h"
#include "functions/Point3D.h"
#include "simulator/FlightPlan.h"
#include <iostream>
#include <string>
#include <vector>

using namespace functions;
using UAVFlightPlan::EarthLocation;
using namespace std;
// using namespace glider_planner;

int main(int argc, char **argv) {
  ArgumentData arg(argc,argv);
  
  if (argc < 5) {
    cout << "Use: " << arg[0] << " <relative_flight_plan_file> <center_file> <qgc_file> <kml_file>" << endl;
    return -1; 
  }
  
  simulator::FlightPlan fp(arg[1]);
  
  cout << fp.toString() << endl;
  EarthLocation e(arg[2]);
  
  UAVFlightPlan::UAVFlightPlan uav(fp, e);
  string kml_file = arg[4];
  string qgc_file = arg[3];
  cout << " Exporting to KML file: " << kml_file << endl;
   
  arg.getOption("kml", kml_file);
#ifdef USE_KML
  uav.exportToKMLFile(kml_file);
#endif
  uav.toQGCFile(qgc_file);

  
  return 0;
}
