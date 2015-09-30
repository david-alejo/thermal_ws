#include <iostream>
#include <functions/ArgumentData.h>
#include "functions/functions.h"
#include "glider_planner/CompleteSystem.h"
#include <ros/ros.h>

using namespace functions;
using namespace std;

using namespace glider_planner;

int main(int argc, char **argv) {
  if (argc < 2) {
      cerr << "Usage: " << argv[0] << " <data_filename>\n";
      return -1;
    }
    
    ros::init(argc, argv, "GliderSystem");

  ArgumentData arg(argc, argv);
  CompleteSystem sys(arg.at(1));
  string events_file = "events.tex";
  if (arg.size() > 2) {
    events_file = arg.at(2);
  }

  if (sys.isLoaded() &&  sys.execute()) {
      cout << "Planning proccess done successfully.\n";
      cout << "Exporting the events to " << events_file << endl;
      if (functions::writeStringToFile(events_file, sys.getURM()->eventsToLatex("events", "events"))) {
	cout << "File written OK" << endl;
      } else {
	cerr << "Could not write events file\n";
      }
    } else {
      if (sys.isLoaded()) {
        cout << "Could not execute the planning process.\n";
        } else {
          cout << "Could not load the data\n";
        }
    }

    
  return 0;
}

