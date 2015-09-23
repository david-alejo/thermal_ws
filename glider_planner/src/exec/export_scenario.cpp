#include <iostream>
#include <functions/ArgumentData.h>
#include "src/CompleteSystem.h"

using namespace functions;
using namespace std;

using namespace glider_planner;

// This example will export the data contained in the scenario into a kml file 

int main(int argc, char **argv) {
    ArgumentData arg(argc,argv);
    if (argc < 3) {
        cerr << "Use: " << arg.at(0) << "<input_filename> <output_filename>" << endl;
        return 1;
    } else {
      CompleteSystem s(arg.at(1));
      if (s.getURM()->exportKMLFile(arg.at(2))) {
	cout << "File exported successfully.\n";
      }
      
      
    }
    
    return 0;
}
