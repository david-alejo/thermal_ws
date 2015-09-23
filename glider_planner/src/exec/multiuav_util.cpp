#include <iostream>
#include <functions/ArgumentData.h>
#include "glider_planner/CompleteSystem.h"
#include <functions/functions.h>

using namespace functions;
using namespace std;

using namespace glider_planner;

bool exportBlockFile(const string& filename, ParseBlock& block);

// This utility is used to generate new problems from existing ones by changing the UAV number, the initial conditions and so on and forth

int main(int argc, char **argv) {
  if (argc < 3) {
      cerr << "Usage: " << argv[0] << " <data_filename> <output_filename> [--different_ccii <number_of_tests>]\n";
      return -1;
    }

  ArgumentData arg(argc, argv);
  CompleteSystem sys(arg.at(1));
  
  if (!sys.isLoaded()) {
    cerr << "Could not load the input file.\n";
    return -1;
  }
  
  if (!arg.isOption("different_ccii")) {
    ParseBlock *block = sys.toBlock();
      
    if (!exportBlockFile(arg.at(2), *block)) {
      cerr << "Could not export the file.\n";
      return -1;
    }
    delete block;
      
    CompleteSystem sys2(arg.at(2));
      
    if (sys2.isLoaded()) {
      cout << "Test passed.\n";
    } else {
      cerr << "Could not load the copy file.\n";
      return -1;
    }
  } else {
    int num_tests = 1;
    arg.getOption("different_ccii", num_tests);
    double min_dist = 100.0;
    if (arg.isOption("min_dist")) {
      arg.getOption("min_dist", min_dist);
    }
    bool ok = true;
    int i = 0;
    if (arg.isOption("offset")) {
      arg.getOption("offset", i);
    }
    bool decrease_uavs = arg.isOption("decrease_uavs");
    
    for (; i < num_tests && ok; i++) {
      sys.randomizeIICC(min_dist);
      ParseBlock *block = NULL;
      for (int j = 1; j <= sys.nUAVs() && decrease_uavs; j++) {
	// The same system with increasing number of UAVs is generated
	block = sys.toBlock(j);
	ostringstream os;
	os << arg.at(2) << functions::numberToString(i) << "_" << j;
	ok = exportBlockFile(os.str(), *block);
	delete block;
      }
      if (!decrease_uavs) {
	ParseBlock *block = sys.toBlock();
	ostringstream os;
	os << arg.at(2) << functions::numberToString(i);
	ok = exportBlockFile(os.str(), *block);
	delete block;
      }
      
    }
  }

  return 0;
}

bool exportBlockFile(const string &filename, ParseBlock& block)
{
  bool ret_val = true;
  try {
    ofstream ofs;
    
    ofs.open(filename.c_str());
    
    if (ofs.is_open()) {
      ofs << block;
      ofs.close();
    } else {
      ret_val = false;
      cerr << "exportLoadFile --> could not open the file.\n";
    }
  } catch (exception &e) {
    ret_val = false;
    cerr << "exportLoadFile --> caught an exception while writing the problem\n";
  }
  
  return ret_val;
}
