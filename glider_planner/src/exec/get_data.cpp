#include <functions/FormattedTime.h>
#include <functions/functions.h>
#include <functions/ArgumentData.h>
#include <iostream>
#include <sstream>
#include <time.h>

using namespace std;
// using namespace glider_planner;

int main(int argc, char **argv) {
  if (argc != 5) {
    cout << "Usage: " << argv[0] << " <init> <end> <time_file> <count_file>\n";
    return -1;
  }
  
  functions::ArgumentData arg(argc, argv);
  int init, end;
  istringstream is(arg[1]);
  is >> init;
  istringstream is2(arg[2]);
  is2 >> end;
  
  double t, t1, t2;
  int h, m, sec, usecs;
  ostringstream ocurrences_data;
  
  char buf[1000];
  std::vector<double> time_v, ocurrences;
  functions::FormattedTime ft1, ft2;
  
  for (int i = init; i <= end; i++) {
    ostringstream name1, name2;
    name1 << i << "/different" << i << ".out";
    name2 << i << "/poi.tex";
    
    t2 = 0;
    
    ifstream file1(name1.str().c_str());
    if (file1.is_open()) {
      bool end = false;
      
      while (!end && file1.good()) {
	file1.getline(buf, 1000);
	string s(buf);
	int f = s.find("Current time:");
	f = s.find(';', f + 8);
	if (f > 0) {
	  end = true;
	  string subs = s.substr(f + 1, s.size());
	  sscanf(subs.c_str(), "%d:%d:%d . %d", &h, &m, &sec, &usecs);
// 	  cout << "Subs = " << subs << " ";
// 	  cout << "H = " << h << " M = " << m << " Sec = " << sec << " Usec = " << usecs;
	  t1 = 3600 * h + 60 * m + sec + usecs / 1000000.0;
	}
      } 
    } else {
      cout << "File 1 not opened.\n";
    }
    
    ifstream file2(name2.str().c_str());
    int ocs[5];
    
    for (int j = 0; j < 5; j++) {
      ocs[j] = 0;
    }
    
    if (file2.is_open()) {
      // Discard the first seven lines
      for (int l = 0; l < 7 && file2.good(); l++) {
	file2.getline(buf, 1000);
      }
      
      
      int cont = 0;
      while (file2.good()) {
	file2.getline(buf, 1000);
	
	if (cont == 8) {
// 	  cout << "Skipped line: " << buf << endl;
	  cont++;
	  continue; // Skip this line (repeated waypoint) (TODO: comment that for well-constructed plans
	}
	
	string s(buf);
	int c = s.find('&');
	int d  = s.find('&', c + 1);
	if (c > 0 && d > 0) {
	  string subs = s.substr(c + 1, d - c - 1);
	  sscanf(subs.c_str(), "%d:%d:%d.%d", &h, &m, &sec, &usecs);
	  
// 	  cout << "H = " << h << " M = " << m << " Sec = " << sec << " Usec = " << usecs << endl;
	  
	  t = 3600 * h + 60 * m + sec + usecs / 1000000.0;
	  t2 = max(t2,t); // get the greatest time
	  
	
	
	  // Get the UAV that has visited this waypoint
	  c = s.find('&');
	  c = s.find('&', c + 1);
	  d = s.find('&', c + 1);
	  istringstream subs_(s.substr(c + 1, d - c - 1));
	  
	  int uav;
	  subs_ >> uav;
	  if (uav > 0) {  
	    ocs[uav - 1]++;
	  } else {
	    cout << " Beware of test " << i << endl;
	  }
	}
	cont++;
      }
      
      for (int j = 0; j < 5; j++) {
	ocurrences_data << ocs[j] << " ";
      }
      ocurrences_data << endl;
      
      
    }
//     cout << t2 - t1 << "\t";
    time_v.push_back(t2 - t1);
  }
  
  if (!functions::writeStringToFile(arg[4], ocurrences_data.str())) {
    cerr << "Could not save the ocurrences file" << endl;
  }
  if (!functions::writeStringToFile(arg[3], functions::printVector(time_v))) {
    cerr << "Could not save the time file" << endl;
  }
  
  
  return 0;
}

