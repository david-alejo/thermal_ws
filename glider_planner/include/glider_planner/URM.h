/*
    This class implements the methods that will use the Updraft Resource Manager
    Copyright (C) 2012  D. Alejo, J.A. Cobano

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

#ifndef URM_H
#define URM_H

#include "simulator/Updraft.h"
#include "TimeSlot.h"
#include "simulator/FlightPlan.h"
#include "UAV.h"
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#include <string>
#include <vector>
#include <list>
#include <sparser/all.h>
#include <UAVFlightPlan/earthlocation.h>

namespace glider_planner {

typedef std::list<TimeSlot> TimeSlotList;
typedef std::vector<functions::RealVector> WaypointList;

struct PossibleUpdraft {
  functions::RealVector p1;
  functions::RealVector p2;
  functions::RealVector center;
  functions::FormattedTime detection_time;
  
  static const double alive_time = 600.0; // How many secs are this possible updrafts alive?
  
  std::string toString() const;
  
  bool isAlive() const {
    functions::FormattedTime t;
    t.getTime();
    return t - detection_time < alive_time;
  }
};

struct URMEvent {
  int thermal;
  int uav;
  bool exit; // True exit, false enter
  functions::FormattedTime time;
  double altitude;
};

class URM
{
  public:
    // Constants
    static const int OBJECTIVE_NOT_IN_THE_LIST = -2;
    static const int OBJECTIVE_NOT_ASSIGNED = -1;
//     static const int OBJECTIVE_REACHED = -3;

    //! @brief constructor from file
    URM(ParseBlock &block);
    
    //! @brief constructor from updraft vector
    URM(const std::vector<simulator::Updraft> &updraft_vec);

    //! Destructor
    ~URM();
    
    //! @brief Translates the info into a string
    std::string toString() const;
    
    //! @brief Stores all the relevant info of the class into a sparser like format
    ParseBlock *toBlock() const;

    //! @brief Return a string with into Latex format
    //! @return The formatted string
    std::string toLatex(const std::string &caption, const std::string &label, int precision = -1, double scale = 1.0) const;
    
    std::string waypointsToLatex(const std::string &caption, const std::string &label, int precision = -1, double scale = 1.0);
    
    std::string eventsToLatex(const std::string &caption, const std::string &label, int precision = -1, double scale = 1.0) const;
    
    //! @brief Gets a flight plan and extracts its useful info into constrains, adding them to the constrain list
    //! @param fp The input flight plan
    void getConstraints(const simulator::FlightPlan &fp, const UAV *uav);

    //! @brief Gets all the constrains obtained so far
    //! @return The constrain list
    const TimeSlotList &getConstraints();

    const std::vector<simulator::Updraft> &getUpdrafts();
    
    //! @brief Checks that the flight plan does not violate any existing constrain in the system
    //! @param fp The flight plan that will be checked
    //! @param constrain Output: the list of the constrains that have been not rmet
    //! @retval true The flight plan is acceptable
    //! @retval false The flight plan violates some constrains
    bool checkConstraints(const simulator::FlightPlan& fp, TimeSlotList& constrain, const UAV &uav);
    
    //! \brief Adds a constrain to the constrain list
    void addConstraint(const TimeSlot& c);
    
    //! @brief Switches the debug mode to on
    inline void setDebugMode(bool new_val = true) {debug = new_val;}

    //! @return The plans of the vehicles
    inline const std::vector<simulator::FlightPlan> &getPlans() const {return plans;}

    //! @brief Gets the minimum allowed vertical separation between two vehicles in a thermal
    //! @return The minimum separation inside a thermal
    inline double getMinDist() const { return min_dist; }

    functions::FormattedTime getETA(const functions::RealVector &waypoint) {
      boost::shared_lock<boost::shared_mutex> lock(slot_mutex);
      functions::FormattedTime ret;
      ret.getTime();
      if (getAssignedUAV(waypoint) == OBJECTIVE_NOT_ASSIGNED) {
	
      } else if (getAssignedUAV(waypoint) == OBJECTIVE_NOT_IN_THE_LIST) {
	
      } else {
        return ETA.at(waypoint);
      }
      return ret;
    }

    bool assignUAV(const functions::RealVector& waypoint, int uav, double cost, functions::FormattedTime eta);
    
    void unassignUAV(const functions::RealVector &waypoint, int uav);
    
    //! @brief Returns true if all waypoints have been visited successfully
    bool allWaypointsVisited();
    
    //! @brief This functions is called by ThreadUAV whenever an objective waypoint is visited.
    void waypointVisited(const functions::RealVector &vec) {
      boost::unique_lock<boost::shared_mutex> lock(slot_mutex);
      waypoint_reached[vec] = true;
    }
    
    int getAssignedUAV(const functions::RealVector& waypoint);
    
    //! @brief Verifies if one event is being produced (event --> one UAV enters or exists an updraft)
    bool updateEvent(const functions::RealVector& position, int uav, const UAV* uav_);
    
    //! @brief Gets the file checker
    Checker *getChecker() const;
    
    WaypointList getWaypoints();

    //! @brief Checks that the flight plan does not violate any existing constraint in the system
    bool checkConstraints(functions::FormattedTime init_time, double initial_altitude, int updraft_id,
                         TimeSlotList &constrain, const UAV &uav);
			 
    int getUpdraftFromLocation(const functions::RealVector &v) const {
      functions::Point3D p(v);
      return getUpdraftFromLocation(p);
    }
    
    inline void setWaypointArrivalTolerance(double &new_value) { waypoint_arrival_tolerance = new_value; }
    
    inline double getWaypointArrivalTolerance() const { return waypoint_arrival_tolerance; }
    
    //! @brief Checks if an UAV has already passed through a PoI
    //! @brief 
    //! @retval true An unexplored PoI has been visited
    //! @retval false No unexplored PoI has been visited in this time
    bool waypointReached(const functions::RealVector& v, uint uav_id, const UAV* uav);
    
    //! @brief Exports all the waypoint data and updraft data to a kml file
    //! @param filename The name of the file that will be written
    bool exportKMLFile(const std::string &filename);
    
    inline void setCenter(const UAVFlightPlan::EarthLocation new_center) { center = new_center; }
    
    //! @brief Indicates if the point of interest has been already visited.
    bool isWaypointReached(const functions::RealVector &waypoint);
    
    //! @brief This function will be invocked if the planner fails
    simulator::FlightPlan getAlternativePlan(const UAV &uav);
    
    //! @brief Actualizes the exit time when inside an updraft
    functions::Point3D getFinalUpdraftTime(functions::RealVector pos2d, double h0, int n_up, functions::FormattedTime& t, const UAV& uav);
    
     //! @brief Gets the nearest thermal from a position.
    //! @retval -1 No updrafts in vector
    //! @return The ID of the nearest updraft
    int getNearestThermal(const functions::RealVector &pos);
    
    //! @brief Tries to add a new thermal to the systems. First 
    bool addThermal(const simulator::Updraft &new_updraft);
    
    
    bool addPossibleUpdraft(const PossibleUpdraft &p_up);
    
    bool deletePossibleUpdraft(const functions::RealVector &pos_up);
    
    void deleteUpdraft(int n_t) {
      boost::unique_lock<boost::shared_mutex> lock_2(updraft_mutex);
      if (n_t > 0 && n_t < (int)updraft_vector.size()) {
	updraft_vector.at(n_t).setWindSpeed(0.0); // If there is no upwind --> the thermal is considered as dead
      }
    }
    
    int isPossibleUpdraft(const functions::RealVector& new_possible, double min_dist);
    
    std::vector<PossibleUpdraft> getPossibleUpdrafts() const {return possible_updrafts;}
    
    void actualizeThermalParameters(const functions::RealVector &new_center, double new_speed, const functions::RealVector new_drift, int n_up) {
      std::cout << "URM --> Updating parameters: " << new_center.toString() <<  " Wind speed: " << new_speed ;
      std::cout << " New drift: " << new_drift.toString() << " N_up = " << n_up << std::endl;
      
      
      if (n_up >= 0 && n_up < (int)updraft_vector.size()) {
	updraft_vector.at(n_up).updateParameters(new_center, new_speed, new_drift);
      }
    }
    bool writeUpdrafts() const;
    
    bool writePossibleLog() const;
    
    void setStartingTime(functions::FormattedTime start_time);
    
    
    inline bool getPossibleUpdraft(int pos_up, PossibleUpdraft &pos) {
      bool ret_val = false;
      boost::shared_lock<boost::shared_mutex> lock_2(updraft_mutex);
      if (pos_up < (int)possible_updrafts.size() && pos_up >= 0) {
	pos = possible_updrafts.at(pos_up);
	ret_val = true;
      }
      return ret_val;
    }
    double getMaxHeight() const {
        void deleteUpdraft(int n_therm);
      return max_height;
    }
    
    void addWaypoint(const functions::RealVector &new_way);
    
    std::vector<simulator::FlightPlan> getFlightPlans() {return plans;}
    
  protected:
    // Possible updrafts and mutex. TODO: Use it!
    boost::shared_mutex possible_mutex;
    std::vector<PossibleUpdraft> possible_updrafts;
    std::vector<PossibleUpdraft> possible_log;
    std::string possible_log_file;
    std::string waypoint_file;
public:
    
    std::string urm_file;
    std::string poi_file;
protected:
    
    // Updraft vector with its motex
    boost::shared_mutex updraft_mutex;
    std::vector<simulator::Updraft> updraft_vector;
    
    // Constraint list and mutex
    boost::shared_mutex slot_mutex;
    TimeSlotList constraint_list;
    
    std::vector<simulator::FlightPlan> plans;
    WaypointList waypoint_list;
    std::list<URMEvent> events;
    
    double max_height; // Max height of the discovered thermals
    
    // Mutex and info related to the waypoint assignment
    boost::shared_mutex wp_mutex;
    std::map<functions::RealVector, int> assigned_uav; // Relates each waypoint or possible updraft with the UAV that has been assigned to visit it
    std::map<functions::RealVector, double> waypoint_cost; // Relates each waypoint with the minimum cost so far. If reached --> relates it to the altitude
    std::map<functions::RealVector, bool> waypoint_reached; // Relates each waypoint with a bool that indicates whether the waypoint has been reached or not
    std::map<functions::RealVector, functions::FormattedTime> ETA; // Relates each waypoint with its ETA
    std::map<int, int> curr_updraft; // Relates each UAV with an updraft (or -1 if is not in an updraft)
    
    double waypoint_arrival_tolerance;
    UAVFlightPlan::EarthLocation center; // center of the simulation

    double min_dist; // Minimum vertical separation between to vehicles inside an updraft
    
    // Status and behaviour classes
    bool debug;
    bool data_loaded;
    
    std::string updraft_data_output_file;
    
    bool init(ParseBlock &block);
    void init();
    
    //! @brief Returns the ID of the updraft if the point belongs to any updraft
    //! @retval -1 The point p does not belong to any updraft
    //! @return The ID of the updraft or a negative value if the point does not belong to any updraft
    int getUpdraftFromLocation(const functions::Point3D &p) const;
    
    //! @brief Gets the waypoints from file
    bool getWaypointsFromFile(const std::string &filename);
    
    //! @brief Adds one waypoint to the cumulative plan of UAV i
    void appendPlan(const functions::RealVector& v, uint UAVid, bool reach_altitude, const UAV* uav);
};

}

#endif // URM_H
