/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef TIMESLOT_H
#define TIMESLOT_H

#include <sparser/all.h>
#include <string>
#include <functions/FormattedTime.h>

namespace glider_planner {

//! @brief This class will be used to checked if any conflicts are going to be produced when
//! @brief two aircrafts enter in the same updraft.
class TimeSlot
{

public:
  // Default constructor
    TimeSlot();
    
    // Get data from parseblock
    TimeSlot(ParseBlock &data);
    
    //! @brief Main method. Checks if the constrain is accomplished
    //! @param updraft_id The ID of the aircraft
    //! @param time Time when the aircraft has entered
    //! @param initial_altitude Initial altitude of the aircraft in the updraft
    //! @param ascending_rate Ascending rate of the aircraft (m/s)
    //! @param min_dist Minimum vertical separation between two aircrafts in an updraft
    bool check(int updraft_id, functions::FormattedTime time,
               double initial_altitude, double ascending_rate, double min_dist, double max_altitude, int uav_check) const;
    
    //! @brief Represents the data in the class
    //! @return a std::string with the data
    std::string toString() const;
    
    //! @brief Represents the data in a sparser like format
    //! @return A sparser Block of data
    ParseBlock *toBlock() const;
    
    int updraft_id;
    functions::FormattedTime min_time; // Instant when the time enters in the updraft
    double initial_altitude; // Initial altitude of the aircraft when entering in the updraft
    double ascending_rate; // in m/s
    int uav_id; // Identificator of the UAV that made the constrain
    
    bool operator==(const TimeSlot &other) const;
    bool operator!=(const TimeSlot &other) const;
    
private:
  static Checker *getChecker();

  double getMinDist(double initial_altitude, double ascending_rate,
                    double max_altitude, functions::FormattedTime t) const;
};

}

#endif // TimeSlot_H
