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


#include "glider_planner/TimeSlot.h"
#include <functions/functions.h>
#include <sstream>

using namespace std;
using namespace functions;

namespace glider_planner {

TimeSlot::TimeSlot()
{
  min_time.getTime();
  initial_altitude = 0.0;
  ascending_rate = 0.0;
  updraft_id = -1;
  uav_id = 0.0;
}

TimeSlot::TimeSlot(ParseBlock& data)
{
  Checker *check = TimeSlot::getChecker();
  
  try {
    data.checkUsing(check);
    initial_altitude = data("initial_altitude").as<double>();
    ascending_rate = data("ascending_rate").as<double>();
    vector<double> v;
    v = data("min_time").as<vector<double> >();
    if (v.size() >= 2) {
      min_time.setTime((long)v.at(0), (long)v.at(1));
    } else if (v.size() == 1) {
        min_time.setTime(v.at(0), 0);
    }
    updraft_id = data("updraft_id").as<int>();
    uav_id = data("uav_id").as<int>();
    
  } catch (std::exception &e) {
    std::cerr << "AlgorithmConfig (initializer) --> failed to get the data from block. Content: ";
    std::cerr << e.what() << "\n";
  }
  delete check;
}

Checker* TimeSlot::getChecker()
{
  Checker *check = new Checker;
  
  check->addProperty("min_time", new NTimes(1));
  check->addProperty("initial_altitude", new NTimes(1));
  check->addProperty("ascending_rate", new NTimes(1));
  check->addProperty("updraft_id", new NTimes(1));
  check->addProperty("uav_id", new NTimes(1));
  
  return check;
}


bool TimeSlot::check(int updraft_id, functions::FormattedTime time, double initial_altitude,
                     double ascending_rate, double safety_dist, double max_altitude, int uav_check) const
{
  // Returns false if the updraft id is the same and >= 0, the uav ids do not are the same and the safety distance is violated

  return ( !(updraft_id >= 0 && updraft_id == this->updraft_id && uav_id != uav_check && 
             getMinDist(initial_altitude, ascending_rate, max_altitude, time) < safety_dist));
}

double TimeSlot::getMinDist(double initial_altitude, double ascending_rate,
                            double max_altitude, FormattedTime t) const {

  // Calculate the time when the last UAV exits the updraft
  FormattedTime t_exit_0 = min_time + (max_altitude - this->initial_altitude) / this->ascending_rate;
  FormattedTime t_exit_1 = t + (max_altitude - initial_altitude) / ascending_rate;
  
  if ((t_exit_0 > t_exit_1 && min_time < t) || (t_exit_0 < t_exit_1 && min_time > t) ) {
    // If the latter is going to exit before the previous--> min dist equals to 0
    return 0.0;
  }

  // Calculate the distance when the latter enters in the updraft
  double d1 = abs(this->initial_altitude + this->ascending_rate * (t - this->min_time) - initial_altitude);
  // Then calculate the distance when the first exits the updraft
  double d2 = abs(this->initial_altitude + this->ascending_rate * (t_exit_0- this->min_time) -
      (initial_altitude + ascending_rate * (t_exit_0 - t)));



  // Return the minimum
  return min(d1, d2);
}

string TimeSlot::toString() const
{
  ostringstream os;
  
  os << "Min time: " << min_time << "\t Initial altitude: " << initial_altitude;
  os << "\t Ascending rate: " << ascending_rate << "\t Updraft ID: " << updraft_id ;
  os << "\t UAV id: " << uav_id << endl;
  
  return os.str();
}

ParseBlock* TimeSlot::toBlock() const
{
  ParseBlock *ret = new ParseBlock;
  
  ret->setProperty("min_time", numberToString(min_time));
  ret->setProperty("initial_altitude", numberToString(initial_altitude));
  ret->setProperty("ascending_rate", numberToString(ascending_rate));
  ret->setProperty("updraft_id", numberToString(updraft_id));
  
  return ret;
}

bool TimeSlot::operator==(const TimeSlot& other) const
{
  return min_time == other.min_time && ascending_rate == other.ascending_rate
      && initial_altitude == other.initial_altitude && updraft_id == other.updraft_id;
}

bool TimeSlot::operator!=(const TimeSlot& other) const
{
  return !((*this)==other);
}

}