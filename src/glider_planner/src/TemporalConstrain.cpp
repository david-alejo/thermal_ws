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


#include "glider_planner/TemporalConstrain.h"
#include <functions/functions.h>
#include <sstream>

using namespace std;
using namespace functions;

namespace glider_planner {

TemporalConstrain::TemporalConstrain()
{
  min_time = 0.0;
  max_time = 0.0;
  updraft_id = -1;
  uav_id = 0.0;
}

TemporalConstrain::TemporalConstrain(ParseBlock& data)
{
  Checker *check = TemporalConstrain::getChecker();
  
  try {
    data.checkUsing(check);
    max_time = data("max_time").as<double>();
    min_time = data("min_time").as<double>();
    updraft_id = data("updraft_id").as<int>();
    uav_id = data("uav_id").as<int>();
    
  } catch (std::exception &e) {
    std::cerr << "AlgorithmConfig (initializer) --> failed to get the data from block. Content: ";
    std::cerr << e.what() << "\n";
  }
  delete check;
}

Checker* TemporalConstrain::getChecker()
{
  Checker *check = new Checker;
  
  check->addProperty("min_time", new NTimes(1));
  check->addProperty("max_time", new NTimes(1));
  check->addProperty("updraft_id", new NTimes(1));
  check->addProperty("uav_id", new NTimes(1));
  
  return check;
}


bool TemporalConstrain::check(int updraft_id, double time)
{
  // Returns false if the updraft id is the same and >= 0 and if the time is in the interval
  return ( !(updraft_id >= 0 && updraft_id == this->updraft_id && time < max_time && time > min_time));
}

string TemporalConstrain::toString() const
{
  ostringstream os;
  
  os << "Min time: " << min_time << "\t Max time: " << max_time << "\t Updraft ID: " << updraft_id;
  os << "\t UAV id: " << uav_id << endl;
  
  return os.str();
}

ParseBlock* TemporalConstrain::toBlock() const
{
  ParseBlock *ret = new ParseBlock;
  
  ret->setProperty("min_time", numberToString(min_time));
  ret->setProperty("max_time", numberToString(max_time));
  ret->setProperty("updraft_id", numberToString(updraft_id));
  
  return ret;
}

bool TemporalConstrain::operator==(const TemporalConstrain& other) const
{
  return min_time == other.min_time && max_time == other.max_time && updraft_id == other.updraft_id;
}

bool TemporalConstrain::operator!=(const TemporalConstrain& other) const
{
  return !((*this)==other);
}

}