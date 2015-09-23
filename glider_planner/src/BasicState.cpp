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


#include "glider_planner/BasicState.h"

#include <sstream>

using namespace functions;

namespace glider_planner {

BasicState::BasicState(FormattedTime t, double alt)
{
  time = t;
  altitude = alt;
}

BasicState::BasicState() {
  time.getTime();
  altitude = 0.0;
}

void BasicState::init(const BasicState& s)
{
  altitude = s.altitude;
  time = s.time;
}

std::string BasicState::toString() const
{
  std::ostringstream os;
  os << "Altitude = " << altitude << "\t Time = " << time.getFormattedTime();
  
  return os.str();
}

}