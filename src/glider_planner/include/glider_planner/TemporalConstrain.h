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


#ifndef TEMPORALCONSTRAIN_H
#define TEMPORALCONSTRAIN_H

#include <sparser/all.h>
#include <string>

namespace glider_planner {

class TemporalConstrain
{

public:
  // Default constructor
    TemporalConstrain();
    
    // Get data from parseblock
    TemporalConstrain(ParseBlock &data);
    
    //! @brief Main method. Checks if the constrain is accomplished
    bool check(int updraft_id, double time);
    
    //! @brief Represents the data in the class
    //! @return a std::string with the data
    std::string toString() const;
    
    //! @brief Represents the data in a sparser like format
    //! @return A sparser Block of data
    ParseBlock *toBlock() const;
    
    int updraft_id;
    double min_time;
    double max_time;
    int uav_id; // Identificator of the UAV that made the constrain
    
    bool operator==(const TemporalConstrain &other) const;
    bool operator!=(const TemporalConstrain &other) const;
    
private:
  static Checker *getChecker();
};

}

#endif // TEMPORALCONSTRAIN_H
