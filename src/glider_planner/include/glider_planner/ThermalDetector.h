/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  sinosuke <email>

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


#ifndef THERMALDETECTOR_H
#define THERMALDETECTOR_H

#include <sparser/all.h>
#include <functions/RealVector.h>
#include <functions/FormattedTime.h>
#include "URM.h"
#include "UAV.h"

namespace glider_planner {
  
class ThermalDetector
{
public:
  static const double alive_time = 0.5;
   
  struct DetectionData {
    std::vector<functions::RealVector> states; // Height value since the ascension was detected.
    functions::RealVector center;
    functions::FormattedTime first_time;
    functions::FormattedTime last_time;
    double altitude_gain;
    
    ~DetectionData();
    void calculateCenter();
    std::string toString() const;
  };
  
  ThermalDetector(ParseBlock &b, int uav_id, URM *urm, UAV *parent);
  
  ~ThermalDetector();
  
  void thermalDetectionStep(functions::RealVector& vec, bool second_exploration);
  
  //! @brief Describes the action that is done when a new thermal is detected.
  void thermalAction(bool second_exploration);
  
  //! @brief gets the log data and converts it to a string
  std::string getLog() const;
  
  //! @brief Writes the log to the file specified in log_filename attribute
  bool writeLog() const;
  
  functions::RealVector getPerpendicularLine(const functions::RealVector &p1, const functions::RealVector &p2) const;
  
  ParseBlock *toBlock() const;
  
protected:
  double min_height, max_height;
  double t_consecutive;
  bool ascent_detected;
  double sample_time;
  double usual_radius;
  std::string log_filename;
  functions::RealVector last_state;
  DetectionData *current_detection;
  int uav_id;
  std::vector<DetectionData *> detection_log;
  URM *urm; // Will not be freed!
  UAV *parent;
  int last_therm, last_poss;
  functions::RealVector init_therm;
  functions::FormattedTime init_pos;
  bool descending_in_thermal;
  double dead_thermal_threshold;
  
  void init(ParseBlock& block);
  
  void init();
  
  void dispose();
  
  Checker *getChecker() const;
  
    bool sendPossibleThermal(const functions::RealVector& p1, const functions::RealVector& p2) const;
};

}

#endif // THERMALDETECTOR_H
