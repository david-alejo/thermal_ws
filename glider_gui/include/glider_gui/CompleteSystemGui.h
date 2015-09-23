/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  sinosuke <email>

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


#ifndef COMPLETESYSTEMGUI_H
#define COMPLETESYSTEMGUI_H

#include <glider_planner/CompleteSystem.h>
#include <glider_planner/UAVROS.h>
#include "glider_gui.h"
#include <UAVFlightPlan/earthlocation.h>
#include <QtGui/QTableWidget>
#include <QPaintEngine>
#include <glider_planner/UAVState.h>
#include <marble/GeoDataCoordinates.h>

class glider_gui;

class CompleteSystemGui: public QObject, public glider_planner::CompleteSystem
{  
  Q_OBJECT
public:
    CompleteSystemGui(const std::string &filename, glider_gui* parent = 0);
    virtual ~CompleteSystemGui();
    

public slots:
  void update();
  
public:
  void initializeWaypointTable();
  void initializeUAVLocations();
  void initializeLocationTable();
  
  std::vector<UAVFlightPlan::EarthLocation> getPositions();
  
  UAVFlightPlan::EarthLocation getPosition(unsigned int n_uav);
  
  bool execute();
  
  inline double getSleepTime() const {return sleep_time;}
  
signals:
  void UAVCoordinatesChanged(Marble::GeoDataCoordinates coord, unsigned int n_uav);
  void UAVStateChanged(glider_planner::UAVState st, unsigned int n_uav);
protected:
  
  void printWaypointData();
  void showUAVLocation();
  void printLocationData();
  void sendStatusData();
  void setText(const QString &text, QTableWidgetItem *it);
  
  glider_gui *parent;
};

#endif // COMPLETESYSTEMGUI_H
