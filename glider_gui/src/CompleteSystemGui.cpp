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


#include "CompleteSystemGui.h"
#include "glider_gui.h"
#include <iostream>
#include <QLabel>
#include <QTableWidget>
#include <QTest>
#include <functions/functions.h>

#include "ScenarioLayer.h"

using namespace std;
using namespace functions;
using UAVFlightPlan::EarthLocation;
using namespace glider_planner;

CompleteSystemGui::CompleteSystemGui(const string& filename, glider_gui* parent): QObject(), CompleteSystem(filename)
{
  this->parent = parent;
}

CompleteSystemGui::~CompleteSystemGui()
{

}


bool CompleteSystemGui::execute()
{
  bool ret_val = is_loaded;
  _stop = false;
  
  // This only launches the UAV threads, the rest of execution is in update slot!

  start_threads();
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
    uav_threads.at(i)->execute();
  }
  
  return ret_val;
}

void CompleteSystemGui::update()
{
  // Initialization
  _stop = urm->allWaypointsVisited();
  if ( !_stop) {
    printLocationData();
    showUAVLocation();
    printWaypointData();
    sendStatusData();
  } else {
    // We have reached the final of the simulation --> export the proper things and wait for the child threads
    // TODO  move this to Glider GUI
    for (unsigned int i = 0; i < uav_threads.size(); i++) {
      ostringstream os;
      os << "uav" << i << ".trj";
      uav_threads.at(i)->join();
      // Export the trajectories of each vehicle
      uav_threads.at(i)->getUAV()->exportTrajectory(os.str());
    }
  
    parent->enableAll();
  }
}

void CompleteSystemGui::initializeWaypointTable() {
  QTableWidget *tab = parent->tableWidget;
  
  WaypointList ways = urm->getWaypoints();
  
  tab->setRowCount(ways.size());
  
  for (unsigned int i = 0; i < ways.size(); i++) {
    QString text_location = QString::fromStdString(ways.at(i).toString());
    QString text_numway = QString::fromStdString(numberToString(i + 1));
    QTableWidgetItem *it1 = new QTableWidgetItem(text_numway);
    QTableWidgetItem *it2 = new QTableWidgetItem(text_location);
    QTableWidgetItem *it3 = new QTableWidgetItem(QString::fromStdString(boolToString(urm->isWaypointReached(ways.at(i)))));
    QString text_assigned, text_time;
    
    text_assigned = QString::fromUtf8("NOT ASSIGNED");
    text_time = QString::fromUtf8("                                        ", 30);
    
    QTableWidgetItem *it4 = new QTableWidgetItem(text_assigned);
    QTableWidgetItem *it5 = new QTableWidgetItem(text_time);
//     QTableWidgetItem *it4 = new QTableWidgetItem(QString::fromStdString(urm->getETA(ways.at(i)).getFormattedTime()));
    tab->setItem(i, 0, it1);
    tab->setItem(i, 1, it2);
    tab->setItem(i, 2, it3);
    tab->setItem(i, 3, it4);
    tab->setItem(i, 4, it5);
//     tab->setItem(i, 3, it4);
  }
  tab->repaint();
}

void CompleteSystemGui::initializeLocationTable() {
  QTableWidget *tab = parent->tableWidget_2;
  
  tab->setRowCount(uav_threads.size());
  
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
	QString text_location = QString::fromStdString(getUAV(i)->current_location.toString());
    QTableWidgetItem *it1 = new QTableWidgetItem(text_location);
    QTableWidgetItem *it2 = new QTableWidgetItem(QString::fromStdString(getUAV(i)->active_plan.toString()));
    QString text_assigned, text_time;
    tab->setItem(i, 0, it1);
    tab->setItem(i, 1, it2);
  }
  tab->repaint();
}



void CompleteSystemGui::printWaypointData()
{
  WaypointList ways = urm->getWaypoints();
  QTableWidget *tab = parent->tableWidget;
  
  for (unsigned int i = 0; i < ways.size(); i++) {
    QString text_location = QString::fromStdString(ways.at(i).toString());
    QString text_numway = QString::fromStdString(numberToString(i + 1));
    
    setText(text_numway, tab->item(i,0));
    setText(text_location, tab->item(i, 1));
    setText(QString::fromStdString(boolToString(urm->isWaypointReached(ways.at(i)))), tab->item(i, 2));
    int assigned = urm->getAssignedUAV(ways.at(i));
    
    if ( assigned >= 0 ) {
      setText(QString::number(assigned), tab->item(i,3));
      //QString text_time = QString::fromStdString(urm->getETA(ways.at(i)).getFormattedTime(false));
      //setText(text_time, tab->item(i, 4));
    } else {
      string tex("NOT ASSIGNED");
      setText(QString::fromStdString(tex), tab->item(i,3));
	  QString text_time = QString::fromUtf8("                                        ", 30);
	  setText(text_time, tab->item(i, 4));
    }
  }
}

void CompleteSystemGui::printLocationData()
{
	QTableWidget *tab = parent->tableWidget_2;
	
	for (unsigned int i = 0; i < uav_threads.size(); i++) {
		QString text_location = QString::fromStdString(getUAV(i)->current_location.toString());

		setText(text_location, tab->item(i, 0));
		setText(QString::fromStdString(getUAV(i)->active_plan.toString()), tab->item(i,1));
	}
}

void CompleteSystemGui::setText(const QString &text, QTableWidgetItem *it) {
  if (it != NULL) {
    it->setText(text);
  }
}

void CompleteSystemGui::initializeUAVLocations()
{
  vector<EarthLocation> v = getPositions();
  
  for (unsigned int i = 0; i < v.size(); i++) {
//     parent->w->addMarker(Marker::UAVMarker, v.at(i), parent->uav_icon);
  }
}

void CompleteSystemGui::showUAVLocation()
{
  std::vector<EarthLocation> v = getPositions();
  
  for (unsigned int i = 0; i < v.size(); i++) {
    emit(UAVCoordinatesChanged(ScenarioLayer::toMarble(v.at(i)),i));
  }
  parent->updateMap();
}

vector< EarthLocation > CompleteSystemGui::getPositions()
{
  vector<EarthLocation> v;
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
    v.push_back(getPosition(i));
  }
  return v;
}

EarthLocation CompleteSystemGui::getPosition(unsigned int n_uav)
{
  EarthLocation loc;
  if (uav_threads.size() > n_uav) {
    RealVector vec = getUAV(n_uav)->current_location;
    loc = parent->getLocation(vec.at(0), vec.at(1));
  }
  return loc;
}
void CompleteSystemGui::sendStatusData(){
  for (unsigned int i = 0; i < uav_threads.size(); i++) {
    if (getUAV(i)->getType() == "ROS") {
      const UAVROS *uav = dynamic_cast<const UAVROS*>(getUAV(i));
      UAVState st;
      st.percent=uav->getPercent();
      st.voltage=uav->getVoltage();
      st.airspeed=uav->getAirSpeed();
      st.ascrate=uav->getAscRate();
      st.throttle=uav->getThrottle();
      st.altitude=uav->getAltitude();
      
      emit(UAVStateChanged(st,i));
    }
  }
}