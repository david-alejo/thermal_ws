/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  sinosuke <email>

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


#include "ScenarioLayer.h"

#include <UAVFlightPlan/earthlocation.h>

using namespace Marble;
using simulator::Updraft;
using functions::RealVector;
using UAVFlightPlan::EarthLocation;

ScenarioLayer::ScenarioLayer(Marble::MarbleWidget* widget, CompleteSystemGui *ref, const QString &uav_image_file, 
			     bool paint_up, bool paint_way):
m_widget(widget), paint_waypoints(paint_way), paint_updrafts(paint_up)
{
  gui_ref = ref;
  
  home = GeoDataCoordinates(gui_ref->getCenter().getLongitude(), gui_ref->getCenter().getLatitude(), gui_ref->getCenter().getAltitude());
  m_widget->setCenterLatitude(home.latitude());
  m_widget->setCenterLongitude(home.longitude());
  m_widget->zoomView(2400);
  
  way_pen = new QPen(QBrush(QColor::fromRgb(255,255,255,200)), 3.0, Qt::SolidLine, Qt::RoundCap ) ;
  up_pen = new QPen(QBrush(QColor::fromRgb(0,0,255,200)), 3.0, Qt::SolidLine, Qt::RoundCap ) ;
}

ScenarioLayer::~ScenarioLayer()
{
  delete way_pen;
  delete up_pen;
}



bool ScenarioLayer::render(GeoPainter* painter, ViewportParams* viewport, const QString& renderPos, GeoSceneLayer* layer)
{
  if (gui_ref == NULL || gui_ref->getURM() == NULL) {
    return true;
  }
  if (paint_waypoints) {
    paintWaypoints(painter, viewport, renderPos, layer);
  }
  if(paint_updrafts) {
   paintUpdrafts(painter);
  }
  
  return true;
}

// bool ScenarioLayer::eventFilter(QObject* obj, QEvent* event)
// {
// 
// }

QStringList ScenarioLayer::renderPosition() const
{
  // In first instance we will over all the information of the map 
  return QStringList() << "USER_TOOLS";
}

void ScenarioLayer::paintWaypoints(GeoPainter* painter, ViewportParams* viewport, const QString& renderPos, GeoSceneLayer* layer)
{
  if (gui_ref == NULL || gui_ref->getURM() == NULL) {
    return;
  }
  glider_planner::WaypointList wp_list(gui_ref->getURM()->getWaypoints());
  
  UAVFlightPlan::EarthLocation loc;
  
  loc = gui_ref->getCenter();
  
  // Set the pen for the ellipses
  painter->setPen(*way_pen);
  for (unsigned int i = 0; i < wp_list.size(); i++) {
    loc = gui_ref->getCenter();
    loc.shift(wp_list[i][0], wp_list[i][1]);
    painter->drawEllipse(toMarble(loc), 5, 5);
  }
}

void ScenarioLayer::paintUpdrafts(GeoPainter* painter) {
  std::vector<Updraft> ups = gui_ref->getURM()->getUpdrafts();
  
  // Set the pen for the ellipses
  painter->setPen(*up_pen);
  
  for (unsigned int cont = 0; cont < ups.size(); cont++) {
    UAVFlightPlan::EarthLocation loc(gui_ref->getCenter());
    
    RealVector v = ups.at(cont).getLocation();
    
    loc.shift(v.at(0), v.at(1));
    GeoDataCoordinates coord(toMarble(loc));
    painter->drawEllipse(coord, 5, 5);
  }
}

