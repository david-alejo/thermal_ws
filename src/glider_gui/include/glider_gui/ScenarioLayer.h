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


#ifndef SCENARIOLAYER_H
#define SCENARIOLAYER_H

#include <marble/MarbleWidget.h>
#include <marble/LayerInterface.h>
#include <marble/GeoPainter.h>

#include "CompleteSystemGui.h"



class CompleteSystemGui;

class ScenarioLayer:public Marble::LayerInterface
{
public:
  // Constructor
    ScenarioLayer(Marble::MarbleWidget* widget, CompleteSystemGui *ref, const QString &uav_image_file, 
		  bool paint_up = true, bool paint_way = true);
    
    // Destructor
    virtual ~ScenarioLayer();
    
    // Implemented from LayerInterface
    virtual QStringList renderPosition() const;
 
    // Implemented from LayerInterface
    virtual bool render( Marble::GeoPainter *painter, Marble::ViewportParams *viewport,
       const QString& renderPos = "NONE", Marble::GeoSceneLayer * layer = 0 );
    
    // Overriding QObject
//     virtual bool eventFilter(QObject *obj, QEvent *event);
    
    inline void setWaypointPen(QPen *pen) {
      delete way_pen;
      way_pen = pen;
    }
    
    inline void setUpdraftPen(QPen *pen) {
      delete up_pen;
      up_pen = pen;
    }
    
    inline static Marble::GeoDataCoordinates toMarble(const UAVFlightPlan::EarthLocation &loc) {
      return Marble::GeoDataCoordinates(loc.getLongitude(), loc.getLatitude(), 0.0, Marble::GeoDataCoordinates::Degree);
    }
       
protected:
  Marble::MarbleWidget *m_widget;
  bool paint_waypoints, paint_updrafts;
  
  QPen *way_pen, *up_pen;
  
  Marble::GeoDataCoordinates home;
  
  CompleteSystemGui *gui_ref;
  
  void paintWaypoints( Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString& renderPos, Marble::GeoSceneLayer *layer);
  
  void paintUpdrafts(Marble::GeoPainter* painter);
};

#endif // SCENARIOLAYER_H
