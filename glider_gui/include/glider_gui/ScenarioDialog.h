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


#ifndef SCENARIODIALOG_H
#define SCENARIODIALOG_H

#include <QDialog>
#ifndef _WIN32
#include <qwt/qwt_symbol.h>
#else
#include <qwt_symbol.h>
#endif

#ifndef _MSC_VER
#include "ui_scenario_dialog.h"
#else
#include "../vs/ui_scenario_dialog.h"
#endif

class ScenarioDialog:public QDialog, Ui_Scenario_Dialog
{
  Q_OBJECT

public:
    ScenarioDialog(QWidget *parent);
    
    static bool getScenarioDialogResults(QWidget *parent, bool &paint_graph, bool &paint_updraft,  
					 bool &clean_graph, QString &updraft_file, QString &waypoint_file, QString &uav_file);
    
private:
//   static QColor getColorFromBox(const QComboBox &box);
//   static QwtSymbol::Style getStyleFromBox(const QComboBox &box);
    
private slots:
    void getUpdraftFile();
    void getWaypointFile();
    void getUAVFile();
};

#endif // SCENARIODIALOG_H
