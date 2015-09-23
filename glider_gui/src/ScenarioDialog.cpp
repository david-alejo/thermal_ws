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


#include "ScenarioDialog.h"
#include <QFileDialog>

#include <iostream>

ScenarioDialog::ScenarioDialog(QWidget *parent):QDialog(parent)
{
  setupUi(this);
  connect(fold_button_2, SIGNAL(pressed()), SLOT(getWaypointFile()));
  connect(fold_button_3, SIGNAL(pressed()), SLOT(getUpdraftFile()));
  connect(fold_button_4, SIGNAL(pressed()), SLOT(getUAVFile()));
  // Default values:
  filename_edit_3->setText("D:\\Ardupilot\\Experimentos\\urm_bin\\Test_data\\updraftmarker.jpg");
  filename_edit_4->setText("D:\\Ardupilot\\Experimentos\\urm_bin\\Test_data\\uav.jpg");
  filename_edit_2->setText("D:\\Ardupilot\\Experimentos\\urm_bin\\Test_data\\waypoint.jpg");
}

bool ScenarioDialog::getScenarioDialogResults(QWidget *parent, bool &paint_ways, bool &paint_updraft,  
					 bool &clean_plot, QString &updraft_file, QString &waypoint_file, QString &uav_file)
{
  ScenarioDialog *diag = new ScenarioDialog(parent);
  diag->show();
  bool ret_val = false;
  
  if (diag->exec() == QDialog::Accepted) {
    ret_val = true;
    
    paint_ways = diag->paint_waypoints_box->isChecked();
//     if (paint_ways) {
//       std::cout << "Waypoint paint checked\n";
//     }     
    paint_updraft = diag->paint_updraft_box->isChecked();

    
    clean_plot = diag->clean_plot_box->isChecked();
    waypoint_file = diag->filename_edit_2->text();
    updraft_file = diag->filename_edit_3->text();
    uav_file = diag->filename_edit_4->text();
//     std::cout << "File content: " << file.toStdString() << "." << std::endl;
    
  }
  
  return ret_val;
}

/*
QColor ScenarioDialog::getColorFromBox(const QComboBox &box) 
{
  QColor ret;
  
  if (box.currentText() == box.itemText(0)) {
    // Red
    ret = Qt::red;
  } else if (box.currentText() == box.itemText(1)) {
    // Blue
    ret = Qt::blue;
  } else if (box.currentText() == box.itemText(2)) {
    // Green
    ret = Qt::green;
  } else if (box.currentText() == box.itemText(3)) {
    // Black
    ret = Qt::black;
  } else if (box.currentText() == box.itemText(4)) {
    // Gray
    ret = Qt::gray;
  } else {
    ret = Qt::yellow;
  }
  
  return ret;
}

QwtSymbol::Style ScenarioDialog::getStyleFromBox(const QComboBox &box) 
{
  QwtSymbol::Style ret;
  
  if (box.currentText() == box.itemText(0)) {
    // Diamond
    ret = QwtSymbol::Diamond;
  } else if (box.currentText() == box.itemText(1)) {
    // Circle
    ret = QwtSymbol::DTriangle;
  } else {
    // Green
    ret = QwtSymbol::Cross;
  }
  
  return ret;
}*/

void ScenarioDialog::getUpdraftFile()
{
  QString s = QFileDialog::getOpenFileName(this, tr("Select Updraft Icon Filename"));
  if (!s.isEmpty()) {
    filename_edit_3->setText(s);
  }
}

void ScenarioDialog::getWaypointFile() {
  QString s = QFileDialog::getOpenFileName(this, tr("Waypoint Icon Filename"));
  if (!s.isEmpty()) {
    filename_edit_2->setText(s);
  }
}

void ScenarioDialog::getUAVFile()
{
  QString s = QFileDialog::getOpenFileName(this, tr("UAV Icon Filename"));
  if (!s.isEmpty()) {
    filename_edit_4->setText(s);
  }
}

