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


#include "AxisDialog.h"
#include <functions/functions.h>

AxisDialog::AxisDialog(QWidget *parent):QDialog(parent)
{
  setupUi(this);
}

bool AxisDialog::getNewAxisLimits(QWidget * parent, double& min_x, double& min_y, double& max_x, double& max_y)
{
  AxisDialog *diag = new AxisDialog(parent);
  diag->show();

  // Show the current limits
  diag->x_min_edit->setText(functions::numberToString(min_x).c_str());
  diag->x_max_edit->setText(functions::numberToString(max_x).c_str());
  diag->y_min_edit->setText(functions::numberToString(min_y).c_str());
  diag->y_max_edit->setText(functions::numberToString(max_y).c_str());
  bool ret_val = false;
  
  if(diag->exec() == QDialog::Accepted) {
    ret_val = true;
    bool aux;
    min_x = diag->x_min_edit->text().toDouble(&aux);
    ret_val &= aux;
    max_x = diag->x_max_edit->text().toDouble(&aux);
    ret_val &= aux;
    min_y = diag->y_min_edit->text().toDouble(&aux);
    ret_val &= aux;
    max_y = diag->y_max_edit->text().toDouble(&aux);
    ret_val &= aux;
  }
  delete diag;
  
  return ret_val;
}

