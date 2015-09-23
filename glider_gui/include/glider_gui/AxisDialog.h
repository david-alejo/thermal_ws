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


#ifndef AXISDIALOG_H
#define AXISDIALOG_H

#include <QDialog>
#include "ui_axis_dialog.h"

class AxisDialog:public QDialog, Ui_AxisDialog
{

public:
    AxisDialog(QWidget *parent);
    
    static bool getNewAxisLimits(QWidget * parent,double &min_x, double &min_y, double &max_x, double &max_y);
};

#endif // AXISDIALOG_H
