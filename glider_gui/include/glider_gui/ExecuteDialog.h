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


#ifndef EXECUTEDIALOG_H
#define EXECUTEDIALOG_H

#include <QDialog>
#include "ui_execute_dialog.h"
#include <glider_planner/CompleteSystem.h>

class ExecuteDialog:public QDialog, Ui_ExecuteDialog
{

public:
    ExecuteDialog(QWidget *parent);
    virtual ~ExecuteDialog();
    
    //! @BRIEF Recommended procedure to call to this Dialog. It creates the dialog and executes the plan.
    static bool execute(glider_planner::CompleteSystem *c_sys, simulator::FlightPlan &plan, QWidget *parent = NULL);
};

#endif // EXECUTEDIALOG_H
