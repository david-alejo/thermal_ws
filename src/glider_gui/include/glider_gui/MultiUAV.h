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


#ifndef MULTIUAV_H
#define MULTIUAV_H

#include <QFileDialog>
#include "ui_multi_uav.h"


class MultiUAV:public QDialog, Ui::MultiUAVDialog
{
  Q_OBJECT
  
  public:
    MultiUAV();
    MultiUAV(QWidget *parent);
    virtual ~MultiUAV();
  
  
  public slots:
  void addUAV();
  void removeUAV();
  
  
public: 
    //! @BRIEF Recommended procedure to call to this Dialog. It creates the dialog and executes the plan.
    static bool getFilenames(QWidget* parent, std::list<QString> &files);
  

};

#endif // MULTIUAV_H
