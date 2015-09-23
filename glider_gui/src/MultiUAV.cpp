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


#include "MultiUAV.h"
#include <qt4/QtGui/QFileDialog>

MultiUAV::MultiUAV(QWidget* parent): QDialog(parent)
{
  setupUi(this);
  
  connect(add_UAV_button, SIGNAL(pressed()), this, SLOT(addUAV()));
  connect(remove_UAV_button, SIGNAL(pressed()), this, SLOT(removeUAV()));
  connect(buttonBox_2->button(QDialogButtonBox::Cancel), SIGNAL(pressed()), SLOT(close()));
  connect(buttonBox_2->button(QDialogButtonBox::Ok), SIGNAL(pressed()), SLOT(accept()));
  
  
}

MultiUAV::~MultiUAV()
{

}

MultiUAV::MultiUAV()
{

}


bool MultiUAV::getFilenames(QWidget* parent, std::list<QString> &files)
{
  // Erase planner content
  bool ret_val = false;
  
  MultiUAV *diag = new MultiUAV(parent);
  
  if (files.size() > 0) {
    diag->listWidget->addItem(*(files.begin()));
    diag->listWidget->item(1)->setFlags(Qt::ItemIsEditable);
  }
  diag->show();
  
  if (diag->exec() == QDialog::Accepted) {
    files.clear();
    for (unsigned int i = 1; i < diag->listWidget->count(); i++) {
      files.push_back(diag->listWidget->item(i)->text());
    }
    
    if (files.size() > 0) {
      ret_val = true;
    }
  }
  
  return ret_val;
}

void MultiUAV::addUAV()
{
  QString s = QFileDialog::getOpenFileName(this, tr("Configuration file"));
  if (!s.isEmpty()) {
    listWidget->addItem(s);
  }
}

void MultiUAV::removeUAV()
{
  listWidget->removeItemWidget(listWidget->takeItem(listWidget->currentRow()));
}


