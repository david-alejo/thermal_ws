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


#include "LatexDialog.h"


LatexDialog::LatexDialog(QWidget *parent):QDialog(parent)
{
  setupUi(this);
}

bool LatexDialog::getTableData(QWidget *parent, std::string& label, std::string& caption)
{
  LatexDialog *dialog = new LatexDialog(parent);
  bool ret_val = false;
  
  dialog->show();
  
  if (dialog->exec() == QDialog::Accepted) {
    ret_val = true;
    label = dialog->label_edit->text().toStdString();
    caption = dialog->caption_edit->text().toStdString();
  }
  
  delete dialog;
  return ret_val;
}


