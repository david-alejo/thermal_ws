#ifndef UAS_WIDGET_H__
#define UAS_WIDGET_H__

#include <QWidget>
#include "ui_uas_widget.h"
#include "glider_planner/UAVState.h"

#include "glider_planner/UAV.h"

class UAS_Widget : public QWidget {
  
  Q_OBJECT
  
public:
  UAS_Widget(QWidget *parent,glider_planner::UAV *uav);
  
public slots:
  void updateUAVState(glider_planner::UAVState st);
  void setThrottle(int i);
protected:
  Ui::UAS *view;
  glider_planner::UAV *_uav;
};

#endif