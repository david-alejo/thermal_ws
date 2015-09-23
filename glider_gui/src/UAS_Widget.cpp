#include "UAS_Widget.h"
#include "glider_planner/UAVROS.h"

using glider_planner::UAV;
using glider_planner::UAVROS;
UAS_Widget::UAS_Widget(QWidget *parent, UAV *uav) : QWidget (parent) {
    view = new Ui::UAS;
    view->setupUi(this);
    connect(view->spinBox, SIGNAL(valueChanged (int)),
	    this,SLOT(setThrottle(int)));
    _uav = uav;
}

void UAS_Widget::updateUAVState(glider_planner::UAVState st)
{
  view->progressBar -> setValue(st.percent);
  view->lcdNumber_2 -> display(st.altitude);
  view->lcdNumber_4 -> display(st.airspeed);
//   view->spinBox -> setValue(st.throttle);
  view->lcdNumber -> display(st.voltage);
  view->lcdNumber_3 -> display(st.ascrate);
  
}


  
void UAS_Widget::setThrottle(int i)
{
  glider_planner::UAVROS *uav_ros = dynamic_cast<UAVROS *>(_uav);
  if (uav_ros != NULL) {
    uav_ros->setThrMaxRetry(i);
  }
}
