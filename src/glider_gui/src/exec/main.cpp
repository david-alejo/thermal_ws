#include <QtGui/QApplication>
#include "glider_gui.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ros::init(argc, argv, "glider_gui");
    glider_gui foo;
    foo.show();
    return app.exec();
}
