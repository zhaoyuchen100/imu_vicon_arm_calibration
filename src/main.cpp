#include <include/imu_vicon_arm_calibration_gui/mainwindow.h>
#include <QtWidgets/QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "imu_calibration_gui");
    MainWindow w;
    w.show();
    return a.exec();
}
