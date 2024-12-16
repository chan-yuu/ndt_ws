#ifndef DIAGNOSIS_PANEL_H
#define DIAGNOSIS_PANEL_H

#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <ros/ros.h>
#include <car_interfaces/FaultDiagnosisInterface.h>

namespace diagnosis_panel
{

class DiagnosisPanel : public rviz::Panel
{
    Q_OBJECT
public:
    DiagnosisPanel(QWidget* parent = 0);

protected:
    virtual void onInitialize();

private Q_SLOTS:
    void updateState(const car_interfaces::FaultDiagnosisInterface::ConstPtr& msg);
    void handleTimeout();

private:
    QLabel* createIndicator(const QString& name, QVBoxLayout* layout);

    // UI Elements
    QLabel* lidar_indicator_;
    QLabel* can_indicator_;
    QLabel* gps_can_indicator_;
    QLabel* gps_system_indicator_;
    QLabel* camera_indicator_;
    QLabel* internet_indicator_;

    // ROS-related
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    QTimer* timer_;
};

} // end namespace diagnosis_panel

#endif // DIAGNOSIS_PANEL_H