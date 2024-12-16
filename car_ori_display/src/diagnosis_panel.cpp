#include "car_ori_display/diagnosis_panel.h"
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPalette>
#include <QStyle>
#include <QTimer>
#include <ros/ros.h>
#include <QGraphicsDropShadowEffect>
#include "car_ori_display/diagnosis_panel.h"

namespace diagnosis_panel
{

DiagnosisPanel::DiagnosisPanel(QWidget* parent)
    : rviz::Panel(parent), timer_(new QTimer(this))
{
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(240, 240, 240));
    setAutoFillBackground(true);
    setPalette(palette);

    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->setSpacing(20);

    QHBoxLayout* row1_layout = new QHBoxLayout;
    QVBoxLayout* lidar_layout = new QVBoxLayout;
    lidar_indicator_ = createIndicator("Lidar", lidar_layout);
    row1_layout->addLayout(lidar_layout);

    QVBoxLayout* can_layout = new QVBoxLayout;
    can_indicator_ = createIndicator("VCU CAN", can_layout);
    row1_layout->addLayout(can_layout);

    main_layout->addLayout(row1_layout);

    QHBoxLayout* row2_layout = new QHBoxLayout;
    QVBoxLayout* gps_can_layout = new QVBoxLayout;
    gps_can_indicator_ = createIndicator("GPS CAN", gps_can_layout);
    row2_layout->addLayout(gps_can_layout);

    QVBoxLayout* gps_system_layout = new QVBoxLayout;
    gps_system_indicator_ = createIndicator("GPS System", gps_system_layout);
    row2_layout->addLayout(gps_system_layout);

    main_layout->addLayout(row2_layout);

    QHBoxLayout* row3_layout = new QHBoxLayout;
    QVBoxLayout* camera_layout = new QVBoxLayout;
    camera_indicator_ = createIndicator("Camera", camera_layout);
    row3_layout->addLayout(camera_layout);

    QVBoxLayout* internet_layout = new QVBoxLayout;
    internet_indicator_ = createIndicator("Internet", internet_layout);
    row3_layout->addLayout(internet_layout);

    main_layout->addLayout(row3_layout);

    setLayout(main_layout);

    sub_ = nh_.subscribe("fault_diagnosis_data", 1, &DiagnosisPanel::updateState, this);

    connect(timer_, &QTimer::timeout, this, &DiagnosisPanel::handleTimeout);
    timer_->start(1000);
}

void DiagnosisPanel::onInitialize()
{
    // 如果需要初始化其他内容，可以在这里添加
}

QLabel* DiagnosisPanel::createIndicator(const QString& name, QVBoxLayout* layout)
{
    QLabel* indicator = new QLabel;
    indicator->setFixedSize(40, 40);
    indicator->setStyleSheet("background-color: green; border-radius: 20px;");
    indicator->setAlignment(Qt::AlignCenter);

    QLabel* label = new QLabel(name);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("font-size: 14px; font-weight: bold;");

    // 添加阴影效果
    QGraphicsDropShadowEffect* shadowEffect = new QGraphicsDropShadowEffect;
    shadowEffect->setBlurRadius(4);
    shadowEffect->setOffset(2, 2);
    shadowEffect->setColor(QColor(0, 0, 0, 50));
    indicator->setGraphicsEffect(shadowEffect);

    layout->setAlignment(Qt::AlignCenter);
    layout->addWidget(indicator, 0, Qt::AlignHCenter);
    layout->addWidget(label, 0, Qt::AlignHCenter);
    return indicator;
}

void DiagnosisPanel::updateState(const car_interfaces::FaultDiagnosisInterface::ConstPtr& msg)
{
    auto setIndicatorState = [](QLabel* indicator, int state) {
        QString style = state? "background-color: green; border-radius: 20px;" : "background-color: red; border-radius: 20px;";
        indicator->setStyleSheet(style);
    };

    setIndicatorState(lidar_indicator_, msg->lidar_state);
    setIndicatorState(can_indicator_, msg->can_state);
    setIndicatorState(gps_can_indicator_, msg->gps_can_state);
    setIndicatorState(gps_system_indicator_, msg->gps_system_state);
    setIndicatorState(camera_indicator_, msg->camera_state);
    setIndicatorState(internet_indicator_, msg->internet_state);

    timer_->start(1500);
}

void DiagnosisPanel::handleTimeout()
{
    auto setIndicatorState = [](QLabel* indicator) {
        indicator->setStyleSheet("background-color: red; border-radius: 20px;");
    };

    setIndicatorState(lidar_indicator_);
    setIndicatorState(can_indicator_);
    setIndicatorState(gps_can_indicator_);
    setIndicatorState(gps_system_indicator_);
    setIndicatorState(camera_indicator_);
    setIndicatorState(internet_indicator_);
}

} // end namespace diagnosis_panel

PLUGINLIB_EXPORT_CLASS(diagnosis_panel::DiagnosisPanel, rviz::Panel)