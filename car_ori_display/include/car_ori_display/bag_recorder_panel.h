/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-10-27 16:25:26
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-11-05 11:48:02
 * @FilePath: /undefined/home/cyun/10.27/src/car_ori_display/include/car_ori_display/bag_recorder_panel.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef BAG_RECORDER_PANEL_H
#define BAG_RECORDER_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QGridLayout>
#include <QListWidget>
#include <QFileDialog>
#include <QProcess>
#include <QStringList>

namespace bag_recorder
{

class BagRecorderPanel : public rviz::Panel
{
    Q_OBJECT
public:
    BagRecorderPanel(QWidget* parent = 0);

protected Q_SLOTS:
    void refreshTopics();
    // 记录选中话题的函数声明
    void startRecording();
    void stopRecording();
    void filterTopics(const QString& filter_text);
    // 新增：记录所有话题的函数声明
    void recordAllTopics();

private:
    void updateTopicsList(const QStringList& topics);

    QPushButton* refresh_button_;
    QPushButton* record_button_;
    QPushButton* stop_button_;
    QLineEdit* search_input_;  // 用于搜索话题的输入框
    QListWidget* topics_list_;
    QStringList all_topics_;   // 保存所有话题列表以用于搜索
    QStringList selected_topics_;
    QProcess* record_process_;

    QPushButton* record_all_button_;

    ros::NodeHandle nh_;
};

} // end namespace bag_recorder

#endif // BAG_RECORDER_PANEL_H