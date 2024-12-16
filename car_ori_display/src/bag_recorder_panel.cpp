/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-11-05 12:11:11
 * @FilePath: /undefined/home/cyun/10.27/src/car_ori_display/src/bag_recorder_panel.cpp
 * @Description: 用于记录bag数据的rviz panel
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/bag_recorder_panel.h"
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDebug>
#include <QFileDialog>

namespace bag_recorder
{

BagRecorderPanel::BagRecorderPanel(QWidget* parent)
  : rviz::Panel(parent), record_process_(nullptr)
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    // 刷新按钮
    refresh_button_ = new QPushButton("刷新话题");
    refresh_button_->setStyleSheet("QPushButton { background-color: #5e35b1; color: white; padding: 4px; border-radius: 2px; }");
    connect(refresh_button_, &QPushButton::clicked, this, &BagRecorderPanel::refreshTopics);
    main_layout->addWidget(refresh_button_);

    // 搜索框
    search_input_ = new QLineEdit;
    search_input_->setPlaceholderText("搜索话题");
    search_input_->setStyleSheet("QLineEdit { padding: 2px; border: 1px solid #3498db; border-radius: 2px; }");
    connect(search_input_, &QLineEdit::textChanged, this, &BagRecorderPanel::filterTopics);
    main_layout->addWidget(search_input_);

    // 显示话题列表的控件
    topics_list_ = new QListWidget;
    topics_list_->setSelectionMode(QAbstractItemView::MultiSelection);
    topics_list_->setStyleSheet("QListWidget { background-color: #f0f0f0; border: 1px solid #ccc; border-radius: 2px; } QListWidget::item:selected { background-color: #3498db; color: white; }");
    main_layout->addWidget(topics_list_);

    // 记录按钮和停止按钮的布局
    QHBoxLayout* button_layout = new QHBoxLayout;
    record_button_ = new QPushButton("开始记录");
    record_button_->setStyleSheet("QPushButton { background-color: #2ecc71; color: white; padding: 4px; border-radius: 2px; }");
    connect(record_button_, &QPushButton::clicked, this, &BagRecorderPanel::startRecording);
    button_layout->addWidget(record_button_);

    stop_button_ = new QPushButton("停止记录");
    stop_button_->setStyleSheet("QPushButton { background-color: #e74c3c; color: white; padding: 4px; border-radius: 2px; }");
    connect(stop_button_, &QPushButton::clicked, this, &BagRecorderPanel::stopRecording);
    stop_button_->setEnabled(false);  // 初始时禁用
    button_layout->addWidget(stop_button_);

    // 添加记录所有话题的按钮
    record_all_button_ = new QPushButton("记录所有话题");
    record_all_button_->setStyleSheet("QPushButton { background-color: #3498db; color: white; padding: 4px; border-radius: 2px; }");
    connect(record_all_button_, &QPushButton::clicked, this, &BagRecorderPanel::recordAllTopics);
    main_layout->addWidget(record_all_button_);

    main_layout->addLayout(button_layout);
    setLayout(main_layout);
}

void BagRecorderPanel::refreshTopics()
{
    // 执行 rostopic list 命令
    QProcess process;
    process.start("rostopic list");
    process.waitForFinished();
    QString output(process.readAllStandardOutput());

    // 将话题列表转换为 QStringList 并更新UI
    all_topics_ = output.split("\n", QString::SkipEmptyParts);
    updateTopicsList(all_topics_);
}

void BagRecorderPanel::updateTopicsList(const QStringList& topics)
{
    // 保存当前选中的话题
    QStringList previously_selected_topics;
    for (QListWidgetItem* item : topics_list_->selectedItems())
    {
        previously_selected_topics << item->text();
    }

    topics_list_->clear();
    for (const QString& topic : topics)
    {
        QListWidgetItem* item = new QListWidgetItem(topic, topics_list_);
        if (previously_selected_topics.contains(topic))
        {
            item->setSelected(true);
        }
    }
}

void BagRecorderPanel::startRecording()
{
    // 获取选中的话题
    selected_topics_.clear();
    for (QListWidgetItem* item : topics_list_->selectedItems())
    {
        selected_topics_ << item->text();
    }

    if (selected_topics_.isEmpty())
    {
        QMessageBox::warning(this, "No Topics Selected", "Please select at least one topic to record.");
        return;
    }

    // 打开文件保存对话框
    QString file_name = QFileDialog::getSaveFileName(this, "Save ROS Bag", "", "ROS Bag Files (*.bag)");
    if (file_name.isEmpty())
    {
        return;  // 用户取消了保存
    }

    // 构建 rosbag record 命令
    QStringList args;
    args << "record";
    args << selected_topics_;
    args << "-O" << file_name;

    // 启动 rosbag record 命令
    record_process_ = new QProcess(this);
    record_process_->start("rosbag", args);

    if (record_process_->waitForStarted())
    {
        qDebug() << "Started recording topics:" << selected_topics_;
        record_button_->setEnabled(false);  // 禁用记录按钮
        stop_button_->setEnabled(true);  // 启用停止按钮
    }
    else
    {
        QMessageBox::critical(this, "Error", "Failed to start recording.");
        delete record_process_;
        record_process_ = nullptr;
    }
}

void BagRecorderPanel::stopRecording()
{
    if (record_process_)
    {
        record_process_->terminate();
        record_process_->waitForFinished();
        delete record_process_;
        record_process_ = nullptr;

        record_button_->setEnabled(true);  // 启用记录按钮
        stop_button_->setEnabled(false);  // 禁用停止按钮

        QMessageBox::information(this, "Recording Stopped", "Recording has been stopped.");
    }
}

void BagRecorderPanel::filterTopics(const QString& filter_text)
{
    QStringList filtered_topics;
    for (const QString& topic : all_topics_)
    {
        if (topic.contains(filter_text, Qt::CaseInsensitive))
        {
            filtered_topics << topic;
        }
    }
    updateTopicsList(filtered_topics);
}

// 新添加的记录所有话题的函数
void BagRecorderPanel::recordAllTopics()
{
    // 打开文件保存对话框获取保存位置和文件名
    QString file_name = QFileDialog::getSaveFileName(this, "Save ROS Bag", "", "ROS Bag Files (*.bag)");
    if (file_name.isEmpty())
    {
        return;  // 用户取消了保存
    }

    // 构建 rosbag record -a -b 0命令
    QStringList args;
    args << "record";
    args << "-a";
    args << "-b";
    args << "0";
    args << "-O" << file_name;
    // rosbag record -a -b 0 --split --size=5120
    args << "--split";
    args << "--size=100";

    // 启动 rosbag record -a -b 0命令
    record_process_ = new QProcess(this);
    record_process_->start("rosbag", args);

    if (record_process_->waitForStarted())
    {
        qDebug() << "Started recording all topics.";
        record_button_->setEnabled(false);  // 禁用记录按钮
        stop_button_->setEnabled(true);  // 启用停止按钮
    }
    else
    {
        QMessageBox::critical(this, "Error", "Failed to start recording all topics.");
        delete record_process_;
        record_process_ = nullptr;
    }
}

} // end namespace bag_recorder

PLUGINLIB_EXPORT_CLASS(bag_recorder::BagRecorderPanel, rviz::Panel)