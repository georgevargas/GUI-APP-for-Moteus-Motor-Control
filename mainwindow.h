#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma once

#include <QMainWindow>
#include <QThread>
#include <regex>
#include <iostream>
#include <string>
#include <algorithm>
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QPixmap>
#include <stdlib.h> // required for system call
#include "MoteusAPI.h"

QT_BEGIN_NAMESPACE
class QPrinter;
QT_END_NAMESPACE

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    Ui::MainWindow *ui;
    QThread *thread;
    QThread *thread1;
    double start_position = -0.35;
    double stop_position = 0.35;
    double velocity = 0.75;
    double max_torque = 1;
    double feedforward_torque = 0.1;
    double kp_scale = 1;
    double kd_scale = 1;
    double bounds_min = -0.35;
    double bounds_max = 0.35;
    double Cycle_Start_Stop = 100;
    double Cycle_Delay = 1.1;

    string dev_name = "/dev/ttyACM0";
    int moteus_id = 1;

    void setup();
    void Init_Motor();

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

signals:
    void sendSetup(int device,int mode);
    void sendToWorker(QString msg, QString dev_name,int Motor_id,double start_position,double stop_position,double velocity,double max_torque,double feedforward_torque,double kp_scale,
         double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay);

public slots:
    void receiveMsg(QString);
    void getFromWorker(QString);

private slots:

    void on_btnStop_Motor_clicked();
    void on_btnRun_Velocity_clicked();
    void on_btnRun_Position_clicked();
    void on_btnStop_Cycle_clicked();
    void on_btnRec_positions_clicked();
    void on_btnRun_Recorded_clicked();
    void on_btnClear_Recorded_clicked();
    void on_btnRead_Status_clicked();
    void on_comboBox_currentIndexChanged(int index);

    void on_Slider_Start_Position_valueChanged(double value);
    void on_Slider_Stop_Position_valueChanged(double value);
    void on_Slider_Velocity_valueChanged(double value);
    void on_Slider_Max_Torque_valueChanged(double value);
    void on_Slider_Feedforward_valueChanged(double value);
    void on_Slider_KP_Scale_valueChanged(double value);
    void on_Slider_KD_Scale_valueChanged(double value);
    void on_Slider_Cycle_Start_Stop_valueChanged(double value);
    void on_Slider_Cycle_Delay_valueChanged(double value);

    void on_Counter_Start_Position_valueChanged(double value);
    void on_Counter_Stop_Position_valueChanged(double value);
    void on_Counter_Velocity_valueChanged(double value);
    void on_Counter_Max_Torque_valueChanged(double value);
    void on_Counter_Feedforward_valueChanged(double value);
    void on_Counter_KP_Scale_valueChanged(double value);
    void on_Counter_KD_Scale_valueChanged(double value);
    void on_Counter_Cycle_Start_Stop_valueChanged(double value);
    void on_Counter_Cycle_Delay_valueChanged(double value);

    void on_actionExit_triggered();
    void on_actionAbout_triggered();
    void on_actionHelp_triggered();
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
};

#endif // MAINWINDOW_H