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
    double accel_limit = 5.0;
    double position = 1;
    double velocity_limit = 1.5;
    double max_torque = 20;
    double feedforward_torque = 0.1;
    double kp_scale = 1;
    double kd_scale = 1;
    double Cycle_Start_Stop = 100;
    double Cycle_Delay = 1.1;
    bool   Dynamic = false;
    bool   Enable_startup_nearest_commands = true;

    string dev_name = "/dev/ttyACM0";
    int moteus_id = 1;
    int Number_of_Motors = 3;
    double Motor_rest_position[10] ={0.0,0.0,0.208496,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains rest positions for each motor
    double bounds_min[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains the minimum positions for each motor
    double bounds_max[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains the maximum positions for each motor

    void setup();
    void Init_Motor();

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

signals:
    void sendSetup();
    void sendToWorker(QString msg, QString dev_name,int Motor_id,double accel_limit,double position,double velocity_limit,double max_torque,double feedforward_torque,double kp_scale,
         double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay);

public slots:
    void receiveMsg(QString  msg,int Motor_id,double limit_min,double limit_max);
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

    void on_Slider_Accel_Limit_valueChanged(double value);
    void on_Slider_Position_valueChanged(double value);
    void on_Slider_Velocity_Limit_valueChanged(double value);
    void on_Slider_Max_Torque_valueChanged(double value);
    void on_Slider_Feedforward_valueChanged(double value);
    void on_Slider_KP_Scale_valueChanged(double value);
    void on_Slider_KD_Scale_valueChanged(double value);
    void on_Slider_Cycle_Start_Stop_valueChanged(double value);
    void on_Slider_Cycle_Delay_valueChanged(double value);

    void on_Counter_Accel_Limit_valueChanged(double value);
    void on_Counter_Position_valueChanged(double value);
    void on_Counter_Velocity_Limit_valueChanged(double value);
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
    void on_btnStart_Motor_clicked();
    void on_checkBox_Dymamic_clicked();
    void on_btnSetNearest_clicked();
    void on_btnGo_To_Rest_Position_clicked();
};

#endif // MAINWINDOW_H
