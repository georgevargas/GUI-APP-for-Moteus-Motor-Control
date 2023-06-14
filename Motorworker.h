#ifndef MOTORCVWORKER_H
#define MOTORCVWORKER_H
#include "MoteusAPI.h"
#include "mainwindow.h"
#include <QObject>

#include <sstream>
#include <iomanip>
#include <stdlib.h> // required for system call

#include <stdio.h>

using namespace std;

class Motorworker : public QObject
{
    Q_OBJECT


public:
    explicit Motorworker(QObject *parent = nullptr);
    ~Motorworker();

signals:
    void sendMsg(QString  msg,int Motor_id,double bounds_min,double bounds_max);
    void sendToMain(QString);

public slots:
    void getFromMain(QString msg, QString dev_name,int Motor_id,double start_position,double position,double velocity_limit,double max_torque,double feedforward_torque,double kp_scale,
                     double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay);
    void receiveSetup();
    void run_cycles();

private:
    void Check_Motor_Error(QString dev_name,int Motor_id);
    double l_accel_limit;
    double l_position;
    double l_velocity_limit;
    double l_max_torque;
    double l_feedforward_torque;
    double l_kp_scale;
    double l_kd_scale;
    double l_Cycle_Start_Stop;
    double l_Cycle_Delay;
    QString l_dev_name;
    double delay = 0;
    double l_Cycle;
    int    l_Motor_id;
    int    Dynamic_Motor_id = 0;
    int Trajectory_Timeout_1 = 200; // 20 secs
    int Trajectory_Timeout_2 = 100; // 10 secs
    double l_bounds_min[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains the minimum positions for each motor
    double l_bounds_max[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains the mamimum positions for each motor

    bool   Rec_run_Enable = false;
    bool   Dynamic = false;
    vector <double> list_Position;
    vector <int>    list_Motor_id;
    vector <double> list_Delay;
    vector <double> list_velocity_limit;
    vector <double> list_Max_torque;
    vector <double> list_Feedforward_torque;
    vector <double> list_Kp_scale;
    vector <double> list_Kd_scale;
    vector <double> list_accel_limit;


    int current_list_index =0;

};

#endif // MOTORWORKER_H
