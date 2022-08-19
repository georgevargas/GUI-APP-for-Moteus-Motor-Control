#ifndef OPENCVWORKER_H
#define OPENCVWORKER_H
#include "MoteusAPI.h"
#include "mainwindow.h"
#include <QObject>

#include <sstream>
#include <iomanip>
#include <stdlib.h> // required for system call

#include <stdio.h>

using namespace std;

class OpenCvWorker : public QObject
{
    Q_OBJECT


public:
    explicit OpenCvWorker(QObject *parent = nullptr);
    ~OpenCvWorker();

signals:
    void sendMsg(QString);
    void sendToMain(QString);

public slots:
    void getFromMain(QString msg, QString dev_name,int Motor_id,double start_position,double stop_position,double velocity,double max_torque,double feedforward_torque,double kp_scale,
                     double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay);
    void receiveSetup(int device,int mode );
    void run_cycles();

private:
    void Check_Motor_Error(QString dev_name,int Motor_id);
    double l_start_position;
    double l_stop_position;
    double l_velocity;
    double l_max_torque;
    double l_feedforward_torque;
    double l_kp_scale;
    double l_kd_scale;
    double l_bounds_min;
    double l_bounds_max;
    double l_Cycle_Start_Stop;
    double l_Cycle_Delay;
    QString l_dev_name;
    double delay = 0;
    double l_Cycle;
    int    l_Motor_id;
    bool   Rec_run_Enable = false;
    vector <double> list_Position;
    vector <int>    list_Motor_id;
    vector <double> list_Delay;
    vector <double> list_Velocity;
    vector <double> list_Max_torque;
    vector <double> list_Feedforward_torque;
    vector <double> list_Kp_scale;
    vector <double> list_Kd_scale;


    int current_list_index =0;

};

#endif // OPENCVWORKER_H






