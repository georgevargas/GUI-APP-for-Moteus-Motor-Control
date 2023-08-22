#ifndef MOTORCVWORKER_H
#define MOTORCVWORKER_H
#include "mainwindow.h"
#include <QObject>

#include <sstream>
#include <iomanip>
#include <stdlib.h> // required for system call

#include <stdio.h>

using namespace std;
enum class Fault {
  kNoFault = 0,
  kCalibrationFault = 32,
  kMotorDriverFault = 33,
  kOverVoltageFault = 34,
  kEncoderFault = 35,
  kMotorNotConfiguredFault = 36,
  kPwmCycleOverrunFault = 37,
  kOverTemperatureFault = 38,
  kOutsideLimitFault = 39,
};

struct State {
  double position = NAN;
  double velocity = NAN;
  double torque = NAN;
  double q_curr = NAN;
  double d_curr = NAN;
  double rezero_state = NAN;
  double voltage = NAN;
  double temperature = NAN;
  double fault = NAN;
  double mode = NAN;
  bool TrajectoryComplete = false;
  // flags
  bool mode_flag = false;
  bool position_flag = false;
  bool velocity_flag = false;
  bool torque_flag = false;
  bool q_curr_flag = false;
  bool d_curr_flag = false;
  bool abs_position_flag = false;
  bool motor_temperature_flag = false;
  bool TrajectoryComplete_flag = false;
  bool home_state_flag = false;
  bool voltage_flag = false;
  bool temperature_flag = false;
  bool fault_flag = false;

  State& EN_Position() {
    position_flag = true;
    return *this;
  }
  State& EN_Velocity() {
    velocity_flag = true;
    return *this;
  }
  State& EN_Torque() {
    torque_flag = true;
    return *this;
  }
  State& EN_QCurr() {
    q_curr_flag = true;
    return *this;
  }
  State& EN_DCurr() {
    d_curr_flag = true;
    return *this;
  }
  State& EN_abs_position() {
    abs_position_flag = true;
    return *this;
  }
  State& motor_temperature() {
    motor_temperature_flag = true;
    return *this;
  }
  State& EN_TrajectoryComplete() {
    TrajectoryComplete_flag = true;
    return *this;
  }
  State& EN_home_state() {
    home_state_flag = true;
    return *this;
  }
  State& EN_Voltage() {
    voltage_flag = true;
    return *this;
  }
  State& EN_Temp() {
    temperature_flag = true;
    return *this;
  }
  State& EN_Fault() {
    fault_flag = true;
    return *this;
  }
  State& EN_Mode() {
    mode_flag = true;
    return *this;
  }

  void Reset() {
    position_flag = false;
    velocity_flag = false;
    torque_flag = false;
    q_curr_flag = false;
    d_curr_flag = false;
    abs_position_flag = false;
    motor_temperature_flag = false;
    home_state_flag = false;
    voltage_flag = false;
    temperature_flag = false;
    fault_flag = false;
    mode_flag = false;
    TrajectoryComplete_flag = false;
  }
};

class Motorworker : public QObject
{
    Q_OBJECT


public:
    explicit Motorworker(QObject *parent = nullptr);
    ~Motorworker();

signals:
    void sendMsg(QString  msg,int Motor_id,double value1,double value2,double value3,double value4,double value5);
    void sendToMain(QString);

public slots:
    void getFromMain(QString msg, QString dev_name,int Motor_id,double start_position,double position,double velocity_limit,double max_torque,double feedforward_torque,double kp_scale,
                     double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay);
    void receiveSetup();
    void run_cycles();

private:
    bool Check_Motor(int Motor_id);
    bool Check_TrajectoryComplete(int Motor_id);
    bool Wait_TrajectoryComplete(int Motor_id);
    bool Check_Velocity(int Motor_id);
    bool SendPositionCommand(int Motor_id,
                             double position,
                             double velocity_limit,
                             double accel_limit,
                             double max_torque,
                             double feedforward_torque,
                             double kp_scale,
                             double kd_scale,
                             double velocity, // end velocity
                             double watchdog_timer = NAN) const;

    bool SendStopCommand(int moteus_id);
    void ReadState(int moteus_id, State& curr_state) const;
    double l_accel_limit;
    double l_position;
    double l_velocity_limit;
    double l_max_torque;
    double l_feedforward_torque;
    double l_kp_scale;
    double l_kd_scale;
    double l_Cycle_Start_Stop;
    double l_Cycle_Delay;
    double delay = 0;
    double l_Cycle;
    int    l_Motor_id;
    int    position_Motor_id = 0;
    int    Dynamic_Motor_id = 0;
    int Trajectory_Timeout_1 = 200; // 20 secs
    int Trajectory_Timeout_2 = 100; // 10 secs
    double l_bounds_min[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains the minimum positions for each motor
    double l_bounds_max[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Contains the mamimum positions for each motor
    bool   Position_wait = false;
    bool   Rec_run_Enable = false;
    bool   Dynamic = false;
    bool   Motor_error = false;
    bool   TrajectoryComplete_error = false;
    bool   Step_Mode = false;
    bool   TrajectoryComplete_pause = false;
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
