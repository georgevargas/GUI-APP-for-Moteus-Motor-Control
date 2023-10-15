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
  double home_state = NAN;
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
    void sendMsg(QString  msg,int Motor_id,double value1,double value2,double value3,double value4,double value5,double value6,double value7);
    void sendToMain(QString);

public slots:
    void getFromMain_position_commands(QString msg, int Motor_id,double start_position,double position,double velocity_limit,double max_torque,double feedforward_torque,double kp_scale,
                     double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay,double position_X,double position_Y);
    void getFromMain_file_commands(QString msg, QString file_name);
    void getFromMain_motor_commands(QString msg, int Motor_id);
    void getFromMain_diagnostic_write_commands(QString msg, int Motor_id, double Value1, double Value2, double Value3);
    void getFromMain_diagnostic_read_commands(QString msg, int Motor_id);
    void receiveSetup();
    void run_cycles();

private:
    void Record_Position(int Motor_id, double accel_limit, double position, double velocity_limit, double max_torque, double feedforward_torque, double kp_scale,
                                   double kd_scale, double bounds_min, double bounds_max, double Delay);
    void inverse_kin(double theta[], double x, double y);
    void forward_kin(double result[], double theta1, double theta2);
    bool Collision_Check( int Motor_id, double position);
    bool Check_Motor(int Motor_id);
    bool Check_TrajectoryComplete(int Motor_id);
    bool Wait_TrajectoryComplete(int Motor_id);
    bool SendPositionCommand(int Motor_id,
                             double position,
                             double velocity_limit,
                             double accel_limit,
                             double max_torque,
                             double feedforward_torque,
                             double kp_scale,
                             double kd_scale,
                             double velocity, // end velocity
                             double watchdog_timer = NAN) ;

    bool SendStopCommand(int moteus_id);
    void ReadState(int moteus_id, State& curr_state) const;

    double L1 = 4.7; // Length 0f arm 1
    double L2 = 8.0; // Length 0f arm 2
    double min_Y = -8.2;
    double min_Pos_X = 1.5;
    double min_Neg_X = -1.5;
    double inner_radius = L1 - L2; // unreachable inner radius around the origin of X,Y.
    double Motor2_rotation_limit = 0.349943;

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
    double position_Gen_Elbow_Up[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double position_Gen_Elbow_Down[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double last_position_destination[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double last_record_destination[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
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
