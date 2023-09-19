#include "Motorworker.h"
#include "ui_mainwindow.h"
#include <QtCore>
#include <errno.h>   // Error number definitions
#include <fcntl.h>   // File control definitions
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <sys/ioctl.h>
#include <termios.h>  // POSIX terminal control definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fstream>
#include <format>
#include "moteus.h"

using namespace mjbots;
using namespace moteus;

Motorworker::Motorworker(QObject *parent) :
    QObject(parent)
{

}


Motorworker::~Motorworker()
{
}
double* Motorworker::inverse_kin(double x, double y)
{
    double * theta = new double[4]{0.0, 0.0, 0.0, 0.0};

// calculate theta2 angle for motor 2 for elbow up and elbow down
    double theta2 =     acos((x*x+y*y-(L1*L1+L2*L2))/(2*L1*L2));
    double theta2_1 = -   acos((x*x+y*y-(L1*L1+L2*L2))/(2*L1*L2));

    // calculate theta1 angle for motor 1 for elbow up and elbow down
    double tanY =  (L2*sin(theta2  ))/(L1+L2*cos(theta2  ));
    double tanY1 = (L2*sin(theta2_1))/(L1+L2*cos(theta2_1));

    theta[1] = theta2;
    theta[3] = theta2_1;

    // calculate q1 angle for motor 1 for both cases
    // four quadrent arctan solution

    if (y >= 0 && x >= 0 )
    {
        theta[0] = atan(-y/x) + atan(tanY);
        theta[2] = atan(-y/x) + atan(tanY1);
    }
    else if (y >= 0 && x < 0 )
    {
        theta[0] = - std::numbers::pi - atan(y/x) + atan(tanY);
        theta[2] = - std::numbers::pi - atan(y/x) + atan(tanY1);
    }
    else if (y < 0 && x >= 0)
    {
        theta[0] = atan(-y/x) + atan(tanY);
        theta[2] = atan(-y/x) + atan(tanY1);
    }
    else if (y < 0 && x < 0)
    {
        theta[0] = - std::numbers::pi - atan(y/x) + atan(tanY);
        theta[2] = - std::numbers::pi - atan(y/x) + atan(tanY1);
    }

    return theta;
}
double* Motorworker::forward_kin(double theta1, double theta2)
{
    double * result = new double[2]{0.0, 0.0};
    result[0] = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    result[1] = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    return result;
}

void Motorworker::Record_Position(int Motor_id, double accel_limit, double position, double velocity_limit, double max_torque, double feedforward_torque, double kp_scale,
                               double kd_scale, double bounds_min, double bounds_max, double Delay) // slot implementation
{
    Rec_run_Enable = false;
    Step_Mode = false;
    Position_wait = false;
    std::ostringstream out;
    out.str("");


    double position_limited = position;
    if (bounds_max != NAN && position_limited > bounds_max)
        position_limited = bounds_max;
    else if (bounds_min != NAN && position_limited < bounds_min)
        position_limited = bounds_min;

    last_record_destination[Motor_id-1] = position_limited; //save this position as the last recorded position for this motor id.

    list_Position.push_back(position_limited);
    list_Motor_id.push_back(Motor_id);
    list_Delay.push_back(Delay);
    list_velocity_limit.push_back(velocity_limit);
    list_Max_torque.push_back(max_torque);
    list_Feedforward_torque.push_back(feedforward_torque);
    list_Kp_scale.push_back(kp_scale);
    list_Kd_scale.push_back(kd_scale);
    list_accel_limit.push_back(accel_limit);
    current_list_index = 0;

    out << "Current position: " << position  << ", Velocity limit: " << velocity_limit << ", Accel limit: " << accel_limit << ", Motor: " << Motor_id << ", Delay: " << Delay << endl;
    emit sendToMain(QString::fromStdString(out.str()));

}

bool Motorworker::Check_Motor(int Motor_id)
{
    bool error = false;
    Motor_error = false;
    TrajectoryComplete_error = false;
    // define a state object
    State curr_state;
    std::ostringstream out;
    out.str("");

    // reset the state
    curr_state.Reset();

    //read status
    curr_state.EN_Fault();
    curr_state.EN_Mode();
    curr_state.EN_TrajectoryComplete();
    ReadState(Motor_id, curr_state);

    Fault fault = static_cast<Fault>(curr_state.fault);
    Mode mode = static_cast<Mode>(curr_state.mode);
    bool TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);

    if (fault != Fault::kNoFault || mode == Mode::kFault || mode == Mode::kPositionTimeout)
    {
        error = true;
        Motor_error = true;
        out.str("");
        out << "Fault detected, Issuing Motor stop first to clear error"  << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        out.str("");
        switch (fault)  {
            case Fault::kNoFault:
                out << "Fault:\t\t" << curr_state.fault << " = no fault" << endl;
                break;

            case Fault::kCalibrationFault:
                out << "Fault:\t\t" << curr_state.fault << " = calibration fault" << endl;
                break;
            case Fault::kMotorDriverFault:
                out << "Fault:\t\t" << curr_state.fault << " = motor driver fault" << endl;
                break;
            case Fault::kOverVoltageFault:
                out << "Fault:\t\t" << curr_state.fault << " =  over voltage fault" << endl;
                break;
            case Fault::kEncoderFault:
                out << "Fault:\t\t" << curr_state.fault << " =  encoder fault" << endl;
                break;
            case Fault::kMotorNotConfiguredFault:
                out << "Fault:\t\t" << curr_state.fault << " = motor not configured fault" << endl;
                break;
            case Fault::kPwmCycleOverrunFault:
                out << "Fault:\t\t" << curr_state.fault << " = pwm cycle overrun fault" << endl;
                break;
            case Fault::kOverTemperatureFault:
                out << "Fault:\t\t" << curr_state.fault << " = over temperature fault" << endl;
                break;
            case Fault::kOutsideLimitFault:
                out << "Fault:\t\t" << curr_state.fault << " = outside limit fault" << endl;
                break;
            default:
                out << "Fault:\t\t" << curr_state.fault << " = unknown fault" << endl;
        }
        switch (mode)  {
            case Mode::kStopped:
                out << "Mode:\t\t" << curr_state.mode << " = Stopped" << endl;
                break;
            case Mode::kFault:
                out << "Mode:\t\t" << curr_state.mode << " = Fault" << endl;
                break;
            case Mode::kEnabling:
                out << "Mode:\t\t" << curr_state.mode << " = Enabling" << endl;
                break;
            case Mode::kCalibrating:
                out << "Mode:\t\t" << curr_state.mode << " = Calibrating" << endl;
                break;
            case Mode::kCalibrationComplete:
                out << "Mode:\t\t" << curr_state.mode << " = CalibrationComplete" << endl;
                break;
            case Mode::kPwm:
                out << "Mode:\t\t" << curr_state.mode << " = Pwm" << endl;
                break;
            case Mode::kVoltage:
                out << "Mode:\t\t" << curr_state.mode << " = Voltage" << endl;
                break;
            case Mode::kVoltageFoc:
                out << "Mode:\t\t" << curr_state.mode << " = VoltageFoc" << endl;
                break;
            case Mode::kVoltageDq:
                out << "Mode:\t\t" << curr_state.mode << " = Voltage Dq" << endl;
                break;
            case Mode::kCurrent:
                out << "Mode:\t\t" << curr_state.mode << " = Current" << endl;
                break;
            case Mode::kPosition:
                out << "Mode:\t\t" << curr_state.mode << " = Position" << endl;
                break;
            case Mode::kPositionTimeout:
                out << "Mode:\t\t" << curr_state.mode << " = Position Timeout" << endl;
                break;
            case Mode::kZeroVelocity:
                out << "Mode:\t\t" << curr_state.mode << " = Zero Velocity" << endl;
                break;
            case Mode::kStayWithin:
                out << "Mode:\t\t" << curr_state.mode << " = Stay Within Bounds" << endl;
                break;
            case Mode::kMeasureInd:
                out << "Mode:\t\t" << curr_state.mode << " = Measure Inductance" << endl;
                break;
            case Mode::kBrake:
                out << "Mode:\t\t" << curr_state.mode << " = Break" << endl;
                break;

            default:
                out << "Mode:\t\t" << curr_state.mode << " = unknown mode" << endl;
        }
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
    {
        error = true;
        TrajectoryComplete_error = true;
        out << "Trajectory is not Complete, waiting for complete "  << " , Motor = " << Motor_id << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    return !error;
}

bool Motorworker::Check_TrajectoryComplete(int Motor_id)
{
    bool error = false;
    TrajectoryComplete_error = false;
    // define a state object
    State curr_state;
    std::ostringstream out;
    out.str("");

    // reset the state
    curr_state.Reset();

    //read current position
    curr_state.EN_Fault();
    curr_state.EN_Mode();
    curr_state.EN_TrajectoryComplete();
    ReadState(Motor_id, curr_state);

    Fault fault = static_cast<Fault>(curr_state.fault);
    Mode mode = static_cast<Mode>(curr_state.mode);
    bool TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);
    if (fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
    {
        error = true;
        TrajectoryComplete_error = true;
    }
    return !error;
}

bool Motorworker::Wait_TrajectoryComplete(int Motor_id)
{
    bool error = false;
    Motor_error = false;
    TrajectoryComplete_error = false;
    // define a state object
    State curr_state;
    std::ostringstream out;
    out.str("");

    // reset the state
    curr_state.Reset();

    //read current position
    curr_state.EN_Fault();
    curr_state.EN_Mode();
    curr_state.EN_TrajectoryComplete();
    ReadState(Motor_id, curr_state);

    Fault fault = static_cast<Fault>(curr_state.fault);
    Mode mode = static_cast<Mode>(curr_state.mode);
    bool TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);
    if (fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
    {
        error = true;
        TrajectoryComplete_error = true;
        // wait until Trajectory Complete is true and check for fault

        out << "Trajectory is not Complete, waiting for complete "  << " , Motor = " << Motor_id << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        int count = Trajectory_Timeout_1; // 20 sec
        while (count > 0 && fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
        {
            QThread::msleep(100);  //Blocking delay 100ms
            curr_state.EN_Fault();
            curr_state.EN_Mode();
            curr_state.EN_TrajectoryComplete();
            ReadState(Motor_id, curr_state);
            fault = static_cast<Fault>(curr_state.fault);
            mode = static_cast<Mode>(curr_state.mode);
            TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);
            count--;
        }
        if (!(count > 0) )
        {
            out.str("");
            out << "Timeout waiting for Trajectory Complete, Motor: " << Motor_id << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
        else if (fault != Fault::kNoFault || mode == Mode::kFault || mode == Mode::kPositionTimeout)
        {
            Motor_error = true;
            out.str("");
            out << "Fault detected, while waiting for Trajectory Complete"  << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
    }
    return !error;
}

bool Motorworker::SendStopCommand(int moteus_id) {
    moteus::Controller::Options options;
    options.id = moteus_id;
    options.default_query = false;
    moteus::Controller controller(options);

    // Command a stop to the controller in order to clear any faults.
    controller.SetStop();

  return true;
}
void Motorworker::ReadState(int moteus_id, State& curr_state) const {
    moteus::Query::Format q_com;
    q_com.mode = mjbots::moteus::Resolution::kInt8;
    q_com.position = mjbots::moteus::Resolution::kFloat;
    q_com.velocity = mjbots::moteus::Resolution::kFloat;
    q_com.torque = mjbots::moteus::Resolution::kFloat;
    q_com.q_current = mjbots::moteus::Resolution::kFloat;
    q_com.d_current = mjbots::moteus::Resolution::kFloat;
    q_com.abs_position = mjbots::moteus::Resolution::kFloat;
    q_com.motor_temperature = mjbots::moteus::Resolution::kFloat;
    q_com.trajectory_complete = mjbots::moteus::Resolution::kInt16;
    q_com.home_state = mjbots::moteus::Resolution::kInt16;
    q_com.voltage = mjbots::moteus::Resolution::kFloat;
    q_com.temperature = mjbots::moteus::Resolution::kFloat;
    q_com.fault = mjbots::moteus::Resolution::kInt8;

  if (!curr_state.position_flag)
    q_com.position = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.velocity_flag)
    q_com.velocity = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.torque_flag)
    q_com.torque = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.q_curr_flag)
    q_com.q_current = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.d_curr_flag)
    q_com.d_current = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.abs_position_flag)
    q_com.abs_position = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.motor_temperature_flag)
    q_com.motor_temperature = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.home_state_flag)
    q_com.home_state = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.TrajectoryComplete_flag)
    q_com.trajectory_complete = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.voltage_flag)
    q_com.voltage = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.temperature_flag)
    q_com.temperature = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.fault_flag)
    q_com.fault = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.mode_flag)
    q_com.mode = mjbots::moteus::Resolution::kIgnore;

  moteus::Controller::Options options;
  options.id = moteus_id;
  options.default_query = false;
  moteus::Controller controller(options);

  const auto maybe_result = controller.SetQuery(&q_com);
  const auto qr = maybe_result->values;
  curr_state.position = qr.position;
  curr_state.velocity = qr.velocity;
  curr_state.torque = qr.torque;
  curr_state.q_curr = qr.q_current;
  curr_state.d_curr = qr.d_current;
  curr_state.voltage = qr.voltage;
  curr_state.temperature = qr.temperature;
  curr_state.fault = qr.fault;
  curr_state.mode = static_cast<float>(qr.mode);
  curr_state.TrajectoryComplete = qr.trajectory_complete;
  curr_state.home_state = static_cast<float>(qr.home_state);
}

bool Motorworker::Collision_Check( int Motor_id, double position)
{
    std::ostringstream out;
    std::istringstream iss;
    double position_1 = 0;
    double position_2 = 0;

    //if a motor is stopped get current position and set the last_position_destination
    // define a state object
    State curr_state;

    // reset the state
    curr_state.Reset();
    curr_state.EN_Position();
    curr_state.EN_Mode();

    //read current velocity
    ReadState(1, curr_state);
    Mode mode1 = static_cast<Mode>(curr_state.mode);
    if(mode1 == Mode::kStopped)
        last_position_destination[0] = curr_state.position;

    ReadState(2, curr_state);
    Mode mode2 = static_cast<Mode>(curr_state.mode);
    if(mode2 == Mode::kStopped)
        last_position_destination[1] = curr_state.position;

    // get next X and Y position
    if (Motor_id == 1)
    {   // motor 1
        position_1 = position;
        position_2 = last_position_destination[1]; // motor 2 last position
    }
    else
    {   // motor 2
        position_1 = last_position_destination[0];
        position_2 = position;
    }

    // convert revolutions to radians
    double theta1 = -position_1 * 2 * std::numbers::pi;
    double theta2 = position_2 * 2 * std::numbers::pi;

    out.str("");

    double* result = forward_kin(theta1, theta2);
    double x = 0;
    double y = 0;

    // get x and y and eliminate very small numbers using format.
    try
    {
        iss.str(std::format("{:.3f}\n" , result[0]));
        iss >> x;

        iss.str(std::format("{:.3f}\n" , result[1]));
        iss >> y;
    }
    catch(std::format_error& error)
    {
        cout  << error.what();
    }

    // now check if the x and y position is legal
    if ( y < min_Y )
    {
       out.str("");
       try
       {
           out << std::format("Y {:.3f} is less than minimum Y {:.3f} ", y,min_Y) << endl;
       }
       catch(std::format_error& error)
       {
           cout  << error.what();
       }

       emit sendToMain(QString::fromStdString(out.str()));
       Rec_run_Enable = false;
       return false;

    }
    else if ( y < 0 && x >= 0 && x < min_Pos_X )
    {
       out.str("");
       try
       {
           out << std::format("X {:.3f} is less than minimum positive X below y 0 {:.3f} ", x, min_Pos_X) << endl;
       }
       catch(std::format_error& error)
       {
           cout  << error.what();
       }

       emit sendToMain(QString::fromStdString(out.str()));
       Rec_run_Enable = false;
       return false;

    }
    else if ( y < 0 && x < 0 && x > min_Neg_X )
    {
       out.str("");
       try
       {
           out << std::format("X {:.3f} is less than minimum negative X below y 0 {:.3f} ", x,min_Neg_X) << endl;
       }
       catch(std::format_error& error)
       {
           cout  << error.what();
       }

       emit sendToMain(QString::fromStdString(out.str()));
       Rec_run_Enable = false;
       return false;
    }
    else
    {
        return true;
    }
}

bool Motorworker::SendPositionCommand(  int Motor_id,
                                        double position,
                                        double velocity_limit,
                                        double accel_limit,
                                        double max_torque,
                                        double feedforward_torque,
                                        double kp_scale,
                                        double kd_scale,
                                        double velocity,
                                        double watchdog_timer)
{
    std::ostringstream out;
    moteus::PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = velocity;
    cmd.maximum_torque = max_torque;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.kp_scale = kp_scale;
    cmd.kd_scale = kd_scale;
    cmd.feedforward_torque = feedforward_torque;
    cmd.velocity_limit = velocity_limit;
    cmd.accel_limit = accel_limit;
    cmd.watchdog_timeout = watchdog_timer;

    moteus::PositionMode::Format res;
    res.position = mjbots::moteus::Resolution::kFloat;
    res.velocity = mjbots::moteus::Resolution::kFloat;
    res.feedforward_torque = mjbots::moteus::Resolution::kFloat;
    res.kp_scale = mjbots::moteus::Resolution::kFloat;
    res.kd_scale = mjbots::moteus::Resolution::kFloat;
    res.maximum_torque = mjbots::moteus::Resolution::kFloat;
    res.stop_position = mjbots::moteus::Resolution::kFloat;
    res.watchdog_timeout = mjbots::moteus::Resolution::kFloat;
    res.velocity_limit = mjbots::moteus::Resolution::kFloat;
    res.accel_limit = mjbots::moteus::Resolution::kFloat;
    res.fixed_voltage_override = mjbots::moteus::Resolution::kIgnore;

    moteus::Controller::Options options;
    options.id = Motor_id;
    options.default_query = false;
    moteus::Controller controller(options);

    if (!Collision_Check( Motor_id, position))
    {
        out << "error on posiion command " << Motor_id << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        return false;
    }

    last_position_destination[Motor_id-1] = position; // remember the last position destination for this motor
    controller.SetPosition(cmd,&res);
    return true;

}
void Motorworker::run_cycles()
{
    if (!Step_Mode && !Rec_run_Enable && Position_wait)
    {
            // check TrajectoryComplete
            if (Check_TrajectoryComplete(position_Motor_id))
            {
                std::ostringstream out;
                Position_wait = false;

                // define a state object
                State curr_state1;

                // reset the state
                curr_state1.Reset();

                //read current position
                curr_state1.EN_Position();
                ReadState(position_Motor_id, curr_state1);

                out << "Motor = " << position_Motor_id << " Ending position:\t" << curr_state1.position << endl;
                emit sendToMain(QString::fromStdString(out.str()));
            }
    }
    else if (Step_Mode || (Rec_run_Enable && l_Cycle <= l_Cycle_Start_Stop))
    {
        if (TrajectoryComplete_pause)
        {
            // check TrajectoryComplete
            if (Check_TrajectoryComplete(l_Motor_id))
            {
               TrajectoryComplete_pause = false;
            }
            return;
        }
        // check if delay is over
        else if (delay >= l_Cycle_Delay)
        {
            // should be true if list is not empty
            if (current_list_index < (int)list_Position.size())
            {
                // start the next in the list
                l_Motor_id = list_Motor_id[current_list_index];

                TrajectoryComplete_pause = false;
                if (!Check_Motor(l_Motor_id))
                {
                   if (Motor_error)
                   {
                       // send one stop command
                       SendStopCommand(l_Motor_id);
                   }
                   else if (TrajectoryComplete_error)
                   {
                       TrajectoryComplete_pause = true;
                   }
                   return;
                }
                delay = 0;
                l_Cycle_Delay = list_Delay[current_list_index];

                std::ostringstream out;

                if (Dynamic && Dynamic_Motor_id == list_Motor_id[current_list_index])
                {
                    if (!SendPositionCommand(list_Motor_id[current_list_index],
                                            list_Position[current_list_index],
                                            l_velocity_limit,
                                            l_accel_limit,
                                            l_max_torque,
                                            l_feedforward_torque,
                                            l_kp_scale,
                                            l_kd_scale,
                                            0.0 // end velocity
                                            ))
                    {
                        out << "error on posiion command " << l_Motor_id << endl;
                        emit sendToMain(QString::fromStdString(out.str()));
                    }
                }
                else
                {
                    if (!SendPositionCommand(list_Motor_id[current_list_index],
                                            list_Position[current_list_index],
                                            list_velocity_limit[current_list_index],
                                            list_accel_limit[current_list_index],
                                            list_Max_torque[current_list_index],
                                            list_Feedforward_torque[current_list_index],
                                            list_Kp_scale[current_list_index],
                                            list_Kd_scale[current_list_index],
                                            0.0 // end velocity
                                            ))
                    {
                        out << "error on posiion command " << l_Motor_id << endl;
                        emit sendToMain(QString::fromStdString(out.str()));
                    }
                }

                out.str("");
                if (Dynamic && Dynamic_Motor_id == list_Motor_id[current_list_index])
                {
                    try
                    {
                        out << std::format("Position to: {:.3f}", list_Position[current_list_index])
                            << std::format(", Velocity: {:.3f}", l_velocity_limit)
                            << std::format(", Accel: {:.3f}", l_accel_limit)
                            << std::format(", Motor: {}", list_Motor_id[current_list_index])
                            << std::format(", Max torque: {:.3f}", l_max_torque)
                            << std::format(", Feedforward torque: {:.3f}", l_feedforward_torque)
                            << std::format(", KP scale: {:.3f}", l_kp_scale)
                            << std::format(", KD scale: {:.3f}", l_kd_scale)
                            << endl;
                    }
                    catch(std::format_error& error)
                    {
                        cout  << error.what();
                    }

                }
                else
                {
                    try
                    {
                        out << std::format("Position to: {:.3f}", list_Position[current_list_index])
                            << std::format(", Velocity: {:.3f}", list_velocity_limit[current_list_index])
                            << std::format(", Accel: {:.3f}", list_accel_limit[current_list_index])
                            << std::format(", Motor: {}", list_Motor_id[current_list_index])
                            << std::format(", Delay: {}", list_Delay[current_list_index])
                            << endl;
                    }
                    catch(std::format_error& error)
                    {
                        cout  << error.what();
                    }
                }
                emit sendToMain(QString::fromStdString(out.str()));

                // go to next list entry
                current_list_index++ ;
                if (current_list_index >= (int)list_Position.size())
                {
                    // the list is done start the list over
                    current_list_index = 0;
                    if(Rec_run_Enable)
                    {
                        // start the next cycle if there is one
                        l_Cycle += 1;
                        if (l_Cycle >= l_Cycle_Start_Stop)
                        {
                           Rec_run_Enable = false;
                           out.str("");
                           out << "Cycle:\t" << l_Cycle << "\tRun Cycles Done" << endl;
                           emit sendToMain(QString::fromStdString(out.str()));
                        }
                        else
                        {
                            out.str("");
                            out << "Cycle:\t" << l_Cycle << endl;
                            emit sendToMain(QString::fromStdString(out.str()));
                        }
                    }
                }
                Step_Mode =false;
            }
        }
        else
        {
             delay += .01;
        }
    }
}

void Motorworker::receiveSetup()
{
    TrajectoryComplete_pause = false;
    Rec_run_Enable = false;
    Step_Mode = false;
    current_list_index = 0;
    std::ostringstream out;
    out << " Run Cycles Stopped" << endl;
    emit sendToMain(QString::fromStdString(out.str()));
}
void Motorworker::getFromMain_file_commands(QString msg, QString file_name) // slot implementation
{
    // file commands
    if (msg == "Save File")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        string fileName = file_name.toStdString();
        ofstream db;
        db.open(fileName);
        if( db.is_open() )
        {
            for( current_list_index=0; current_list_index < (int)list_Position.size(); current_list_index++ )
            {
                db << current_list_index << endl;
                db << "Position: " << list_Position[current_list_index] << endl
                     << "Motor: " << list_Motor_id[current_list_index] << endl
                     << "Delay: " << list_Delay[current_list_index] << endl
                     << "Velocity_limit: " << list_velocity_limit[current_list_index] << endl
                     << "Accel_limit: " << list_accel_limit[current_list_index] << endl
                     << "Max_torque: " << list_Max_torque[current_list_index] << endl
                     << "Feedforward_torque: " << list_Feedforward_torque[current_list_index] << endl
                     << "Kp_scale: " << list_Kp_scale[current_list_index]  << endl
                     << "Kd_scale: " << list_Kd_scale[current_list_index] << endl ;
            }
        }
        else
        {
            out.str("");
            out << "Cannot open file for writing." << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

        db.close();
        current_list_index = 0;

    }
    else if (msg == "Open File")
    {
        Rec_run_Enable = false;
        Step_Mode = false;
        Position_wait = false;

        std::ostringstream out;

        QString fileName = file_name;
        QString line;
        int val_int;
        double val_double;
        string val_string;
        std::istringstream iss;
        QFile file(fileName);
        if (file.open(QIODevice::ReadOnly | QIODevice::Text))
        {

            list_Position.clear();
            list_Motor_id.clear();
            list_Delay.clear();
            list_velocity_limit.clear();
            list_Max_torque.clear();
            list_Feedforward_torque.clear();
            list_Kp_scale.clear();
            list_Kd_scale.clear();
            list_accel_limit.clear();
            while  (!file.atEnd())
            {
                line = file.readLine(); // read in sequence number
                iss.str(line.toStdString());
                iss >> val_int;
                if (iss.fail()) // check if not a number
                {
                    // something wrong happened
                    iss.clear();
                    out.str("");
                    out << "sequence number in recorded file is not a number. Check for extra eol at file end" << endl;
                    emit sendToMain(QString::fromStdString(out.str()));
                    break;
                }

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_Position.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_int;
                list_Motor_id.push_back(val_int);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_Delay.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_velocity_limit.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_accel_limit.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_Max_torque.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_Feedforward_torque.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_Kp_scale.push_back(val_double);

                line = file.readLine();
                iss.str(line.toStdString());
                iss >> val_string;
                iss >> val_double;
                list_Kd_scale.push_back(val_double);

            }
        }
        else
        {
            out.str("");
            out << "Cannot open file for reading.\n" << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

        file.close();
        current_list_index = 0;

    }
    else
    {
        emit sendToMain(msg);
    }
}

void Motorworker::getFromMain_motor_commands(QString msg, int Motor_id) // slot implementation
{

    // status stuff
    if (msg == "Update Position")
    {
        double position_1 = 1.123;
        double position_2 = 2.123;
        double position_3 = 3.123;

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();
        curr_state.EN_Position();

        //read current velocity
        ReadState(1, curr_state);
        position_1 = curr_state.position;

        ReadState(2, curr_state);
        position_2 = curr_state.position;

        ReadState(3, curr_state);
        position_3 = curr_state.position;

        // reset the state
        curr_state.Reset();

        //read current velocity
        curr_state.EN_Velocity();
        curr_state.EN_Torque();
        curr_state.EN_Temp();
        curr_state.EN_QCurr();

        ReadState(Motor_id, curr_state);
        emit sendMsg("get Position",Motor_id,position_1,curr_state.velocity,curr_state.torque,curr_state.temperature,curr_state.q_curr,position_2,position_3);
    }
    else if (msg == "Update Velocity")
    {
        double position_1 = 1.123;
        double position_2 = 2.123;
        double position_3 = 3.123;

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();
        curr_state.EN_Position();

        //read current velocity
        ReadState(1, curr_state);
        position_1 = curr_state.position;

        ReadState(2, curr_state);
        position_2 = curr_state.position;

        ReadState(3, curr_state);
        position_3 = curr_state.position;

        // reset the state
        curr_state.Reset();

        //read current velocity
        curr_state.EN_Velocity();
        curr_state.EN_Torque();
        curr_state.EN_Temp();
        curr_state.EN_QCurr();

        ReadState(Motor_id, curr_state);
        emit sendMsg("get velocity",Motor_id,position_1,curr_state.velocity,curr_state.torque,curr_state.temperature,curr_state.q_curr,position_2,position_3);
    }
    else if (msg == "Read_Status")
    {
        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();

        // enable read register flags
        curr_state.EN_Position();
        curr_state.EN_Velocity();
        curr_state.EN_Torque();
        curr_state.EN_QCurr();
        curr_state.EN_DCurr();
        curr_state.EN_Voltage();
        curr_state.EN_Temp();
        curr_state.EN_Fault();
        curr_state.EN_Mode();
        curr_state.EN_TrajectoryComplete();

        // read registers
        ReadState(Motor_id, curr_state);

        // print everyting
        try
        {
            out << std::format("Position:\t\t{:.6f}", curr_state.position) << endl;
            out << std::format("Velocity:\t\t{:.6f}", curr_state.velocity) << endl;
            out << std::format("Torque:\t\t{:.6f}", curr_state.torque) << endl;
            out << std::format("Q Current:\t\t{:.6f}", curr_state.q_curr) << endl;
            out << std::format("D Current:\t\t{:.6f}", curr_state.d_curr) << endl;
            out << std::format("Voltage:\t\t{:.2f}", curr_state.voltage) << endl;
            out << std::format("Temperature:\t{:.2f}", curr_state.temperature) << endl;
        }
        catch(std::format_error& error)
        {
            cout  << error.what();
        }

        out << "Trajectory Complete:\t";

        if  (curr_state.TrajectoryComplete != 0)
        {
            out << "True" << endl;
        }
        else
        {
            out << "False" << endl;
        }

        unsigned int fault = static_cast<unsigned int>(curr_state.fault);
        switch (fault)
        {
            case 0:
                out << "Fault:\t\t" << curr_state.fault << " = no fault" << endl;
                break;

            case 32:
                out << "Fault:\t\t" << curr_state.fault << " = calibration fault" << endl;
                break;
            case 33:
                out << "Fault:\t\t" << curr_state.fault << " = motor driver fault" << endl;
                break;
            case 34:
                out << "Fault:\t\t" << curr_state.fault << " =  over voltage fault" << endl;
                break;
            case 35:
                out << "Fault:\t\t" << curr_state.fault << " =  encoder fault" << endl;
                break;
            case 36:
                out << "Fault:\t\t" << curr_state.fault << " = motor not configured fault" << endl;
                break;
            case 37:
                out << "Fault:\t\t" << curr_state.fault << " = pwm cycle overrun fault" << endl;
                break;
            case 38:
                out << "Fault:\t\t" << curr_state.fault << " = over temperature fault" << endl;
                break;
            case 39:
                out << "Fault:\t\t" << curr_state.fault << " = outside limit fault" << endl;
                break;
            default:
                out << "Fault:\t\t" << curr_state.fault << " = unknown fault" << endl;
        }
        unsigned int mode = static_cast<unsigned int>(curr_state.mode);
        switch (mode)
        {
            case 0:
                out << "Mode:\t\t" << curr_state.mode << " = Stopped" << endl;
                break;
            case 1:
                out << "Mode:\t\t" << curr_state.mode << " = Fault" << endl;
                break;
            case 2:
                out << "Mode:\t\t" << curr_state.mode << " = Enabling" << endl;
                break;
            case 3:
                out << "Mode:\t\t" << curr_state.mode << " = Calibrating" << endl;
                break;
            case 4:
                out << "Mode:\t\t" << curr_state.mode << " = CalibrationComplete" << endl;
                break;
            case 5:
                out << "Mode:\t\t" << curr_state.mode << " = Pwm" << endl;
                break;
            case 6:
                out << "Mode:\t\t" << curr_state.mode << " = Voltage" << endl;
                break;
            case 7:
                out << "Mode:\t\t" << curr_state.mode << " = VoltageFoc" << endl;
                break;
            case 8:
                out << "Mode:\t\t" << curr_state.mode << " = Voltage Dq" << endl;
                break;
            case 9:
                out << "Mode:\t\t" << curr_state.mode << " = Current" << endl;
                break;
            case 10:
                out << "Mode:\t\t" << curr_state.mode << " = Position" << endl;
                break;
            case 11:
                out << "Mode:\t\t" << curr_state.mode << " = Position Timeout" << endl;
                break;
            case 12:
                out << "Mode:\t\t" << curr_state.mode << " = Zero Velocity" << endl;
                break;
            case 13:
                out << "Mode:\t\t" << curr_state.mode << " = Stay Within Bounds" << endl;
                break;
            case 14:
                out << "Mode:\t\t" << curr_state.mode << " = Measure Inductance" << endl;
                break;

            default:
                out << "Mode:\t\t" << curr_state.mode << " = unknown mode" << endl;
        }

        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Send Stop")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        // Command a stop to the controller in order to clear any faults.
        controller.SetStop();

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        QThread::msleep(1500);  //Blocking delay 1500ms

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        ReadState(Motor_id, curr_state);
        out << "Motor: " << Motor_id << " Stopped position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }

    else if (msg == "Clear Recorded")
    {
        Rec_run_Enable = false;
        Step_Mode = false;
        Position_wait = false;

        list_Position.clear();
        list_Motor_id.clear();
        list_Delay.clear();
        list_velocity_limit.clear();
        list_Max_torque.clear();
        list_Feedforward_torque.clear();
        list_Kp_scale.clear();
        list_Kd_scale.clear();
        list_accel_limit.clear();
        current_list_index = 0;

        emit sendToMain(msg);
    }
    else if (msg == "Set Dynamic")
    {
        Dynamic = true;
        Dynamic_Motor_id = Motor_id;
    }
    else if (msg == "Clear Dynamic")
    {
        Dynamic = false;
    }
    else if (msg == "Check Device")
    {
        std::ostringstream out;
        out.str("");

        bool device_evable = false;
         try {
            moteus::Controller::ProcessTransportArgs({});
            moteus::Controller::Options options;
            options.id = Motor_id;
            options.default_query = false;
            moteus::Controller controller(options);

            controller.DiagnosticWrite("tel stop\n");
            controller.DiagnosticFlush();
            device_evable = true;

         } catch (std::exception& e) {
            cout << "Could not open moteus transport: " << e.what() << "\n";
            device_evable = false;
         }

        emit sendMsg("Check Device",device_evable,0,0,0,0,0,0,0);
    }
    else
    {
        emit sendToMain(msg);
    }
}
void Motorworker::Motorworker::getFromMain_diagnostic_write_commands(QString msg, int Motor_id, double Value1, double Value2, double Value3)
{
    if (msg == "set PID")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        std::ostringstream out;

        // set KP command
        out.str("");
        out << "conf set servo.pid_position.kp " << Value1 << endl;
        controller.DiagnosticCommand(out.str());

        // set KD command
        out.str("");
        out << "conf set servo.pid_position.kd " << Value2 << endl;
        controller.DiagnosticCommand(out.str());

        // set Ki command
        out.str("");
        out << "conf set servo.pid_position.ki " << Value3 << endl;
        controller.DiagnosticCommand(out.str());

        out.str("");
        out << "Motor: " << Motor_id
            << "\tkp: " << Value1
            << "\tkd: " << Value2
            << "\t ki: " << Value3
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "Set Output Nearest")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        if (Value2 > 0)
        {
            // if value 2 is non zero, set nearsest is conditional on home state being < 2
            curr_state.Reset();
            curr_state.EN_home_state();
            ReadState(Motor_id, curr_state);
            if (curr_state.home_state > 1)
                return;
        }

        moteus::OutputNearest::Command cmd;
        cmd.position = Value1;

        moteus::OutputNearest::Format res;

        // Command OutputNearest
        controller.SetOutputNearest(cmd,&res);


        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        curr_state.EN_home_state();
        ReadState(Motor_id, curr_state);

        out.str("");
        try
        {
            out << std::format("Motor: {} Position:\t{:.6f}", Motor_id , curr_state.position) << endl;
        }
        catch(std::format_error& error)
        {
            cout  << error.what();
        }
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set rotor_to_output_ratio")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        std::ostringstream out;
        out.str("");

        // set rotor_to_output_ratiocommand
        out << "conf set motor_position.rotor_to_output_ratio " << Value1 << endl;
        controller.DiagnosticCommand(out.str());

        out.str("");
        out << "Motor: " << Motor_id
            << "\t Gear Ratio: " << Value1
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set break voltage")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        std::ostringstream out;
        out.str("");

        // set break voltage command
        out << "conf set servo.flux_brake_min_voltage " << Value1 << endl;
        controller.DiagnosticCommand(out.str());

        out.str("");
        out << "Motor: " << Motor_id
            << "\t Break Min Voltage: " << Value1
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set Position Offset")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        std::ostringstream out;
        out.str("");

        // set Position Offset command
        out << "conf set motor_position.output.offset " << Value1 << endl;
        controller.DiagnosticCommand(out.str());

        out.str("");
        out << "Motor: " << Motor_id
            << "\t motor_position.output.offset: " << Value1
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set motor limits")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        std::ostringstream out;
        out.str("");

        // set min limit command
        out << "conf set servopos.position_min " << Value1 << endl;
        controller.DiagnosticCommand(out.str());
        l_bounds_min[Motor_id-1] = Value1;

        // set max limit command
        out.str("");
        out << "conf set servopos.position_max " << Value2 << endl;
        controller.DiagnosticCommand(out.str());
        l_bounds_max[Motor_id-1] = Value2;

        out.str("");
        out << "Motor: " << Motor_id
            << " limit min:\t" << l_bounds_min[Motor_id-1]
            << "\tlimit max:\t" << l_bounds_max[Motor_id-1] << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "conf write")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        std::ostringstream out;
        out.str("");

        // save conf
        controller.DiagnosticCommand("conf write");

        out.str("");
        out << "configuration write" << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }

    else if (msg == "Get Cur X,Y")
    {
        std::ostringstream out;
        std::istringstream iss;

        double X = 0;
        double Y = 0;

        double position_1 = 0;
        double position_2 = 0;

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();
        curr_state.EN_Position();

        //read current velocity
        ReadState(1, curr_state);
        position_1 = curr_state.position;

        ReadState(2, curr_state);
        position_2 = curr_state.position;

        double motor1_revolutions = position_1;
        double motor2_revolutions = position_2;

        // convert revolutions to radians
        double theta1 = -motor1_revolutions * 2 * std::numbers::pi;
        double theta2 = motor2_revolutions * 2 * std::numbers::pi;

        out.str("");

        double* result = forward_kin(theta1, theta2);

        // Eliminate very small numbers using format.
        try
        {
            iss.str(std::format("{:.3f}\n" , result[0]));
            iss >> X;

            iss.str(std::format("{:.3f}\n" , result[1]));
            iss >> Y;
        }
        catch(std::format_error& error)
        {
            cout  << error.what();
        }
        emit sendMsg("Get Cur X,Y",0,X,Y,0,0,0,0,0);
    }
    else
    {
        emit sendToMain(msg);
    }
}

void Motorworker::getFromMain_diagnostic_read_commands(QString msg, int Motor_id)
{
    if (msg == "get rotor_to_output_ratio")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        double value = std::numeric_limits<double>::quiet_NaN();

        std::ostringstream out;
        // get rotor_to_output_ratio
            value = std::stod(
                controller.DiagnosticCommand("conf get motor_position.rotor_to_output_ratio",
                                             moteus::Controller::kExpectSingleLine));
       emit sendMsg("get gear ratio",Motor_id,value,0,0,0,0,0,0);

    }
    else if (msg == "get break voltage")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        double value = std::numeric_limits<double>::quiet_NaN();

        std::ostringstream out;

        // get min break voltage
        value = std::stod(
            controller.DiagnosticCommand("conf get servo.flux_brake_min_voltage",
                                         moteus::Controller::kExpectSingleLine));

        emit sendMsg("get Break Voltage",Motor_id,value,0,0,0,0,0,0);
    }
    else if (msg == "get Position Offset")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        double value = std::numeric_limits<double>::quiet_NaN();

        std::ostringstream out;

        // get min break voltage
        value = std::stod(
            controller.DiagnosticCommand("conf get motor_position.output.offset",
                                         moteus::Controller::kExpectSingleLine));
        emit sendMsg("get Position Offset",Motor_id,value,0,0,0,0,0,0);
    }
    else if (msg == "get motor limits")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        // get min limit command
        l_bounds_min[Motor_id-1] = std::stod(
            controller.DiagnosticCommand("conf get servopos.position_min",
                                         moteus::Controller::kExpectSingleLine));

        // get max limit command
        l_bounds_max[Motor_id-1] = std::stod(
            controller.DiagnosticCommand("conf get servopos.position_max",
                                         moteus::Controller::kExpectSingleLine));

        emit sendMsg("set motor limits",Motor_id,l_bounds_min[Motor_id-1],l_bounds_max[Motor_id-1],0,0,0,0,0);
    }
    else if (msg == "get PID")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        moteus::Controller::Options options;
        options.id = Motor_id;
        options.default_query = false;
        moteus::Controller controller(options);

        double value1 = std::numeric_limits<double>::quiet_NaN();
        double value2 = std::numeric_limits<double>::quiet_NaN();
        double value3 = std::numeric_limits<double>::quiet_NaN();

        // get kp command
        value1 = std::stod(
            controller.DiagnosticCommand("conf get servo.pid_position.kp",
                                         moteus::Controller::kExpectSingleLine));
        // get kd command
        value2 = std::stod(
            controller.DiagnosticCommand("conf get servo.pid_position.kd",
                                         moteus::Controller::kExpectSingleLine));
        // get ki command
        value3 = std::stod(
            controller.DiagnosticCommand("conf get servo.pid_position.ki",
                                         moteus::Controller::kExpectSingleLine));

        emit sendMsg("get PID",Motor_id,value1,value2,value3,0,0,0,0);
    }
    else
    {
        emit sendToMain(msg);
    }
}

void Motorworker::getFromMain_position_commands(QString msg, int Motor_id, double accel_limit, double position, double velocity_limit, double max_torque, double feedforward_torque, double kp_scale,
                               double kd_scale, double bounds_min, double bounds_max, double Cycle, double Delay,double position_X,double position_Y) // slot implementation
{
    if (msg == "Record Position")
    {
        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        ReadState(Motor_id, curr_state);
        Record_Position(Motor_id, accel_limit,curr_state.position, velocity_limit,max_torque, feedforward_torque,kp_scale,
                                       kd_scale, bounds_min,bounds_max,Delay);
    }
    else if (msg == "Run Recorded")
    {
        Rec_run_Enable = false;
        Step_Mode = false;
        Position_wait = false;

        l_velocity_limit = velocity_limit;
        l_accel_limit = accel_limit;
        l_max_torque = max_torque;
        l_feedforward_torque = feedforward_torque;
        l_kp_scale = kp_scale;
        l_kd_scale = kd_scale;
        l_Cycle_Start_Stop = Cycle;
        current_list_index = 0;
        l_Cycle = 0;
        // set l_Cycle_Delay and delay equal to cause first list load
        l_Cycle_Delay = 1;
        delay = 1;
       if (!list_Position.empty()) // check if list is empty
        {
           TrajectoryComplete_pause = false;
           Rec_run_Enable = true;
        }
    }
    else if (msg == "Step Recorded")
    {
        Rec_run_Enable = false;
        Step_Mode = false;
        Position_wait = false;

        l_velocity_limit = velocity_limit;
        l_accel_limit = accel_limit;
        l_max_torque = max_torque;
        l_feedforward_torque = feedforward_torque;
        l_kp_scale = kp_scale;
        l_kd_scale = kd_scale;

        l_Cycle_Start_Stop = Cycle;
        l_Cycle = 0;
        // set l_Cycle_Delay and delay equal to cause first list load
        l_Cycle_Delay = 1;
        delay = 1;
       if (!list_Position.empty()) // check if list is empty
        {
           TrajectoryComplete_pause = false;
           Step_Mode = true;
        }
    }    else if (msg == "Update Dynamic")
    {
        l_velocity_limit = velocity_limit;
        l_accel_limit = accel_limit;
        l_max_torque = max_torque;
        l_feedforward_torque = feedforward_torque;
        l_kp_scale = kp_scale;
        l_kd_scale = kd_scale;
        Dynamic_Motor_id = Motor_id;
    }

    else if (msg == "Send Start")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               SendStopCommand(Motor_id);
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(Motor_id);
           }
        }

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        ReadState(Motor_id, curr_state);

        std::ostringstream out;
        out.str("");
        out << "Current position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        if (!SendPositionCommand(   Motor_id,
                                    curr_state.position,
                                    velocity_limit,
                                    accel_limit,
                                    max_torque,
                                    feedforward_torque,
                                    kp_scale,
                                    kd_scale,
                                    0.0 // end velocity
                                    ))
        {
            out << "error on posiion command " << Motor_id << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

        position_Motor_id = Motor_id;
        Position_wait = true;
     }
    else if (msg == "Go To Rest Position")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               SendStopCommand(Motor_id);
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(Motor_id);
           }
        }

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        std::ostringstream out;
        out.str("");
        double position_limited = position;
        if (bounds_max != NAN && position_limited > bounds_max)
            position_limited = bounds_max;
        else if (bounds_min != NAN && position_limited < bounds_min)
            position_limited = bounds_min;
        if (!SendPositionCommand( Motor_id,
                                  position_limited, // position
                                  velocity_limit,
                                  accel_limit,
                                  max_torque,
                                  feedforward_torque,
                                  kp_scale,
                                  kd_scale,
                                  0.0 // end velocity
                                ))
        {
            out << "error on posiion command " << Motor_id << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
        else
        {
            Wait_TrajectoryComplete(Motor_id);

            //read current position
            curr_state.Reset();
            curr_state.EN_Position();
            ReadState(Motor_id, curr_state);
            out << "Motor: " << Motor_id << " Position:\t" << curr_state.position << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

    }
    else if (msg == "Go To Position")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               SendStopCommand(Motor_id);
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(Motor_id);
           }
        }

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        ReadState(Motor_id, curr_state);

        std::ostringstream out;
        out.str("");
        out << "Motor = " << Motor_id << " Starting position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

        double position_limited = position;
        if (bounds_max != NAN && position_limited > bounds_max)
            position_limited = bounds_max;
        else if (bounds_min != NAN && position_limited < bounds_min)
            position_limited = bounds_min;

        if (!SendPositionCommand(   Motor_id,
                                    position_limited,
                                    velocity_limit,
                                    accel_limit,
                                    max_torque,
                                    feedforward_torque,
                                    kp_scale,
                                    kd_scale,
                                    0.0 // end velocity
                                    ))
        {
            out << "error on posiion command " << Motor_id << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

        position_Motor_id = Motor_id;
        Position_wait = true;

    }
    else if (msg == "Run Forever")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               SendStopCommand(Motor_id);
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(Motor_id);
           }
        }

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        ReadState(Motor_id, curr_state);

        out.str("");
        out << "Starting position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        float final_velocity = velocity_limit;
        if (position < 0.0)
            final_velocity = -velocity_limit;

        if (!SendPositionCommand(Motor_id,
                                curr_state.position,
                                velocity_limit,
                                accel_limit,
                                max_torque,
                                feedforward_torque,
                                kp_scale,
                                kd_scale,
                                final_velocity // end velocity
                                ))
        {
            out << "error on posiion command " << Motor_id << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

        position_Motor_id = Motor_id;
        Position_wait = true;
    }
    else if (msg == "Record X,Y")
    {
        std::ostringstream out;

        double x = position_X;
        double y = position_Y;

        double outer_r = L1+L2;

        double dist = x * x  + y  * y ;
        if ( !(dist < outer_r * outer_r))
        {
           out.str("");
           try
           {
               out << std::format("X {:.3f} ,Y {:.3f} is not inside the radius of arm1 {:.3f} + arm2 {:.3f} ", x , y,L1,L2) << endl;
           }
           catch(std::format_error& error)
           {
               cout  << error.what();
           }

           emit sendToMain(QString::fromStdString(out.str()));
        }
        else if ( inner_radius > 0 && (dist < inner_radius * inner_radius) )
        {
           out.str("");
           try
           {
               out << std::format("X {:.3f} ,Y {:.3f} is not outside inner radius of {:.3f} ", x , y,inner_radius) << endl;
           }
           catch(std::format_error& error)
           {
               cout  << error.what();
           }

           emit sendToMain(QString::fromStdString(out.str()));
        }
        else if ( y < min_Y )
        {
           out.str("");
           try
           {
               out << std::format("Y {:.3f} is less than minimum Y {:.3f} ", y,min_Y) << endl;
           }
           catch(std::format_error& error)
           {
               cout  << error.what();
           }

           emit sendToMain(QString::fromStdString(out.str()));
        }
        else if ( y < 0 && x >= 0 && x < min_Pos_X )
        {
           out.str("");
           try
           {
               out << std::format("X {:.3f} is less than minimum positive X below y 0 {:.3f} ", x, min_Pos_X) << endl;
           }
           catch(std::format_error& error)
           {
               cout  << error.what();
           }

           emit sendToMain(QString::fromStdString(out.str()));
        }
        else if ( y < 0 && x < 0 && x > min_Neg_X )
        {
           out.str("");
           try
           {
               out << std::format("X {:.3f} is less than minimum negative X below y 0 {:.3f} ", x,min_Neg_X) << endl;
           }
           catch(std::format_error& error)
           {
               cout  << error.what();
           }

           emit sendToMain(QString::fromStdString(out.str()));
        }
        else
        {

            double * theta = inverse_kin(x,y);

            out.str("");

            // convert radians to revolutions
            double motor1_revolutions1 = theta[0]/(2*std::numbers::pi);
            double motor2_revolutions1 = theta[1]/(2*std::numbers::pi);
            double motor1_revolutions2 = theta[2]/(2*std::numbers::pi);
            double motor2_revolutions2 = theta[3]/(2*std::numbers::pi);

            position_Gen_Elbow_Up[0] = motor1_revolutions2;
            position_Gen_Elbow_Up[1] = motor2_revolutions2;
            position_Gen_Elbow_Down[0] = motor1_revolutions1;
            position_Gen_Elbow_Down[1] = motor2_revolutions1;

            bool Elbow_Up_limit_error = false;
            bool Elbow_Down_limit_error = false;

            // check for position outside limits
            if (    (l_bounds_max[0] != NAN && position_Gen_Elbow_Up[0] > l_bounds_max[0]) ||
                    (l_bounds_min[0] != NAN && position_Gen_Elbow_Up[0] < l_bounds_min[0]) ||
                    (l_bounds_max[1] != NAN && position_Gen_Elbow_Up[1] > l_bounds_max[1]) ||
                    (l_bounds_min[1] != NAN && position_Gen_Elbow_Up[1] < l_bounds_min[1]) ||
                    (position_Gen_Elbow_Up[1] > Motor2_rotation_limit)  ||
                    (position_Gen_Elbow_Up[1] < -Motor2_rotation_limit))
            {
                Elbow_Up_limit_error = true;
            }

            if (    (l_bounds_max[0] != NAN && position_Gen_Elbow_Down[0] > l_bounds_max[0]) ||
                    (l_bounds_min[0] != NAN && position_Gen_Elbow_Down[0] < l_bounds_min[0]) ||
                    (l_bounds_max[1] != NAN && position_Gen_Elbow_Down[1] > l_bounds_max[1]) ||
                    (l_bounds_min[1] != NAN && position_Gen_Elbow_Down[1] < l_bounds_min[1]) ||
                    (position_Gen_Elbow_Down[1] > Motor2_rotation_limit)  ||
                    (position_Gen_Elbow_Down[1] < -Motor2_rotation_limit))
            {
                Elbow_Down_limit_error = true;
            }

            try
            {
                if (Elbow_Down_limit_error)
                    out << "Elbow Down motor limit error" << endl;
                if (Elbow_Up_limit_error)
                    out << "Elbow Up motor limit error" << endl;

                out << std::format("motor 1 revolutions1 = \t{:.6f} \tmotor 2 revolutions1 =\t{:.6f}", motor1_revolutions1 , motor2_revolutions1) << endl;
                out << std::format("motor 1 revolutions2 = \t{:.6f} \tmotor 2 revolutions2 =\t{:.6f}", motor1_revolutions2 , motor2_revolutions2) << endl;
            }
            catch(std::format_error& error)
            {
                cout  << error.what();
            }

            emit sendToMain(QString::fromStdString(out.str()));

            if (list_Position.empty())
            {
                // define a state object
                State curr_state;

                // reset the state
                curr_state.Reset();
                curr_state.EN_Position();
                //read current position
                ReadState(1, curr_state);
                last_record_destination[0] = curr_state.position;
                ReadState(2, curr_state);
                last_record_destination[1] = curr_state.position;
            }

            if ( !Elbow_Up_limit_error || !Elbow_Down_limit_error)
            {
                if (!Elbow_Up_limit_error)
                {
                    // if X > 0 && motor 1 going positive, move motor 2 first
                    // if X < 0 && motor 1 going negative, move motor 2 first
                    if ( (x >= 0 && position_Gen_Elbow_Up[0] > last_record_destination[0]) ||
                         (x <= 0 && position_Gen_Elbow_Up[0] < last_record_destination[0]) )
                    {
                        //Position motor 2
                        Record_Position(2,accel_limit,position_Gen_Elbow_Up[1],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[1],l_bounds_max[1],Delay);
                        //Position motor 1
                        Record_Position(1,accel_limit,position_Gen_Elbow_Up[0],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[0],l_bounds_max[0],Delay);
                    }
                    else
                    {
                        //Position motor 1
                        Record_Position(1,accel_limit,position_Gen_Elbow_Up[0],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[0],l_bounds_max[0],Delay);
                        //Position motor 2
                        Record_Position(2,accel_limit,position_Gen_Elbow_Up[1],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[1],l_bounds_max[1],Delay);
                    }
                }
                else if (!Elbow_Down_limit_error)
                {
                    // if X > 0 && motor 1 going positive, move motor 2 first
                    // if X < 0 && motor 1 going negative, move motor 2 first
                    if ( (x >= 0 && position_Gen_Elbow_Down[0] > last_record_destination[0]) ||
                         (x <= 0 && position_Gen_Elbow_Down[0] < last_record_destination[0]) )
                    {
                        //Position motor 2
                        Record_Position(2,accel_limit,position_Gen_Elbow_Down[1],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[1],l_bounds_max[1],Delay);
                        //Position motor 1
                        Record_Position(1,accel_limit,position_Gen_Elbow_Down[0],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[0],l_bounds_max[0],Delay);
                    }
                    else
                    {
                        //Position motor 1
                        Record_Position(1,accel_limit,position_Gen_Elbow_Down[0],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[0],l_bounds_max[0],Delay);
                        //Position motor 2
                        Record_Position(2,accel_limit,position_Gen_Elbow_Down[1],velocity_limit,max_torque,feedforward_torque,kp_scale,
                                          kd_scale,l_bounds_min[1],l_bounds_max[1],Delay);
                    }
                }
            }

        }

    }
    else
    {
        emit sendToMain(msg);
    }
}
