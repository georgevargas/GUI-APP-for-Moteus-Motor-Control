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
bool Motorworker::Check_Motor(QString dev_name,int Motor_id)
{
    MoteusAPI api(dev_name.toStdString(), Motor_id);
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
    api.ReadState(curr_state);

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
bool Motorworker::Check_TrajectoryComplete(QString dev_name,int Motor_id)
{
    MoteusAPI api(dev_name.toStdString(), Motor_id);
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
    api.ReadState(curr_state);

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
bool Motorworker::Wait_TrajectoryComplete(QString dev_name,int Motor_id)
{
    MoteusAPI api(dev_name.toStdString(), Motor_id);
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
    api.ReadState(curr_state);

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
            api.ReadState(curr_state);
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
bool Motorworker::Check_Velocity(QString dev_name,int Motor_id)
{
    MoteusAPI api(dev_name.toStdString(), Motor_id);
    // define a state object
    State curr_state;
    std::ostringstream out;
    out.str("");

    // reset the state
    curr_state.Reset();

    //read current velocity
    curr_state.EN_Position();
    curr_state.EN_Velocity();
    curr_state.EN_Torque();
    curr_state.EN_Temp();
    curr_state.EN_QCurr();

    api.ReadState(curr_state);
    emit sendMsg("get velocity",Motor_id,curr_state.position,curr_state.velocity,curr_state.torque,curr_state.temperature,curr_state.q_curr);
    return true;
}
void Motorworker::run_cycles()
{
    if (!Step_Mode && !Rec_run_Enable && Position_wait)
    {
        // check TrajectoryComplete
        if (Check_TrajectoryComplete(Position_dev_name,position_Motor_id))
        {
            std::ostringstream out;
            Position_wait = false;

            MoteusAPI api1(Position_dev_name.toStdString(), position_Motor_id);
            // define a state object
            State curr_state1;

            // reset the state
            curr_state1.Reset();

            //read current position
            curr_state1.EN_Position();
            api1.ReadState(curr_state1);

            out << "Motor = " << position_Motor_id << " Ending position:\t" << curr_state1.position << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
    }
    else if (Step_Mode || (Rec_run_Enable && l_Cycle <= l_Cycle_Start_Stop))
    {
        if (TrajectoryComplete_pause)
        {
            // check TrajectoryComplete
            if (Check_TrajectoryComplete(l_dev_name,l_Motor_id))
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

                MoteusAPI api(l_dev_name.toStdString(), l_Motor_id);
                TrajectoryComplete_pause = false;
                if (!Check_Motor(l_dev_name,l_Motor_id))
                {
                   if (Motor_error)
                   {
                       // send one stop command
                       api.SendStopCommand();
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
                    if (!api.SendPositionCommand(list_Position[current_list_index],
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
                    if (!api.SendPositionCommand(list_Position[current_list_index],
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

void Motorworker::getFromMain(QString msg, QString dev_name, int Motor_id, double accel_limit, double position, double velocity_limit, double max_torque, double feedforward_torque, double kp_scale,
                               double kd_scale, double bounds_min, double bounds_max, double Cycle, double Delay) // slot implementation
{
    if (msg == "Save File")
    {
        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        string fileName = dev_name.toStdString();
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

        QString fileName = dev_name;
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
    else if (msg == "Check Device")
    {
        struct termios toptions;
        int fd;
        std::ostringstream out;
        out.str("");

        fd = open((dev_name.toStdString()).c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd == -1) {
            out << "Warning: Unable to open port, is the fdcanusb device plugged in?" << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
        else if (tcgetattr(fd, &toptions) < 0) {
            out << "Warning: Couldn't get term attributes" << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
        else close(fd);
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
    else if (msg == "Clear_Recorded")
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
    else if (msg == "Record Position")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Step_Mode = false;
        Position_wait = false;

        // define a state object
        State curr_state;
        std::ostringstream out;

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        api.ReadState(curr_state);

        double position_limited = curr_state.position;
        if (bounds_max != NAN && position_limited > bounds_max)
            position_limited = bounds_max;
        else if (bounds_min != NAN && position_limited < bounds_min)
            position_limited = bounds_min;

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

        out << "Current position: " << curr_state.position  << ", Velocity limit: " << velocity_limit << ", Accel limit: " << accel_limit << ", Motor: " << Motor_id << ", Delay: " << Delay << endl;
        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Update Dynamic")
    {
        l_velocity_limit = velocity_limit;
        l_accel_limit = accel_limit;
        l_max_torque = max_torque;
        l_feedforward_torque = feedforward_torque;
        l_kp_scale = kp_scale;
        l_kd_scale = kd_scale;
        Dynamic_Motor_id = Motor_id;
    }
    else if (msg == "Update Velocity")
    {
        Check_Velocity(dev_name, Motor_id);
    }
    else if (msg == "Run_Recorded")
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
        l_dev_name = dev_name;
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
    else if (msg == "Step_Recorded")
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
        l_dev_name = dev_name;
        l_Cycle = 0;
        // set l_Cycle_Delay and delay equal to cause first list load
        l_Cycle_Delay = 1;
        delay = 1;
       if (!list_Position.empty()) // check if list is empty
        {
           TrajectoryComplete_pause = false;
           Step_Mode = true;
        }
    }
    else if (msg == "get rotor_to_output_ratio")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;
        double value = std::numeric_limits<double>::quiet_NaN();

        std::ostringstream out;
        value = 12.34;
        // get rotor_to_output_ratio
            value = std::stod(
                controller.DiagnosticCommand("conf get motor_position.rotor_to_output_ratio",
                                             moteus::Controller::kExpectSingleLine));
       emit sendMsg("get gear ratio",Motor_id,value,0,0,0,0);

    }
    else if (msg == "get break voltage")
    {
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;
        double value = std::numeric_limits<double>::quiet_NaN();

        std::ostringstream out;

        // get min break voltage
        value = std::stod(
            controller.DiagnosticCommand("conf get servo.flux_brake_min_voltage",
                                         moteus::Controller::kExpectSingleLine));

        emit sendMsg("get Break Voltage",Motor_id,value,0,0,0,0);
    }
    else if (msg == "get Position Offset")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;
        double value = std::numeric_limits<double>::quiet_NaN();

        std::ostringstream out;

        // get min break voltage
        value = std::stod(
            controller.DiagnosticCommand("conf get motor_position.output.offset",
                                         moteus::Controller::kExpectSingleLine));
        emit sendMsg("get Position Offset",Motor_id,value,0,0,0,0);
    }
    else if (msg == "get motor limits")
    {
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        // get min limit command
        l_bounds_min[Motor_id-1] = std::stod(
            controller.DiagnosticCommand("conf get servopos.position_min",
                                         moteus::Controller::kExpectSingleLine));

        // get max limit command
        l_bounds_max[Motor_id-1] = std::stod(
            controller.DiagnosticCommand("conf get servopos.position_max",
                                         moteus::Controller::kExpectSingleLine));

        emit sendMsg("set motor limits",Motor_id,l_bounds_min[Motor_id-1],l_bounds_max[Motor_id-1],0,0,0);
    }
    else if (msg == "conf write")
    {
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        out.str("");

        // save conf
        controller.DiagnosticCommand("conf write");
        out.str("");
        out << "configuration write" << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "get PID")
    {
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

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

        emit sendMsg("get PID",Motor_id,value1,value2,value3,0,0);
    }
    else if (msg == "set PID")
    {
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;

        // set KP command
        out.str("");
        out << "conf set servo.pid_position.kp " << kp_scale << endl;
        controller.DiagnosticCommand(out.str());

        // set KD command
        out.str("");
        out << "conf set servo.pid_position.kd " << kd_scale << endl;
        controller.DiagnosticCommand(out.str());

        // set Ki command
        out.str("");
        out << "conf set servo.pid_position.ki " << feedforward_torque << endl;
        controller.DiagnosticCommand(out.str());

        out.str("");
        out << "Motor: " << Motor_id
            << "\tkp: " << kp_scale
            << "\tkd: " << kd_scale
            << "\t ki: " << feedforward_torque
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set rotor_to_output_ratio")
    {

        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        out.str("");

        // set command
        out << "conf set motor_position.rotor_to_output_ratio " << feedforward_torque << endl;
        controller.DiagnosticCommand(out.str());
            out.str("");
            out << "Motor: " << Motor_id
                << "\t Gear Ratio: " << feedforward_torque
                << endl;
            emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set break voltage")
    {

        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        out.str("");

        // set min limit command
        out << "conf set servo.flux_brake_min_voltage " << feedforward_torque << endl;
        controller.DiagnosticCommand(out.str());
        out.str("");
        out << "Motor: " << Motor_id
            << "\t Break Min Voltage: " << feedforward_torque
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set Position Offset")
    {
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        out.str("");

        // set command
        out << "conf set motor_position.output.offset 0 " << feedforward_torque << endl;
        controller.DiagnosticCommand(out.str());

        out.str("");
        out << "Motor: " << Motor_id
            << "\t motor_position.output.offset: " << feedforward_torque
            << endl;
        emit sendToMain(QString::fromStdString(out.str()));
    }
    else if (msg == "set motor limits")
    {

        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        std::ostringstream out;
        out.str("");

        // set min limit command
        out << "conf set servopos.position_min " << bounds_min << endl;
        controller.DiagnosticCommand(out.str());
        l_bounds_min[Motor_id-1] = bounds_min;
        // set max limit command
        out.str("");
        out << "conf set servopos.position_max " << bounds_max << endl;
        controller.DiagnosticCommand(out.str());
        l_bounds_max[Motor_id-1] = bounds_max;

        out.str("");
        out << "Motor: " << Motor_id
            << " limit min:\t" << l_bounds_min[Motor_id-1]
            << "\tlimit max:\t" << l_bounds_max[Motor_id-1] << endl;
        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Send Stop")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

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
        api.ReadState(curr_state);
        out << "Motor: " << Motor_id << " Stopped position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Set Output Nearest")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        moteus::Controller::Options options;
        options.id = Motor_id;
        moteus::Controller controller(options);

        Rec_run_Enable = false;
        Position_wait = false;

        moteus::OutputNearest::Command cmd;
        cmd.position = 0.0;

        // Command OutputNearest
        controller.SetOutputNearest(cmd);

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        api.ReadState(curr_state);
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
    else if (msg == "Send Start")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Position_wait = false;
        if (!Check_Motor(dev_name,Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               api.SendStopCommand();
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(l_dev_name,Motor_id);
           }
        }

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        api.ReadState(curr_state);

        std::ostringstream out;
        out.str("");
        out << "Current position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        if (!api.SendPositionCommand(curr_state.position,
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
        Position_dev_name = dev_name;
        Position_wait = true;
     }
    else if (msg == "Go To Rest Position")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(dev_name,Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               api.SendStopCommand();
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(l_dev_name,Motor_id);
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
        if (!api.SendPositionCommand(position_limited, // position
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
            Wait_TrajectoryComplete(dev_name,Motor_id);

            //read current position
            curr_state.Reset();
            curr_state.EN_Position();
            api.ReadState(curr_state);
            out << "Motor: " << Motor_id << " Position:\t" << curr_state.position << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

    }
    else if (msg == "Go To Position")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(dev_name,Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               api.SendStopCommand();
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(l_dev_name,Motor_id);
           }
        }

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        curr_state.EN_Position();
        api.ReadState(curr_state);

        std::ostringstream out;
        out.str("");
        out << "Motor = " << Motor_id << " Starting position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

        double position_limited = position;
        if (bounds_max != NAN && position_limited > bounds_max)
            position_limited = bounds_max;
        else if (bounds_min != NAN && position_limited < bounds_min)
            position_limited = bounds_min;

        if (!api.SendPositionCommand(position_limited,
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
        Position_dev_name = dev_name;
        Position_wait = true;

    }
    else if (msg == "Run Forever")
    {

        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Position_wait = false;

        if (!Check_Motor(dev_name,Motor_id))
        {
           if (Motor_error)
           {
               // send one stop command
               api.SendStopCommand();
           }
           else if (TrajectoryComplete_error)
           {
               // wait for TrajectoryComplete
               Wait_TrajectoryComplete(l_dev_name,Motor_id);
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
        api.ReadState(curr_state);

        out.str("");
        out << "Starting position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        float final_velocity = velocity_limit;
        if (position < 0.0)
            final_velocity = -velocity_limit;

        if (!api.SendPositionCommand(curr_state.position,
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
        Position_dev_name = dev_name;
        Position_wait = true;
    }
    else if (msg == "Read_Status")
    {

        MoteusAPI api(dev_name.toStdString(), Motor_id);

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
        api.ReadState(curr_state);

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
    else
    {
        emit sendToMain(msg);
    }
}
