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

using namespace mjbots;
using namespace moteus;

Motorworker::Motorworker(QObject *parent) :
    QObject(parent)
{

}


Motorworker::~Motorworker()
{
}
void Motorworker::Check_Motor_Error(QString dev_name,int Motor_id)
{
    MoteusAPI api(dev_name.toStdString(), Motor_id);

    // define a state object
    State curr_state;
    std::ostringstream out;
    out.str("");

    // reset the state
    curr_state.Reset();

    //read current position
    api.ReadState(curr_state.EN_Fault());
    api.ReadState(curr_state.EN_Mode());
    api.ReadState(curr_state.EN_TrajectoryComplete());
    Fault fault = static_cast<Fault>(curr_state.fault);
    Mode mode = static_cast<Mode>(curr_state.mode);
    bool TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);

    if (fault != Fault::kNoFault || mode == Mode::kFault || mode == Mode::kPositionTimeout)
    {
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
            case Mode::kStayWithinBounds:
                out << "Mode:\t\t" << curr_state.mode << " = Stay Within Bounds" << endl;
                break;
            case Mode::kMeasureInductance:
                out << "Mode:\t\t" << curr_state.mode << " = Measure Inductance" << endl;
                break;

            default:
                out << "Mode:\t\t" << curr_state.mode << " = unknown mode" << endl;
        }
        emit sendToMain(QString::fromStdString(out.str()));
        // send one stop command
        api.SendStopCommand();
    }
    else if (fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
    {
        // wait until Trajectory Complete is true and check for fault

        out << "Trajectory is not Complete, waiting for complete "  << " , Motor = " << list_Motor_id[current_list_index] << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        int count = Trajectory_Timeout_1; // 20 sec
        while (count > 0 && fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
        {
            QThread::msleep(100);  //Blocking delay 100ms
            api.ReadState(curr_state.EN_Fault());
            api.ReadState(curr_state.EN_Mode());
            api.ReadState(curr_state.EN_TrajectoryComplete());
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
            out.str("");
            out << "Fault detected, while waiting for Trajectory Complete"  << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
    }
}

void Motorworker::run_cycles()
{
    if (Rec_run_Enable && l_Cycle <= l_Cycle_Start_Stop)
    {
        if (delay >= l_Cycle_Delay)
        {
            if (current_list_index < (int)list_Position.size())
            {
                delay = 0;
                l_Cycle_Delay = list_Delay[current_list_index];
                l_Motor_id = list_Motor_id[current_list_index];

                MoteusAPI api(l_dev_name.toStdString(), l_Motor_id);

                Check_Motor_Error(l_dev_name,l_Motor_id);


                std::ostringstream out;

                if (current_list_index < (int)list_Position.size())
                {

                    if (Dynamic)
                    {
                        api.SendPositionCommand(list_Position[current_list_index],
                                                l_velocity_limit,
                                                l_accel_limit,
                                                l_max_torque,
                                                l_feedforward_torque,
                                                l_kp_scale,
                                                l_kd_scale,
                                                0.0 // end velocity
                                                );
                    }
                    else
                    {
                        api.SendPositionCommand(list_Position[current_list_index],
                                                list_velocity_limit[current_list_index],
                                                list_accel_limit[current_list_index],
                                                list_Max_torque[current_list_index],
                                                list_Feedforward_torque[current_list_index],
                                                list_Kp_scale[current_list_index],
                                                list_Kd_scale[current_list_index],
                                                0.0 // end velocity
                                                );
                    }

                    out.str("");
                    if (Dynamic)
                    {
                        out << "Position to: " << list_Position[current_list_index]
                                << ", Velocity: " << l_velocity_limit
                                << ", Accel: " << l_accel_limit
                                << ", Motor: " << list_Motor_id[current_list_index]
                                << ", Max torque: " << l_max_torque
                                << ", Feedforward torque: " << l_feedforward_torque
                                << ", KP scale: " << l_kp_scale
                                << ", KD scale: " << l_kd_scale
                                << endl;
                    }
                    else
                    {
                        out << "Position to: " << list_Position[current_list_index]
                            << ", Velocity: " << list_velocity_limit[current_list_index]
                            << ", Accel: " << list_accel_limit[current_list_index]
                            << ", Motor: " << list_Motor_id[current_list_index]
                            << ", Delay: " << list_Delay[current_list_index]
                            << endl;
                    }
                    emit sendToMain(QString::fromStdString(out.str()));

                    current_list_index++ ;
                    if (current_list_index >= (int)list_Position.size())
                    {
                        current_list_index = 0;
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
    Rec_run_Enable = false;
    std::ostringstream out;
    out << " Run Cycles Stopped" << endl;
    emit sendToMain(QString::fromStdString(out.str()));
}

void Motorworker::getFromMain(QString msg, QString dev_name, int Motor_id, double accel_limit, double position, double velocity_limit, double max_torque, double feedforward_torque, double kp_scale,
                               double kd_scale, double bounds_min, double bounds_max, double Cycle, double Delay) // slot implementation
{
    if (msg == "Save File")
    {
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

    }
    else if (msg == "Open File")
    {
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
    }
    else if (msg == "Clear Dynamic")
    {
        Dynamic = false;
    }
    else if (msg == "Clear_Recorded")
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

        emit sendToMain(msg);
    }
    else if (msg == "Record Position")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        // define a state object
        State curr_state;
        std::ostringstream out;

        // reset the state
        curr_state.Reset();

        //read current position
        api.ReadState(curr_state.EN_Position());

        double position_limited = curr_state.position;
        if (curr_state.position > bounds_max)
            position_limited = bounds_max;
        else if (curr_state.position < bounds_min)
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
    }
    else if (msg == "Run_Recorded")
    {
        Rec_run_Enable = false;

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
        l_Cycle_Delay = 1;
        delay = 1;
       if (!list_Position.empty()) // check if list is empty
        {
           Rec_run_Enable = true;
        }
    }
    else if (msg == "Send Stop")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        // send one stop command
        api.SendStopCommand();

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        QThread::msleep(1500);  //Blocking delay 1500ms

        // reset the state
        curr_state.Reset();

        //read current position
        api.ReadState(curr_state.EN_Position());
        out << "Motor: " << Motor_id << " Stopped position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Set Output Nearest")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        // send one stop command
        api.SendSetOutputNearest(0.0);

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();

        //read current position
        api.ReadState(curr_state.EN_Position());
        out << "Motor: " << Motor_id << " Position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Send Start")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Check_Motor_Error(dev_name,Motor_id);

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        api.ReadState(curr_state.EN_Position());

        std::ostringstream out;
        out.str("");
        out << "Current position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        api.SendPositionCommand(curr_state.position,
                                velocity_limit,
                                accel_limit,
                                max_torque,
                                feedforward_torque,
                                kp_scale,
                                kd_scale,
                                0.0 // end velocity
                                );
    }
    else if (msg == "Go To Rest Position")
    {
        bool success = false;
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Check_Motor_Error(dev_name,Motor_id);

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        std::ostringstream out;
        out.str("");
        double position_limited = position;
        if (position > bounds_max)
            position_limited = bounds_max;
        else if (position < bounds_min)
            position_limited = bounds_min;
        success = api.SendPositionCommand(position_limited, // position
                                1.0, // velocity_limit
                                1.0, // accel_limit
                                1.0, // max_torque
                                0.1, // feedforward_torque
                                1.0, // kp_scale
                                1.0, // kd_scale
                                0.0  // end velocity
                                );
        if (!success)
        {
            out << "error on posiion command " << Motor_id << endl;
            emit sendToMain(QString::fromStdString(out.str()));

        }
        api.ReadState(curr_state.EN_Fault());
        api.ReadState(curr_state.EN_Mode());
        api.ReadState(curr_state.EN_TrajectoryComplete());
        Fault fault = static_cast<Fault>(curr_state.fault);
        Mode mode = static_cast<Mode>(curr_state.mode);
        bool TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);
        int count = Trajectory_Timeout_2; // 10 sec
        if (fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
        {
            while (count > 0 && fault == Fault::kNoFault && mode == Mode::kPosition && !TrajectoryComplete)
            {
                QThread::msleep(100);  //Blocking delay 100ms
                //read current position
                api.ReadState(curr_state.EN_Fault());
                api.ReadState(curr_state.EN_Mode());
                api.ReadState(curr_state.EN_TrajectoryComplete());
                fault = static_cast<Fault>(curr_state.fault);
                mode = static_cast<Mode>(curr_state.mode);
                TrajectoryComplete = static_cast<bool>(curr_state.TrajectoryComplete);
                count--;
            }
        }
        if (!(count > 0) )
        {
            out.str("");
            out << "Timeout waiting for Trajectory Complete, Motor: " << Motor_id<< endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
        else if (fault != Fault::kNoFault || mode == Mode::kFault || mode == Mode::kPositionTimeout)
        {
            out.str("");
            out << "Fault detected, while waiting for Trajectory Complete"  << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }
        else
        {
            //read current position
            curr_state.Reset();
            api.ReadState(curr_state.EN_Position());
            out << "Motor: " << Motor_id << " Position:\t" << curr_state.position << endl;
            emit sendToMain(QString::fromStdString(out.str()));
        }

    }
    else if (msg == "Go To Position")
    {
        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Check_Motor_Error(dev_name,Motor_id);

        // define a state object
        State curr_state;

        // reset the state
        curr_state.Reset();

        //read current position
        api.ReadState(curr_state.EN_Position());

        std::ostringstream out;
        out.str("");
        out << "Starting position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

        double position_limited = position;
        if (position > bounds_max)
            position_limited = bounds_max;
        else if (position < bounds_min)
            position_limited = bounds_min;

        api.SendPositionCommand(position_limited,
                                velocity_limit,
                                accel_limit,
                                max_torque,
                                feedforward_torque,
                                kp_scale,
                                kd_scale,
                                0.0 // end velocity
                                );

    }
    else if (msg == "Run Forever")
    {

        MoteusAPI api(dev_name.toStdString(), Motor_id);
        Rec_run_Enable = false;
        Check_Motor_Error(dev_name,Motor_id);

        // define a state object
        State curr_state;
        std::ostringstream out;
        out.str("");

        // reset the state
        curr_state.Reset();

        //read current position
        api.ReadState(curr_state.EN_Position());

        out.str("");
        out << "Starting position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        float final_velocity = velocity_limit;
        if (position < 0.0)
            final_velocity = -velocity_limit;

        api.SendPositionCommand(curr_state.position,
                                velocity_limit,
                                accel_limit,
                                max_torque,
                                feedforward_torque,
                                kp_scale,
                                kd_scale,
                                final_velocity // end velocity
                                );

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

        api.ReadState(curr_state.EN_Position());
        api.ReadState(curr_state.EN_Velocity());
        api.ReadState(curr_state.EN_Torque());
        api.ReadState(curr_state.EN_QCurr());
        api.ReadState(curr_state.EN_DCurr());
        api.ReadState(curr_state.EN_Voltage());
        api.ReadState(curr_state.EN_Temp());
        api.ReadState(curr_state.EN_Fault());
        api.ReadState(curr_state.EN_Mode());
        api.ReadState(curr_state.EN_TrajectoryComplete());
        api.ReadState(curr_state.EN_Rezerostate());
        // print everyting
        out << "Position:\t\t" << curr_state.position << endl;
        out << "Velocity:\t\t" << curr_state.velocity << endl;
        out << "Torque:\t\t" << curr_state.torque << endl;
        out << "Q Current:\t\t" << curr_state.q_curr << endl;
        out << "D Current:\t\t" << curr_state.d_curr << endl;
        out << "Voltage:\t\t" << curr_state.voltage << endl;
        out << "Temperature:\t" << curr_state.temperature << endl;
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
        switch (fault)  {
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
        switch (mode)  {
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