#include "opencvworker.h"
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

OpenCvWorker::OpenCvWorker(QObject *parent) :
    QObject(parent)
{

}


OpenCvWorker::~OpenCvWorker()
{
}
void OpenCvWorker::Check_Motor_Error(QString dev_name,int Motor_id)
{
    MoteusAPI api(dev_name.toStdString(), Motor_id);

    // define a state object
    State curr_state;
    std::ostringstream out;
    out.str("");

    // reset the state
    curr_state.Reset();

    //read current position
    api.ReadState(curr_state.EN_Velocity());
    api.ReadState(curr_state.EN_Fault());
    api.ReadState(curr_state.EN_Mode());
    double Run_velocity = curr_state.velocity;
    unsigned int fault = static_cast<unsigned int>(curr_state.fault);
    unsigned int mode = static_cast<unsigned int>(curr_state.mode);

    if (fault !=0 || mode == 1 || mode == 11)
    {
        out.str("");
        out << "Fault detected, Issuing Motor stop first to clear error"  << endl;
        emit sendToMain(QString::fromStdString(out.str()));
        out.str("");
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
        // send one stop command
        api.SendStopCommand();
    }
    else if (Run_velocity > 0.2 || Run_velocity  < -0.2 )
    {
        out.str("");
        out << "Velocity detected, stopping Motor first. Velocity = "  << Run_velocity << endl;
        emit sendToMain(QString::fromStdString(out.str()));

        // send one stop command
        api.SendStopCommand();
        QThread::msleep(1500);  //Blocking delay 1500ms
    }
}

void OpenCvWorker::run_cycles()
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
                    api.SendPositionCommand(list_Position[current_list_index], list_Velocity[current_list_index],
                                            list_Max_torque[current_list_index],list_Feedforward_torque[current_list_index], list_Kp_scale[current_list_index], list_Kd_scale[current_list_index]);

                    out.str("");
                    out << "Position to: " << list_Position[current_list_index] << ", Velocity: " << list_Velocity[current_list_index] << ", Motor: " << list_Motor_id[current_list_index]
                           << ", Delay: " << list_Delay[current_list_index] << endl;
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

void OpenCvWorker::receiveSetup(int device, int mode)
{
    Rec_run_Enable = false;
    std::ostringstream out;
    out << " Run Cycles Stopped" << endl;
    emit sendToMain(QString::fromStdString(out.str()));
}

void OpenCvWorker::getFromMain(QString msg, QString dev_name, int Motor_id, double start_position, double stop_position, double velocity, double max_torque, double feedforward_torque, double kp_scale,
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
                     << "Velocity: " << list_Velocity[current_list_index] << endl
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
            list_Velocity.clear();
            list_Max_torque.clear();
            list_Feedforward_torque.clear();
            list_Kp_scale.clear();
            list_Kd_scale.clear();
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
                list_Velocity.push_back(val_double);

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
    else if (msg == "Clear_Recorded")
    {
        list_Position.clear();
        list_Motor_id.clear();
        list_Delay.clear();
        list_Velocity.clear();
        list_Max_torque.clear();
        list_Feedforward_torque.clear();
        list_Kp_scale.clear();
        list_Kd_scale.clear();

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
        list_Position.push_back(curr_state.position);
        list_Motor_id.push_back(Motor_id);
        list_Delay.push_back(Delay);
        list_Velocity.push_back(velocity);
        list_Max_torque.push_back(max_torque);
        list_Feedforward_torque.push_back(feedforward_torque);
        list_Kp_scale.push_back(kp_scale);
        list_Kd_scale.push_back(kd_scale);

        out << "Current position: " << curr_state.position  << ", Velocity: " << velocity << ", Motor: " << Motor_id << ", Delay: " << Delay << endl;
        emit sendToMain(QString::fromStdString(out.str()));

    }
    else if (msg == "Run_Recorded")
    {
        Rec_run_Enable = false;
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
        out << "Stopped position:\t" << curr_state.position << endl;
        emit sendToMain(QString::fromStdString(out.str()));

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
        api.SendPositionCommand(stop_position, velocity, max_torque,feedforward_torque, kp_scale, kd_scale);

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

        api.SendPositionCommand(NAN, velocity, max_torque,feedforward_torque, kp_scale, kd_scale);

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

        // print everyting
        out << "Position:\t\t" << curr_state.position << endl;
        out << "Velocity:\t\t" << curr_state.velocity << endl;
        out << "Torque:\t\t" << curr_state.torque << endl;
        out << "Q Current:\t\t" << curr_state.q_curr << endl;
        out << "D Current:\t\t" << curr_state.d_curr << endl;
        out << "Voltage:\t\t" << curr_state.voltage << endl;
        out << "Temperature:\t" << curr_state.temperature << endl;
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







