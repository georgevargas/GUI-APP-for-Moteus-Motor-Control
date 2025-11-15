#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#pragma once

#include <QObject>
#include <QMetaType>

#include <QMainWindow>
#include <QThread>
//#include <regex>
#include <iostream>
#include <string>
//#include <algorithm>
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QPixmap>
#include <stdlib.h> // required for system call
#include <QTimer>
#include "qcustomplot.h"

enum class Worker_Cmd : int {
    Record_XY,
    Go_To_Rest_Position,
    Send_Start,
    Go_To_Position,
    Run_Forever,
    Update_Dynamic,
    Record_Position,
    Run_Recorded,
    Step_Recorded,
    Open_File,
    Save_File,
    Update_Velocity,
    Set_Dynamic,
    Clear_Dynamic,
    Check_Device,
    Read_Status,
    Send_Stop,
    Clear_Recorded,
    set_motor_limits,
    set_rotor_to_output_ratio,
    set_break_voltage,
    set_PID,
    set_Position_Offset,
    conf_write,
    Get_Cur_XY,
    Set_Output_Nearest,
    get_motor_limits,
    get_PID,
    get_rotor_to_output_ratio,
    get_break_voltage,
    get_Position_Offset
};
enum class Worker_Stat : int {
    Check_Device,
    Get_Cur_XY,
    set_motor_limits,
    get_PID,
    get_gear_ratio,
    get_Position_Offset,
    get_Break_Voltage,
    get_velocity
};
Q_DECLARE_METATYPE(Worker_Cmd);
Q_DECLARE_METATYPE(Worker_Stat);
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
    QCustomPlot * m_CustomPlot;
    QThread *thread;
    QThread *thread1;
    double accel_limit = 0.75;
    double position = 1;
    double velocity_limit = 0.224;
    double max_torque = 20;
    double feedforward_torque = 0.1;
    double kp_scale = 1;
    double kd_scale = 1;
    double Cycle_Start_Stop = 100;
    double Cycle_Delay = 1.1;
    bool   Dynamic = false;
    bool   Device_enable = false;
    bool   Enable_startup_nearest_commands = false;
    bool   Enable_plot_position = true;
    bool   Enable_plot_velocity = false;
    bool   Enable_plot_torque = false;
    bool   Enable_plot_temperature = false;
    bool   Enable_plot_q_phase_current = false;
    double time = 0;
    // Data buffers
    QVector<qreal> m_YData;
    QVector<qreal> m_XData;
    QVector<qreal> m_YData1;
    QVector<qreal> m_XData1;
    QVector<qreal> m_YData2;
    QVector<qreal> m_XData2;
    QVector<qreal> m_YData3;
    QVector<qreal> m_XData3;
    QVector<qreal> m_YData4;
    QVector<qreal> m_XData4;

    // This object will hold the current value as a text
    // that will appear at the extreme right of the plot,
    QCPItemText *m_ValueIndex;

    QTimer * myTimer;
    QString dev_name = "/dev/ttyACM0";
    int moteus_id = 1;
    int Number_of_Motors = 1;
    double Motor_rest_position[10] ={0.217,0.140,0.2085,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double bounds_min[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double bounds_max[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double kp[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double kd[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double ki[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Gear_Ratio[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Break_Voltage[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Velocity[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Position[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Torque[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Temperature[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double Q_Phase_Current[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double position_offset[10] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double nearest_offset[10] ={0.149242,0.148727,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double position_X = 0;
    double position_Y = 0;
    void setup();
    void registerMyTypes();
    void Init_Motor();

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

signals:
    void sendSetup();
    void sendToWorker_position_commands(Worker_Cmd msg, int Motor_id,double accel_limit,double position,double velocity_limit,double max_torque,double feedforward_torque,double kp_scale,
         double kd_scale,double bounds_min,double bounds_max,double Cycle,double Delay,double position_X,double position_Y);
    void sendToWorker_file_commands(Worker_Cmd msg, QString file_name);
    void sendToWorker_motor_commands(Worker_Cmd msg, int Motor_id);
    void sendToWorker_diagnostic_write_commands(Worker_Cmd msg, int Motor_id, double Value1, double Value2, double Value3);
    void sendToWorker_diagnostic_read_commands(Worker_Cmd msg, int Motor_id);

public slots:
    void receiveMsg(Worker_Stat  msg, int Motor_id,double Value1, double Value2, double Value3,double value4,double value5,double Value6,double Value7);
    void getFromWorker(QString);
    void updateDiagram();

private slots:
    void on_btnStop_Motor_clicked();
    void on_btnRun_Velocity_clicked();
    void on_btnRun_Position_clicked();
    void on_btnStop_Cycle_clicked();
    void on_btnRec_positions_clicked();
    void on_btnRun_Recorded_clicked();
    void on_btnStep_Recorded_clicked();
    void on_btnClear_Recorded_clicked();
    void on_btnRead_Status_clicked();
    void on_comboBox_currentIndexChanged(int index);
    void on_btnPosition_Offset_clicked();
    void on_btnRec_Gear_Ratio_clicked();
    void on_btnRec_update_limits_clicked();
    void on_btnRun_update_KP_clicked();
    void on_btnConf_Write_clicked();
    void on_btnConf_Read_clicked();
    void on_btnSetNearest_clicked();
    void on_btnRec_Break_voltage_clicked();
    void on_btnGo_To_Rest_Position_clicked();
    void on_btnStart_Motor_clicked();

    void on_Slider_Position_valueChanged(int value);
    void on_Slider_Velocity_Limit_valueChanged(int value);
    void on_Slider_Accel_Limit_valueChanged(int value);
    void on_Slider_Max_Torque_valueChanged(int value);
    void on_Slider_Feedforward_valueChanged(int value);
    void on_Slider_KP_Scale_valueChanged(int value);
    void on_Slider_KD_Scale_valueChanged(int value);
    void on_Slider_Cycle_Delay_valueChanged(int value);
    void on_Slider_Cycle_Start_Stop_valueChanged(int value);
    void on_Slider_Position_X_valueChanged(int value);
    void on_Slider_Position_Y_valueChanged(int value);
    void on_Slider_KP_valueChanged(int value);
    void on_Slider_KD_valueChanged(int value);
    void on_Slider_KI_valueChanged(int value);
    void on_Slider_Limit_Min_valueChanged(int value);
    void on_Slider_Limit_Max_valueChanged(int value);
    void on_Slider_Gear_Ratio_valueChanged(int value);
    void on_Slider_Break_voltage_valueChanged(int value);
    void on_Slider_Position_Offset_valueChanged(int value);

    void on_Counter_Accel_Limit_valueChanged(double value);
    void on_Counter_Position_valueChanged(double value);
    void on_Counter_Velocity_Limit_valueChanged(double value);
    void on_Counter_Max_Torque_valueChanged(double value);
    void on_Counter_Feedforward_valueChanged(double value);
    void on_Counter_KP_Scale_valueChanged(double value);
    void on_Counter_KD_Scale_valueChanged(double value);
    void on_Counter_Cycle_Start_Stop_valueChanged(double value);
    void on_Counter_Cycle_Delay_valueChanged(double value);
    void on_Counter_Limit_Min_valueChanged(double value);
    void on_Counter_Limit_Max_valueChanged(double value);
    void on_Counter_KP_valueChanged(double value);
    void on_Counter_KD_valueChanged(double value);
    void on_Counter_KI_valueChanged(double value);
    void on_Counter_Gear_Ratio_valueChanged(double value);
    void on_Counter_Break_voltage_valueChanged(double value);
    void on_Counter_Position_Offset_valueChanged(double value);
    void on_Counter_Position_X_valueChanged(double value);
    void on_Counter_Position_Y_valueChanged(double value);

    void on_checkBox_Dymamic_clicked();

    void on_actionExit_triggered();
    void on_actionAbout_triggered();
    void on_actionHelp_triggered();
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionPosition_changed();
    void on_actionVelocity_changed();
    void on_actionTorque_changed();
    void on_actionTemperature_changed();
    void on_actionQ_Phase_Current_changed();
    void on_btnRun_Cur_X_Y_pos_clicked();
    void on_btn_record_X_Y_clicked();
};

#endif // MAINWINDOW_H
