#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "opencvworker.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <qtimer.h>
#include <QtCore>
#include <cstring>
#include <QtWidgets>
#include <QPrinter>
#include <QPrintDialog>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QInputDialog>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setup();

    Init_Motor();
}

MainWindow::~MainWindow()
{
    thread->quit();
    while(!thread->isFinished());

    delete thread;
    delete ui;
}

void MainWindow::setup()
{

    thread = new QThread();
    OpenCvWorker *worker = new OpenCvWorker();
    QTimer *workerTrigger = new QTimer();
    workerTrigger->setInterval(10);


    connect(workerTrigger, SIGNAL(timeout()), worker, SLOT(run_cycles()));
    connect(this, SIGNAL(sendSetup(int,int)), worker, SLOT(receiveSetup(int,int)));
    connect(worker, SIGNAL(sendMsg(QString)), this, SLOT(receiveMsg(QString)));
    connect(this,SIGNAL(sendToWorker(QString,QString,int,double,double,double,double,double,double,double,double,double,double,double)),worker,SLOT(getFromMain(QString,QString,int,double,
             double,double,double,double,double,double,double,double,double,double)));
    connect(worker,SIGNAL(sendToMain(QString)),this,SLOT(getFromWorker(QString)));

    worker->moveToThread(thread);

    thread->start();

    workerTrigger->start();
    workerTrigger->moveToThread(thread);

}
void MainWindow::getFromWorker(QString msg) //slot implementation
{
    MainWindow::ui->txtXYRadius->appendPlainText(msg);
}

void MainWindow::receiveMsg(QString msg)
{
    MainWindow::ui->txtXYRadius->appendPlainText(msg);
}

void MainWindow:: Init_Motor()
{
        ui->Counter_Start_Position->setValue(start_position);
        ui->Counter_Stop_Position->setValue(stop_position);
        ui->Counter_Velocity->setValue(velocity);
        ui->Counter_Max_Torque->setValue(max_torque);
        ui->Counter_Feedforward->setValue(feedforward_torque);
        ui->Counter_KD_Scale->setValue(kd_scale);
        ui->Counter_KP_Scale->setValue(kp_scale);
        ui->Counter_Cycle_Start_Stop->setValue(Cycle_Start_Stop);
        ui->Counter_Cycle_Delay->setValue(Cycle_Delay);
        ui->Slider_Start_Position->setValue(start_position);
        ui->Slider_Stop_Position->setValue(stop_position);
        ui->Slider_Velocity->setValue(velocity);
        ui->Slider_Max_Torque->setValue(max_torque);
        ui->Slider_Feedforward->setValue(feedforward_torque);
        ui->Slider_KD_Scale->setValue(kd_scale);
        ui->Slider_KP_Scale->setLowerBound(kp_scale);
        ui->Slider_Cycle_Start_Stop->setValue(Cycle_Start_Stop);
        ui->Slider_Cycle_Delay->setValue(Cycle_Delay);

        update();

        emit sendToWorker("Check Device",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);

}

void MainWindow::on_btnRead_Status_clicked()
{
    emit sendToWorker("Read_Status",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnStop_Motor_clicked()
{
    if( thread->isRunning() )
    {
//                thread->requestInterruption();
    }
    emit sendToWorker("Send Stop",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnRun_Position_clicked()
{
    emit sendToWorker("Go To Position",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnRun_Velocity_clicked()
{
    emit sendToWorker("Run Forever",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);

}

void MainWindow::on_btnStop_Cycle_clicked()
{
    emit sendSetup(0, 0);

}
void MainWindow::on_Slider_Start_Position_valueChanged(double value)
{
    start_position = value;
    ui->Counter_Start_Position->setValue(value);

}

void MainWindow::on_Slider_Stop_Position_valueChanged(double value)
{
    stop_position = value;
    ui->Counter_Stop_Position->setValue(value);
}

void MainWindow::on_Slider_Velocity_valueChanged(double value)
{
    velocity = value;
    ui->Counter_Velocity->setValue(value);
}

void MainWindow::on_Slider_Max_Torque_valueChanged(double value)
{
    max_torque = value;
    ui->Counter_Max_Torque->setValue(value);
}

void MainWindow::on_Slider_Feedforward_valueChanged(double value)
{
    feedforward_torque = value;
    ui->Counter_Feedforward->setValue(value);
}

void MainWindow::on_Slider_KP_Scale_valueChanged(double value)
{
    kp_scale = value;
    ui->Counter_KP_Scale->setValue(value);
}

void MainWindow::on_Slider_KD_Scale_valueChanged(double value)
{
    kd_scale = value;
    ui->Counter_KD_Scale->setValue(value);
}

void MainWindow::on_Counter_Start_Position_valueChanged(double value)
{
    start_position = value;
    ui->Slider_Start_Position->setValue(value);

}

void MainWindow::on_Counter_Stop_Position_valueChanged(double value)
{
    stop_position = value;
    ui->Slider_Stop_Position->setValue(value);
}

void MainWindow::on_Counter_Velocity_valueChanged(double value)
{
    velocity = value;
    ui->Slider_Velocity->setValue(value);

}

void MainWindow::on_Counter_Max_Torque_valueChanged(double value)
{
    max_torque = value;
    ui->Slider_Max_Torque->setValue(value);
}

void MainWindow::on_Counter_Feedforward_valueChanged(double value)
{
    feedforward_torque = value;
    ui->Slider_Feedforward->setValue(value);
}

void MainWindow::on_Counter_KP_Scale_valueChanged(double value)
{
    kp_scale = value;
    ui->Slider_KP_Scale->setValue(value);
}

void MainWindow::on_Counter_KD_Scale_valueChanged(double value)
{
    kd_scale = value;
    ui->Slider_KD_Scale->setValue(value);
}

void MainWindow::on_actionExit_triggered()
{
   QApplication::quit();
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::information(this,"About",
     "Motor Controller\n\n"
     "Rev 1.0"
     );
}

void MainWindow::on_actionHelp_triggered()
{
 QMessageBox::information(this,"Help",
     "Motor controller\n\n"
     );
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    moteus_id  = index + 1;
}

void MainWindow::on_Counter_Cycle_Start_Stop_valueChanged(double value)
{
    Cycle_Start_Stop = value;
    ui->Slider_Cycle_Start_Stop->setValue(value);
}

void MainWindow::on_Slider_Cycle_Start_Stop_valueChanged(double value)
{
    Cycle_Start_Stop = value;
    ui->Counter_Cycle_Start_Stop->setValue(value);
}

void MainWindow::on_Counter_Cycle_Delay_valueChanged(double value)
{
    Cycle_Delay = value;
    ui->Slider_Cycle_Delay->setValue(value);
}

void MainWindow::on_Slider_Cycle_Delay_valueChanged(double value)
{
    Cycle_Delay = value;
    ui->Counter_Cycle_Delay->setValue(value);
}


void MainWindow::on_btnRec_positions_clicked()
{
    emit sendToWorker("Record Position",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnRun_Recorded_clicked()
{
    emit sendToWorker("Run_Recorded",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnClear_Recorded_clicked()
{
    emit sendToWorker("Clear_Recorded",QString::fromStdString(dev_name),moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);

}

void MainWindow::on_actionOpen_triggered()
{
    QFileDialog fileDialog(this, tr("Read Record List File"));
    const QStringList filters({"txt" });
    fileDialog.setNameFilters(filters);
    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog.setMimeTypeFilters(filters);
    fileDialog.setDefaultSuffix("txt");
    if (fileDialog.exec() == QDialog::Accepted)
    {
        QString fileName = fileDialog.selectedFiles().first();

        emit sendToWorker("Open File",fileName,moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);
        statusBar()->showMessage(tr("Read \"%1\"")
                                 .arg(QDir::toNativeSeparators(fileName)));
    }

}

void MainWindow::on_actionSave_triggered()
{
    QFileDialog fileDialog(this, tr("Save Record List File"));
    const QStringList filters({"txt" });
    fileDialog.setNameFilters(filters);
    fileDialog.setAcceptMode(QFileDialog::AcceptSave);
    fileDialog.setMimeTypeFilters(filters);
    fileDialog.setDefaultSuffix("txt");
    if (fileDialog.exec() == QDialog::Accepted)
    {
        QString fileName = fileDialog.selectedFiles().first();
        emit sendToWorker("Save File",fileName,moteus_id,start_position,stop_position,velocity,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min,bounds_max,Cycle_Start_Stop,Cycle_Delay);

        statusBar()->showMessage(tr("Saved \"%1\"")
                                 .arg(QDir::toNativeSeparators(fileName)));
    }
}