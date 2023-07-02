#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Motorworker.h"
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
    //Setup my diagram
    ui->myPlot->addGraph();
    ui->myPlot->xAxis->setLabel("Time");
    ui->myPlot->xAxis->setRange(0,2);
    ui->myPlot->yAxis->setLabel("Position");
    ui->myPlot->yAxis->setRange(-.4,-.1);

    //init time
    time = 0.0;

    //init my qtimer
    myTimer = new QTimer(this);
    connect(myTimer, &QTimer::timeout, this, &MainWindow::updateDiagram);
    myTimer->setInterval(50);

    thread = new QThread();
    Motorworker *worker = new Motorworker();
    QTimer *workerTrigger = new QTimer();
    workerTrigger->setInterval(10);


    connect(workerTrigger, SIGNAL(timeout()), worker, SLOT(run_cycles()));
    connect(this, SIGNAL(sendSetup()), worker, SLOT(receiveSetup()));
    connect(worker, SIGNAL(sendMsg(QString,int,double,double,double)), this, SLOT(receiveMsg(QString,int,double,double,double)));
    connect(this,SIGNAL(sendToWorker(QString,QString,int,double,double,double,double,double,double,double,double,double,double,double)),worker,SLOT(getFromMain(QString,QString,int,double,
             double,double,double,double,double,double,double,double,double,double)));
    connect(worker,SIGNAL(sendToMain(QString)),this,SLOT(getFromWorker(QString)));

    worker->moveToThread(thread);

    thread->start();

    workerTrigger->start();
    workerTrigger->moveToThread(thread);
    myTimer->start();

}
void MainWindow::getFromWorker(QString msg) //slot implementation
{
    MainWindow::ui->txtXYRadius->appendPlainText(msg);
}

void MainWindow::updateDiagram()
{
    time += 0.05;
    emit sendToWorker("Update Velocity",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::receiveMsg(QString msg, int Motor_id, double Value1, double Value2 , double Value3)
{
    if (msg == "set motor limits")
    {
        std::ostringstream out;

        bounds_min[Motor_id-1] =  Value1;
        bounds_max[Motor_id-1] =  Value2;

        out.str("");
        out << "Motor: " << Motor_id << " limit min:\t" << bounds_min[Motor_id-1]
        << "\tlimit max:\t" << bounds_max[Motor_id-1] << endl;
        ui->Slider_Limit_Min->setValue(bounds_min[moteus_id -1]);
        ui->Counter_Limit_Min->setValue(bounds_min[moteus_id -1]);
        ui->Slider_Limit_Max->setValue(bounds_max[moteus_id -1]);
        ui->Counter_Limit_Max->setValue(bounds_max[moteus_id -1]);

        MainWindow::ui->txtXYRadius->appendPlainText(QString::fromStdString(out.str()));
    }
    else if (msg == "get PID")
    {
        std::ostringstream out;

        kp[Motor_id-1] =  Value1;
        kd[Motor_id-1] =  Value2;
        ki[Motor_id-1] =  Value3;

        ui->Slider_KP->setValue(kp[moteus_id -1]);
        ui->Counter_KP->setValue(kp[moteus_id -1]);
        ui->Slider_KD->setValue(kd[moteus_id -1]);
        ui->Counter_KD->setValue(kd[moteus_id -1]);
        ui->Slider_KI->setValue(ki[moteus_id -1]);
        ui->Counter_KI->setValue(ki[moteus_id -1]);

        out.str("");
        out << "Motor: " << Motor_id
            << "\tkp: " << kp[Motor_id-1]
            << "\tkd: " << kd[Motor_id-1]
            << "\t ki: " << ki[Motor_id-1]
            << endl;
        MainWindow::ui->txtXYRadius->appendPlainText(QString::fromStdString(out.str()));
    }
    else if (msg == "get gear ratio")
    {
        std::ostringstream out;

        Gear_Ratio[Motor_id-1] =  Value1;

        ui->Slider_Gear_Ratio->setValue(Gear_Ratio[moteus_id -1]);
        ui->Counter_Gear_Ratio->setValue(Gear_Ratio[moteus_id -1]);

        out.str("");
        out << "Motor: " << Motor_id
            << "\tGear Ratio: " << Gear_Ratio[Motor_id-1]
            << endl;
        MainWindow::ui->txtXYRadius->appendPlainText(QString::fromStdString(out.str()));
    }
    else if (msg == "get Break Voltage")
    {
        std::ostringstream out;

        Break_Voltage[Motor_id-1] =  Value1;

        ui->Slider_Break_voltage->setValue(Break_Voltage[moteus_id -1]);
        ui->Counter_Break_voltage->setValue(Break_Voltage[moteus_id -1]);

        out.str("");
        out << "Motor: " << Motor_id
            << "\tBreak Voltage: " << Break_Voltage[Motor_id-1]
            << endl;
        MainWindow::ui->txtXYRadius->appendPlainText(QString::fromStdString(out.str()));
    }
    else if (msg == "get velocity")
    {
        Velocity[Motor_id-1] =  Value1;

        // Add the time the x data buffer
        m_XData.append( time );
        m_YData.append( Velocity[moteus_id-1] );
        if( m_XData.size() > 100 )
        {
            m_XData.remove( 0 );
            m_YData.remove( 0 );
        }

        ui->myPlot->graph(0)->setData( m_XData , m_YData );

        // Set the range of the vertical and horizontal axis of the plot ( not the graph )
        // so all the data will be centered. first we get the min and max of the x and y data
        QVector<double>::iterator xMaxIt = std::max_element( m_XData.begin() , m_XData.end() );
        QVector<double>::iterator xMinIt = std::min_element( m_XData.begin() , m_XData.end() );
        QVector<double>::iterator yMaxIt = std::max_element( m_YData.begin() , m_YData.end() );
        QVector<double>::iterator yMinIt = std::min_element( m_YData.begin() , m_YData.end() );

        qreal yPlotMin = *yMinIt;
        qreal yPlotMax = *yMaxIt;

        qreal xPlotMin = *xMinIt;
        qreal xPlotMax = *xMaxIt;

        // The yOffset just to make sure that the graph won't take the whole
        // space in the plot widget, and to keep a margin at the top, the same goes for xOffset
        qreal yOffset = 0.05 * ( yPlotMax - yPlotMin ) ;
        qreal xOffset = 0.05 *( xPlotMax - xPlotMin );

        if ( (time - oldtime) > 1.0)
        {
            oldtime = time;
            ui->myPlot->xAxis->setRange( xPlotMin , xPlotMax + xOffset );
            ui->myPlot->yAxis->setRange(yPlotMin - yOffset, yPlotMax + yOffset);
        }
        ui->myPlot->replot();
    }
}

void MainWindow:: Init_Motor()
{
        ui->Counter_Accel_Limit->setValue(accel_limit);
        ui->Counter_Position->setValue(position);
        ui->Counter_Velocity_Limit->setValue(velocity_limit);
        ui->Counter_Max_Torque->setValue(max_torque);
        ui->Counter_Feedforward->setValue(feedforward_torque);
        ui->Counter_KD_Scale->setValue(kd_scale);
        ui->Counter_KP_Scale->setValue(kp_scale);
        ui->Counter_Cycle_Start_Stop->setValue(Cycle_Start_Stop);
        ui->Counter_Cycle_Delay->setValue(Cycle_Delay);
        ui->Slider_Accel_Limit->setValue(accel_limit);
        ui->Slider_Position->setValue(position);
        ui->Slider_Velocity_Limit->setValue(velocity_limit);
        ui->Slider_Max_Torque->setValue(max_torque);
        ui->Slider_Feedforward->setValue(feedforward_torque);
        ui->Slider_KD_Scale->setValue(kd_scale);
        ui->Slider_KP_Scale->setLowerBound(kp_scale);
        ui->Slider_Cycle_Start_Stop->setValue(Cycle_Start_Stop);
        ui->Slider_Cycle_Delay->setValue(Cycle_Delay);

        update();

        emit sendToWorker("Check Device",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);

        if (Enable_startup_nearest_commands)
        {
            for (int i = 1; i <= Number_of_Motors; i++)
            {
                emit sendToWorker("Set Output Nearest",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                                  kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
            }
        }

        for (int i = 1; i <= Number_of_Motors; i++)
        {
            emit sendToWorker("get motor limits",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                              kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
        }
        for (int i = 1; i <= Number_of_Motors; i++)
        {
            emit sendToWorker("get PID",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                              kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
        }
        for (int i = 1; i <= Number_of_Motors; i++)
        {
            emit sendToWorker("get rotor_to_output_ratio",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                              kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
        }
        for (int i = 1; i <= Number_of_Motors; i++)
        {
            emit sendToWorker("get break voltage",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                              kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
        }
}

void MainWindow::on_btnRead_Status_clicked()
{
    emit sendToWorker("Read_Status",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnSetNearest_clicked()
{
    for (int i = 1; i <= Number_of_Motors; i++)
    {
        emit sendToWorker("Set Output Nearest",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_btnStop_Motor_clicked()
{
    if( thread->isRunning() )
    {
//                thread->requestInterruption();
    }
    emit sendToWorker("Send Stop",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnGo_To_Rest_Position_clicked()
{

    for (int i = Number_of_Motors; i > 0 ; i--)
    {
    emit sendToWorker("Go To Rest Position",QString::fromStdString(dev_name),i,accel_limit,Motor_rest_position[i-1],velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[i -1],bounds_max[i -1],Cycle_Start_Stop,Cycle_Delay);
    }
    for (int i = 1; i <= Number_of_Motors; i++)
    {
        emit sendToWorker("Send Stop",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[i -1],bounds_max[i -1],Cycle_Start_Stop,Cycle_Delay);
    }
}
void MainWindow::on_btnStart_Motor_clicked()
{
    emit sendToWorker("Send Start",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnRun_Position_clicked()
{
    emit sendToWorker("Go To Position",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnRun_Velocity_clicked()
{
    emit sendToWorker("Run Forever",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);

}

void MainWindow::on_btnStop_Cycle_clicked()
{
    emit sendSetup();

}

void MainWindow::on_Slider_Position_valueChanged(double value)
{
    position = value;
    ui->Counter_Position->setValue(value);
}

void MainWindow::on_Slider_Velocity_Limit_valueChanged(double value)
{
    velocity_limit = value;
    ui->Counter_Velocity_Limit->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}
void MainWindow::on_Slider_Accel_Limit_valueChanged(double value)
{
    accel_limit = value;
    ui->Counter_Accel_Limit->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }

}
void MainWindow::on_Slider_Max_Torque_valueChanged(double value)
{
    max_torque = value;
    ui->Counter_Max_Torque->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_Slider_Feedforward_valueChanged(double value)
{
    feedforward_torque = value;
    ui->Counter_Feedforward->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_Slider_KP_Scale_valueChanged(double value)
{
    kp_scale = value;
    ui->Counter_KP_Scale->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_Slider_KD_Scale_valueChanged(double value)
{
    kd_scale = value;
    ui->Counter_KD_Scale->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_Counter_Position_valueChanged(double value)
{
    position = value;
    ui->Slider_Position->setValue(value);
}

void MainWindow::on_Counter_Velocity_Limit_valueChanged(double value)
{
    velocity_limit = value;
    ui->Slider_Velocity_Limit->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}
void MainWindow::on_Counter_Accel_Limit_valueChanged(double value)
{
    accel_limit = value;
    ui->Slider_Accel_Limit->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}
void MainWindow::on_Counter_Max_Torque_valueChanged(double value)
{
    max_torque = value;
    ui->Slider_Max_Torque->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }

}

void MainWindow::on_Counter_Feedforward_valueChanged(double value)
{
    feedforward_torque = value;
    ui->Slider_Feedforward->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_Counter_KP_Scale_valueChanged(double value)
{
    kp_scale = value;
    ui->Slider_KP_Scale->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_Counter_KD_Scale_valueChanged(double value)
{
    kd_scale = value;
    ui->Slider_KD_Scale->setValue(value);
    if (Dynamic)
    {
        emit sendToWorker("Update Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
}

void MainWindow::on_actionExit_triggered()
{
   QApplication::quit();
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::information(this,"About",
     "Motor Controller\n\n"
     "Rev 2.0"
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
    ui->Slider_Limit_Min->setValue(bounds_min[moteus_id -1]);
    ui->Counter_Limit_Min->setValue(bounds_min[moteus_id -1]);
    ui->Slider_Limit_Max->setValue(bounds_max[moteus_id -1]);
    ui->Counter_Limit_Max->setValue(bounds_max[moteus_id -1]);
    ui->Counter_KP->setValue(kp[moteus_id -1]);
    ui->Slider_KP->setValue(kp[moteus_id -1]);
    ui->Counter_KD->setValue(kd[moteus_id -1]);
    ui->Slider_KD->setValue(kd[moteus_id -1]);
    ui->Counter_KI->setValue(ki[moteus_id -1]);
    ui->Slider_KI->setValue(ki[moteus_id -1]);
    ui->Slider_Gear_Ratio->setValue(Gear_Ratio[moteus_id -1]);
    ui->Counter_Gear_Ratio->setValue(Gear_Ratio[moteus_id -1]);
    ui->Slider_Break_voltage->setValue(Break_Voltage[moteus_id -1]);
    ui->Counter_Break_voltage->setValue(Break_Voltage[moteus_id -1]);
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
    emit sendToWorker("Record Position",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnRun_Recorded_clicked()
{
    emit sendToWorker("Run_Recorded",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnClear_Recorded_clicked()
{
    emit sendToWorker("Clear_Recorded",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);

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

        emit sendToWorker("Open File",fileName,moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
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
        emit sendToWorker("Save File",fileName,moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);

        statusBar()->showMessage(tr("Saved \"%1\"")
                                 .arg(QDir::toNativeSeparators(fileName)));
    }
}

void MainWindow::on_checkBox_Dymamic_clicked()
{
    if (ui->checkBox_Dymamic->isChecked())
    {
        Dynamic = true;
        emit sendToWorker("Set Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
    else
    {
        Dynamic = false;
        emit sendToWorker("Clear Dynamic",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }

}

void MainWindow::on_btnRec_update_limits_clicked()
{
    emit sendToWorker("set motor limits",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}
void MainWindow::on_btnRec_Gear_Ratio_clicked()
{
    emit sendToWorker("set rotor_to_output_ratio",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,Gear_Ratio[moteus_id -1],kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}
void MainWindow::on_btnRec_Break_voltage_clicked()
{
    emit sendToWorker("set break voltage",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,Break_Voltage[moteus_id -1],kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}
void MainWindow::on_btnRun_update_KP_clicked()
{
    emit sendToWorker("set PID",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,ki[moteus_id -1],kp[moteus_id -1],
                      kd[moteus_id -1],bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_Counter_Limit_Min_valueChanged(double value)
{
    bounds_min[moteus_id -1] = value;
    ui->Slider_Limit_Min->setValue(value);

}

void MainWindow::on_Slider_Limit_Min_valueChanged(double value)
{
    bounds_min[moteus_id -1] = value;
    ui->Counter_Limit_Min->setValue(value);
}

void MainWindow::on_Counter_Limit_Max_valueChanged(double value)
{
    bounds_max[moteus_id -1] = value;
    ui->Slider_Limit_Max->setValue(value);
}

void MainWindow::on_Slider_Limit_Max_valueChanged(double value)
{
    bounds_max[moteus_id -1] = value;
    ui->Counter_Limit_Max->setValue(value);
}

void MainWindow::on_Counter_KP_valueChanged(double value)
{
    kp[moteus_id -1] = value;
    ui->Slider_KP->setValue(value);
}

void MainWindow::on_Slider_KP_valueChanged(double value)
{
    kp[moteus_id -1] = value;
    ui->Counter_KP->setValue(value);
}

void MainWindow::on_Counter_KD_valueChanged(double value)
{
    kd[moteus_id -1] = value;
    ui->Slider_KD->setValue(value);
}

void MainWindow::on_Slider_KD_valueChanged(double value)
{
    kd[moteus_id -1] = value;
    ui->Counter_KD->setValue(value);
}

void MainWindow::on_Counter_KI_valueChanged(double value)
{
    ki[moteus_id -1] = value;
    ui->Slider_KI->setValue(value);
}

void MainWindow::on_Slider_KI_valueChanged(double value)
{
    ki[moteus_id -1] = value;
    ui->Counter_KI->setValue(value);
}
void MainWindow::on_Counter_Gear_Ratio_valueChanged(double value)
{
    Gear_Ratio[moteus_id -1] = value;
    ui->Slider_Gear_Ratio->setValue(value);
}

void MainWindow::on_Slider_Gear_Ratio_valueChanged(double value)
{
    Gear_Ratio[moteus_id -1] = value;
    ui->Counter_Gear_Ratio->setValue(value);
}
void MainWindow::on_Counter_Break_voltage_valueChanged(double value)
{
    Break_Voltage[moteus_id -1] = value;
    ui->Slider_Break_voltage->setValue(value);
}

void MainWindow::on_Slider_Break_voltage_valueChanged(double value)
{
    Break_Voltage[moteus_id -1] = value;
    ui->Counter_Break_voltage->setValue(value);
}

void MainWindow::on_btnConf_Write_clicked()
{
    emit sendToWorker("conf write",QString::fromStdString(dev_name),moteus_id,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                      kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
}

void MainWindow::on_btnConf_Read_clicked()
{
    for (int i = 1; i <= Number_of_Motors; i++)
    {
        emit sendToWorker("get motor limits",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
    for (int i = 1; i <= Number_of_Motors; i++)
    {
        emit sendToWorker("get PID",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
    for (int i = 1; i <= Number_of_Motors; i++)
    {
        emit sendToWorker("get rotor_to_output_ratio",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }
    for (int i = 1; i <= Number_of_Motors; i++)
    {
        emit sendToWorker("get break voltage",QString::fromStdString(dev_name),i,accel_limit,position,velocity_limit,max_torque,feedforward_torque,kp_scale,
                          kd_scale,bounds_min[moteus_id -1],bounds_max[moteus_id -1],Cycle_Start_Stop,Cycle_Delay);
    }

    ui->Slider_Limit_Min->setValue(bounds_min[moteus_id -1]);
    ui->Counter_Limit_Min->setValue(bounds_min[moteus_id -1]);
    ui->Slider_Limit_Max->setValue(bounds_max[moteus_id -1]);
    ui->Counter_Limit_Max->setValue(bounds_max[moteus_id -1]);
    ui->Counter_KP->setValue(kp[moteus_id -1]);
    ui->Slider_KP->setValue(kp[moteus_id -1]);
    ui->Counter_KD->setValue(kd[moteus_id -1]);
    ui->Slider_KD->setValue(kd[moteus_id -1]);
    ui->Counter_KI->setValue(ki[moteus_id -1]);
    ui->Slider_KI->setValue(ki[moteus_id -1]);
}
