/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/motorosudp/main_window.hpp"
#include "../include/motorosudp/motoudp.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace motorosudp {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
  ui = new Ui::MainWindowDesign;
  timer = new QTimer;
  timer_ = new QTimer;
  testTimer = new QTimer;
  connect(timer,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  connect(timer_,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  connect(testTimer,SIGNAL(timeout()),this,SLOT(sendDataPosition()));
  ui->setupUi(this);
  qnode.init();
  QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  isCartesianCoordinate = false;
  isSimulation = false;
  isReadRobotState = false;
  current_joints_radian.resize(6);
  timer_->start(20);
  testTimer->start(1000);
  temp=32;
}

MainWindow::~MainWindow() {
  delete ui;
  delete timer;
}


}  // namespace motorosudp
void motorosudp::MainWindow::RequestPosition()
{
  if(isReadRobotState)
  {
    socket->GetPulsePosition();
  }
  else {
    if(ui->pushButtonAdd_1->isDown())
    {
      current_joints_radian.at(0) += M_PI/1000;
    }
    if(ui->pushButtonAdd_2->isDown())
    {
      current_joints_radian.at(1) += M_PI/1000;
    }
    if(ui->pushButtonAdd_3->isDown())
    {
      current_joints_radian.at(2) += M_PI/1000;
    }
    if(ui->pushButtonAdd_4->isDown())
    {
      current_joints_radian.at(3) += M_PI/1000;
    }
    if(ui->pushButtonAdd_5->isDown())
    {
      current_joints_radian.at(4) += M_PI/1000;
    }
    if(ui->pushButtonAdd_6->isDown())
    {
      current_joints_radian.at(5) += M_PI/1000;
    }
    if(ui->pushButtonSubb_1->isDown())
    {
      current_joints_radian.at(0) -= M_PI/1000;
    }
    if(ui->pushButtonSubb_2->isDown())
    {
      current_joints_radian.at(1) -= M_PI/1000;
    }
    if(ui->pushButtonSubb_3->isDown())
    {
      current_joints_radian.at(2) -= M_PI/1000;
    }
    if(ui->pushButtonSubb_4->isDown())
    {
      current_joints_radian.at(3) -= M_PI/1000;
    }
    if(ui->pushButtonSubb_5->isDown())
    {
      current_joints_radian.at(4) -= M_PI/1000;
    }
    if(ui->pushButtonSubb_6->isDown())
    {
      current_joints_radian.at(5) -= M_PI/1000;
    }
    qnode.publishJoint(current_joints_radian);
    current_positions =  qnode.getROSPosition(current_joints_radian);
    if(!isCartesianCoordinate)
    {
      ui->textBrowser1->setText(QString::number(current_joints_radian.at(0)*180/M_PI));
      ui->textBrowser2->setText(QString::number(current_joints_radian.at(1)*180/M_PI));
      ui->textBrowser3->setText(QString::number(current_joints_radian.at(2)*180/M_PI));
      ui->textBrowser4->setText(QString::number(current_joints_radian.at(3)*180/M_PI));
      ui->textBrowser5->setText(QString::number(current_joints_radian.at(4)*180/M_PI));
      ui->textBrowser6->setText(QString::number(current_joints_radian.at(5)*180/M_PI));
    }
    else {
      ui->textBrowser1->setText(QString::number(current_positions.at(0)*1000));
      ui->textBrowser2->setText(QString::number(current_positions.at(1)*1000));
      ui->textBrowser3->setText(QString::number(current_positions.at(2)*1000));
      ui->textBrowser4->setText(QString::number(current_positions.at(3)*180/M_PI));
      ui->textBrowser5->setText(QString::number(current_positions.at(4)*180/M_PI));
      ui->textBrowser6->setText(QString::number(current_positions.at(5)*180/M_PI));
    }
  }
}


void motorosudp::MainWindow::DataReceiveHandler()
{
    socket->ReceiveData();
    if(socket->GetReceiveType(*socket->Get_rx_buffer())==socket->ON_SERVO||socket->GetReceiveType(*socket->Get_rx_buffer())==socket->OFF_SERVO)
    {
      if(isReadRobotState)
        timer->start(20);
    }
    if(isReadRobotState)
    {
      for (int i = 0;i<6;i++) {
        current_joints_radian.at(i)=socket->Pulse2Joint(*(socket->GetCurrentPulse()+i),i)*M_PI/180;
     }

      qnode.publishJoint(current_joints_radian);
      current_positions =  qnode.getROSPosition(current_joints_radian);
      if(ui->checkBoxDrawEndPoints->checkState() == Qt::CheckState::Checked)
      {
        qnode.publishMarker(current_joints_radian);
      }
      if(!isCartesianCoordinate)
      {
        ui->textBrowser1->setText(QString::number(current_joints_radian.at(0)*180/M_PI,'g',4));
        ui->textBrowser2->setText(QString::number(current_joints_radian.at(1)*180/M_PI,'g',4));
        ui->textBrowser3->setText(QString::number(current_joints_radian.at(2)*180/M_PI,'g',4));
        ui->textBrowser4->setText(QString::number(current_joints_radian.at(3)*180/M_PI,'g',4));
        ui->textBrowser5->setText(QString::number(current_joints_radian.at(4)*180/M_PI,'g',4));
        ui->textBrowser6->setText(QString::number(current_joints_radian.at(5)*180/M_PI,'g',4));
      }
      else {
        ui->textBrowser1->setText(QString::number(current_positions.at(0)*1000));
        ui->textBrowser2->setText(QString::number(current_positions.at(1)*1000));
        ui->textBrowser3->setText(QString::number(current_positions.at(2)*1000));
        ui->textBrowser4->setText(QString::number(current_positions.at(3)*180/M_PI));
        ui->textBrowser5->setText(QString::number(current_positions.at(4)*180/M_PI));
        ui->textBrowser6->setText(QString::number(current_positions.at(5)*180/M_PI));
      }
    }
}
void motorosudp::MainWindow::on_pushButtonConnect_clicked()
{
  if(ui->pushButtonConnect->text()=="CONNECT")
  {
    ui->pushButtonConnect->setText("DISCONNECT");
    ui->groupBoxCommand->setEnabled(true);
    //ui->groupBoxControl->setEnabled(true);
    ui->groupBoxDisplay->setEnabled(true);
    //ui->groupBoxGoalPosition->setEnabled(true);
    ui->labelConnectStatus->setText("CONNECTED");
    ui->checkBoxDrawEndPoints->setEnabled(true);
    ui->checkBoxReadRobotState->setEnabled(true);
    socket = new MotoUDP(QHostAddress(ui->comboBoxIPadress->currentText()),ui->textEditPort->toPlainText().toInt());
    socket->ConnectMotoman();
    connect(socket->client,SIGNAL(readyRead()),this,SLOT(DataReceiveHandler()));
  }
  else {
    timer->stop();
    ui->pushButtonConnect->setText("CONNECT");
    ui->groupBoxCommand->setEnabled(false);
    //ui->groupBoxControl->setEnabled(false);
    ui->groupBoxDisplay->setEnabled(false);
    //ui->groupBoxGoalPosition->setEnabled(false);
    ui->labelConnectStatus->setText("NOT CONNECTED YET");
    ui->checkBoxDrawEndPoints->setEnabled(false);
    ui->checkBoxReadRobotState->setEnabled(false);
    socket->CloseMotoman();
    delete socket;
  }
}

void motorosudp::MainWindow::on_checkBoxReadRobotState_stateChanged(int arg1)
{
  if(arg1==2){ //Checked
    timer_->stop();
    timer->start(20);
    isReadRobotState = true;
    ui->groupBoxControl->setEnabled(false);
  }
  else {
    timer->stop();
    timer_->start(20);
    isReadRobotState = false;
    ui->groupBoxControl->setEnabled(true);
  }
}

void motorosudp::MainWindow::on_pushButtonSERVO_clicked()
{
  timer->stop();
  if(ui->pushButtonSERVO->text()=="SERVO ON")
  {
    socket->TurnOnServo();
    //ui->groupBoxPOSITION->setEnabled(true);
    ui->pushButtonSERVO->setText("SERVO OFF");
  }
  else
  {
    socket->TurnOffServo();
//    ui->groupBoxPOSITION->setEnabled(false);
    ui->pushButtonSERVO->setText("SERVO ON");
  }
}

void motorosudp::MainWindow::on_pushButtonHOME_clicked()
{
  int32_t pos[6] = {0,0,0,0,0,0};
  socket->WritePulse(1,0,1500,pos);
}

void motorosudp::MainWindow::on_comboBoxCoordinate_currentIndexChanged(int index)
{
  if(index==1)
  {
    isCartesianCoordinate = true;
    ui->label1->setText("X");
    ui->label2->setText("Y");
    ui->label3->setText("Z");
    ui->label4->setText("RX");
    ui->label5->setText("RY");
    ui->label6->setText("RZ");
    ui->label1_2->setText("mm");
    ui->label2_2->setText("mm");
    ui->label3_2->setText("mm");
    ui->label1_3->setText("X");
    ui->label2_3->setText("Y");
    ui->label3_3->setText("Z");
    ui->label4_3->setText("RX");
    ui->label5_3->setText("RY");
    ui->label6_3->setText("RZ");
    ui->label1_4->setText("mm");
    ui->label2_4->setText("mm");
    ui->label3_4->setText("mm");
//    ui->pushButtonAdd_1->setText("X+");
//    ui->pushButtonSubb_1->setText("X-");
//    ui->pushButtonAdd_2->setText("Y+");
//    ui->pushButtonSubb_2->setText("Y-");
//    ui->pushButtonAdd_3->setText("Z+");
//    ui->pushButtonSubb_3->setText("Z-");
//    ui->pushButtonAdd_4->setText("RX+");
//    ui->pushButtonSubb_4->setText("RX-");
//    ui->pushButtonAdd_5->setText("RY+");
//    ui->pushButtonSubb_5->setText("RY-");
//    ui->pushButtonAdd_6->setText("RZ+");
//    ui->pushButtonSubb_6->setText("RZ-");
  }
  else {
    isCartesianCoordinate = false;
    ui->label1->setText("S");
    ui->label2->setText("L");
    ui->label3->setText("U");
    ui->label4->setText("R");
    ui->label5->setText("B");
    ui->label6->setText("T");
    ui->label1_2->setText("degree");
    ui->label2_2->setText("degree");
    ui->label3_2->setText("degree");
    ui->label1_3->setText("S");
    ui->label2_3->setText("L");
    ui->label3_3->setText("U");
    ui->label4_3->setText("R");
    ui->label5_3->setText("B");
    ui->label6_3->setText("T");
    ui->label1_4->setText("degree");
    ui->label2_4->setText("degree");
    ui->label3_4->setText("degree");
//    ui->pushButtonAdd_1->setText("S+");
//    ui->pushButtonSubb_1->setText("S-");
//    ui->pushButtonAdd_2->setText("L+");
//    ui->pushButtonSubb_2->setText("L-");
//    ui->pushButtonAdd_3->setText("U+");
//    ui->pushButtonSubb_3->setText("U-");
//    ui->pushButtonAdd_4->setText("R+");
//    ui->pushButtonSubb_4->setText("R-");
//    ui->pushButtonAdd_5->setText("B+");
//    ui->pushButtonSubb_5->setText("B-");
//    ui->pushButtonAdd_6->setText("T+");
//    ui->pushButtonSubb_6->setText("T-");
  }
}

void motorosudp::MainWindow::on_checkBoxDrawEndPoints_stateChanged(int arg1)
{
    if(arg1 != 2)
    {
      qnode.deleteMarker();
    }
}

void motorosudp::MainWindow::on_pushButtonMOVJ_clicked()
{
   GetPositionandMove(1,0);
}
void motorosudp::MainWindow::GetPositionandMove(u_int16_t type, u_int32_t class_speed)
{
  bool p[6],v;
  double pos[6];
  u_int32_t vel;
  pos[0] = (ui->textEdit1->toPlainText()).toDouble(p);
  pos[1] = (ui->textEdit2->toPlainText()).toDouble(p+1);
  pos[2] = (ui->textEdit3->toPlainText()).toDouble(p+2);
  pos[3] = (ui->textEdit4->toPlainText()).toDouble(p+3);
  pos[4] = (ui->textEdit5->toPlainText()).toDouble(p+4);
  pos[5] = (ui->textEdit6->toPlainText()).toDouble(p+5);
  vel = (ui->textEditVel->toPlainText()).toUInt(&v);
  if(p[0]&&p[1]&&p[2]&&p[3]&&p[4]&&p[5]&&v)
  {
    if(vel>100||vel<0.1)
    {
      QMessageBox::critical(this,"Error","Please check the velocity range: 0.1% - 100%");
    }
    else
    {
      if(isCartesianCoordinate)
      {
        int32_t cartesian_pos[6];
        for (int i = 0;i<3;i++) {
          cartesian_pos[i] = int32_t(pos[i]*1000);
        }
        for (int i = 3;i<6;i++) {
          cartesian_pos[i] = int32_t(pos[i]*10000);
        }
        socket->WritePosition(type,class_speed,vel*100,cartesian_pos);
      }
      else
      {
        int32_t joints_pos[6];
        for (int i = 0;i<6;i++) {
          joints_pos[i] = socket->Joint2Pulse(pos[i],i);
        }
        socket->WritePulse(type,class_speed,vel*100,joints_pos);
      }
    }
  }
  else
  {
     QMessageBox::critical(this,"Error","Please check the number format of positions and velocity!");
  }
}

void motorosudp::MainWindow::on_pushButtonMOVL_clicked()
{
  GetPositionandMove(2,1);
}

void motorosudp::MainWindow::on_pushButton_2_clicked() //JobStart
{
    socket->StartJob();
}

void motorosudp::MainWindow::on_comboBoxMode_activated(int index)
{
  if(index==1)
  {
    isSimulation = true;
    timer->start(20);
    if(ui->pushButtonSERVO->text()=="SERVO OFF")
    {
      socket->TurnOnServo();
      ui->pushButtonSERVO->setText("SERVO ON");
    }
    if(ui->pushButtonConnect->text()=="DISCONNECT")
    {
      ui->pushButtonConnect->setText("CONNECT");
      ui->labelConnectStatus->setText("NOT CONNECTED YET");
      socket->CloseMotoman();
      delete socket;
    }
    ui->groupBoxUDP->setEnabled(false);
    ui->groupBoxCommand->setEnabled(true);
    ui->pushButtonSERVO->setEnabled(false);
    ui->pushButton_2->setEnabled(false);
    ui->pushButtonMOVL_2->setEnabled(false);
    ui->groupBoxControl->setEnabled(true);
    ui->groupBoxDisplay->setEnabled(false);
    ui->groupBoxGoalPosition->setEnabled(true);
    ui->checkBoxReadRobotState->setCheckState(Qt::CheckState::Unchecked);
    ui->checkBoxReadRobotState->setCheckState(Qt::CheckState::Unchecked);
  }
  else {
    isSimulation = false;
    timer->stop();
    isReadRobotState = false;
    ui->groupBoxCommand->setEnabled(false);
    ui->groupBoxControl->setEnabled(false);
    ui->groupBoxDisplay->setEnabled(false);
    ui->groupBoxGoalPosition->setEnabled(false);
    ui->checkBoxDrawEndPoints->setEnabled(false);
    ui->checkBoxReadRobotState->setEnabled(false);
    ui->groupBoxUDP->setEnabled(true);
  }
}

void motorosudp::MainWindow::on_pushButton_clicked()
{
    ui->textEdit1->setText(ui->textBrowser1->toPlainText());
    ui->textEdit2->setText(ui->textBrowser2->toPlainText());
    ui->textEdit3->setText(ui->textBrowser3->toPlainText());
    ui->textEdit4->setText(ui->textBrowser4->toPlainText());
    ui->textEdit5->setText(ui->textBrowser5->toPlainText());
    ui->textEdit6->setText(ui->textBrowser6->toPlainText());
}


void motorosudp::MainWindow::sendDataPosition()
{
  if(isReadRobotState)
  {
    if(temp==43)
    {
      temp=32;
    }
    socket->WriteVarPosition(42,160139,205057,87615,-1799971,58562,520114);
    temp++;
  }

}
//void motorosudp::MainWindow::on_pushButtonAdd_1_clicked(bool checked)
//{
//    while(checked)
//    {
//      current_joints_radian.at(0) += M_PI/1000;
//    }
//}

void motorosudp::MainWindow::on_pushButton_3_clicked()
{
//    socket->WriteVarPosition(42,0,0,0,0,0,0);
  char jobname[32] = {'T','E','S','T','0','4','0','7','.','J','B','I'};
//  socket->SelectJob(jobname);
//  socket->FileReceiveCommand(jobname);
  socket->JobFile2ByteArray("/home/tapati/motoman_ws/src/motorosudp/TEST04072.JBI");
  qDebug() << socket->tx_file_buffer->toHex();
}

void motorosudp::MainWindow::on_pushButtonMOVL_2_clicked()
{
//  QString a;
//  a.resize(32);
//  a = "TEST0407";

  char jobname[32] = {'T','E','S','T','0','4','0','7','2','.','J','B','I'};
//  socket->SelectJob(jobname);
//  socket->FileReceiveCommand(jobname);
  socket->FileTransmitCommand(jobname);
}

void motorosudp::MainWindow::on_pushButton_4_clicked()
{
   // socket->GetJobFile();
  char jobname[13] = {'T','E','S','T','0','4','0','7','2','.','J','B','I'};
//  socket->SelectJob(jobname);
//  socket->FileReceiveCommand(jobname);
  socket->FileTransmitCommand(jobname);
}
