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
#include "../include/motorosudp/capture.h"
#include "../include/motorosudp/dialogposition.h"


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
  ui->setupUi(this);

  qnode.init();
  timerReal = new QTimer;
  timerSimulation = new QTimer;
  timerSample = new QTimer;
  connect(timerReal,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  connect(timerSimulation,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  connect(timerSample,SIGNAL(timeout()),this,SLOT(SampleHandler()));
  isCartesianCoordinate = false;
  isSimulation = true;

  QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  current_joints_radian.resize(6);
  current_positions.resize(6);
  timerSimulation->start(20);

  points_list = new std::vector <QString> ;
  teaching_step = new std::vector <QString> ;
  points_value = new std::vector <std::vector<double>>;
  index_points=0;
  index_step=0;

//##############Frame Capture#############///

  mOpenCV_videoCapture = new Capture(this);

  QObject::connect(mOpenCV_videoCapture,SIGNAL(newPixmapCaptured_Color()), this, SLOT (handleButton()));

}

void MainWindow::handleButton()
{
    ui->CameraFrame->setPixmap(mOpenCV_videoCapture->pixmap_Color().scaled(640,480));
}

MainWindow::~MainWindow() {
  delete ui;
  delete points_list;
  delete teaching_step;
  delete points_value;
  //delete timer;

  //Frame Capture
  delete ui;
  mOpenCV_videoCapture -> terminate();
}


}  // namespace motorosudp
void motorosudp::MainWindow::RequestPosition()
{
  //qDebug() << "Aloha" ;

  if(!isSimulation)
  {
    socket->GetPulsePosition();
  }
  else {
    if(!isCartesianCoordinate)
    {
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
      //qDebug() << current_joints_radian.at(3);
      qnode.publishJoint(current_joints_radian);
      current_positions =  qnode.getROSPosition(current_joints_radian);
    }

    ui->lineEditS->setText(QString::number(current_joints_radian.at(0)*180/M_PI,'f',4)+" deg");
    ui->lineEditL->setText(QString::number(current_joints_radian.at(1)*180/M_PI,'f',4)+" deg");
    ui->lineEditU->setText(QString::number(current_joints_radian.at(2)*180/M_PI,'f',4)+" deg");
    ui->lineEditR->setText(QString::number(current_joints_radian.at(3)*180/M_PI,'f',4)+" deg");
    ui->lineEditB->setText(QString::number(current_joints_radian.at(4)*180/M_PI,'f',4)+" deg");
    ui->lineEditT->setText(QString::number(current_joints_radian.at(5)*180/M_PI,'f',4)+" deg");

    ui->lineEditX->setText(QString::number(current_positions.at(0)*1000,'f',4)+" mm");
    ui->lineEditY->setText(QString::number(current_positions.at(1)*1000,'f',4)+" mm");
    ui->lineEditZ->setText(QString::number(current_positions.at(2)*1000,'f',4)+" mm");
    ui->lineEditRX->setText(QString::number(current_positions.at(3)*180/M_PI,'f',4)+" deg");
    ui->lineEditRY->setText(QString::number(current_positions.at(4)*180/M_PI,'f',4)+" deg");
    ui->lineEditRZ->setText(QString::number(current_positions.at(5)*180/M_PI,'f',4)+" deg");

      std::vector<double> test;
    //qnode.getJointsPosition(current_positions,true,test);
      timerSimulation->stop();
  }
}


void motorosudp::MainWindow::DataReceiveHandler()
{
    socket->ReceiveData();

//    if(socket->GetReceiveType(*socket->Get_rx_buffer())==socket->ON_SERVO||socket->GetReceiveType(*socket->Get_rx_buffer())==socket->OFF_SERVO)
//    {
//      if(isReadRobotState)
//        timer->start(20);
//    }
//    if(isReadRobotState)
//    {
//      for (int i = 0;i<6;i++) {
//        current_joints_radian.at(i)=socket->Pulse2Joint(*(socket->GetCurrentPulse()+i),i)*M_PI/180;
//     }

//      qnode.publishJoint(current_joints_radian);
//      current_positions =  qnode.getROSPosition(current_joints_radian);
//      if(ui->checkBoxDrawEndPoints->checkState() == Qt::CheckState::Checked)
//      {
//        qnode.publishMarker(current_joints_radian);
//      }
//      if(!isCartesianCoordinate)
//      {
//        ui->textBrowser1->setText(QString::number(current_joints_radian.at(0)*180/M_PI,'g',4));
//        ui->textBrowser2->setText(QString::number(current_joints_radian.at(1)*180/M_PI,'g',4));
//        ui->textBrowser3->setText(QString::number(current_joints_radian.at(2)*180/M_PI,'g',4));
//        ui->textBrowser4->setText(QString::number(current_joints_radian.at(3)*180/M_PI,'g',4));
//        ui->textBrowser5->setText(QString::number(current_joints_radian.at(4)*180/M_PI,'g',4));
//        ui->textBrowser6->setText(QString::number(current_joints_radian.at(5)*180/M_PI,'g',4));
//      }
//      else {
//        ui->textBrowser1->setText(QString::number(current_positions.at(0)*1000));
//        ui->textBrowser2->setText(QString::number(current_positions.at(1)*1000));
//        ui->textBrowser3->setText(QString::number(current_positions.at(2)*1000));
//        ui->textBrowser4->setText(QString::number(current_positions.at(3)*180/M_PI));
//        ui->textBrowser5->setText(QString::number(current_positions.at(4)*180/M_PI));
//        ui->textBrowser6->setText(QString::number(current_positions.at(5)*180/M_PI));
//      }
    //}
}
void motorosudp::MainWindow::on_pushButtonConnect_clicked()
{
  if(ui->pushButtonConnect->text()=="CONNECT")
  {
    ui->pushButtonConnect->setText("DISCONNECT");
    ui->pushButtonServo->setEnabled(true);
    ui->labelStatusServo->setEnabled(true);
    ui->groupBoxConveyer->setEnabled(true);
    ui->groupBoxsafety->setEnabled(true);
    timerReal->start(20);
    timerSimulation->stop();
    isSimulation=false;
    socket = new MotoUDP(QHostAddress(ui->lineEditrobotAddress->text()),10040);
    socket->ConnectMotoman();
    connect(socket->client,SIGNAL(readyRead()),this,SLOT(DataReceiveHandler()));
    ui->labelConnectStatus->setText("CONNECTED SUCCESSFULLY");
    ui->groupBoxJOG->setEnabled(false);
  }
  else {
    ui->pushButtonConnect->setText("CONNECT");
    ui->pushButtonServo->setEnabled(false);
    ui->labelStatusServo->setEnabled(false);
    ui->groupBoxConveyer->setEnabled(false);
    timerSimulation->start(20);
    timerReal->stop();
    isSimulation=true;
    socket->CloseMotoman();
    delete socket;
    ui->groupBoxsafety->setEnabled(false);
    ui->labelConnectStatus->setText("NOT CONNECTED YET");
    ui->groupBoxJOG->setEnabled(true);
  }
}

void motorosudp::MainWindow::on_checkBoxReadRobotState_stateChanged(int arg1)
{
  if(arg1==2){ //Checked
//    timer_->stop();
//    timer->start(20);
//    isReadRobotState = true;
    //ui->groupBoxControl->setEnabled(false);
  }
  else {
//    timer->stop();
//    timer_->start(20);
//    isReadRobotState = false;
    //ui->groupBoxControl->setEnabled(true);
  }
}

void motorosudp::MainWindow::on_pushButtonSERVO_clicked()
{
//  timer->stop();
////  if(ui->pushButtonSERVO->text()=="SERVO ON")
////  {
////    socket->TurnOnServo();
////    //ui->groupBoxPOSITION->setEnabled(true);
////    ui->pushButtonSERVO->setText("SERVO OFF");
////  }
////  else
////  {
////    socket->TurnOffServo();
//////    ui->groupBoxPOSITION->setEnabled(false);
////    ui->pushButtonSERVO->setText("SERVO ON");
//  }
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
//    ui->label1->setText("X");
//    ui->label2->setText("Y");
//    ui->label3->setText("Z");
//    ui->label4->setText("RX");
//    ui->label5->setText("RY");
//    ui->label6->setText("RZ");
//    ui->label1_2->setText("mm");
//    ui->label2_2->setText("mm");
//    ui->label3_2->setText("mm");
//    ui->label1_3->setText("X");
//    ui->label2_3->setText("Y");
//    ui->label3_3->setText("Z");
//    ui->label4_3->setText("RX");
//    ui->label5_3->setText("RY");
//    ui->label6_3->setText("RZ");
//    ui->label1_4->setText("mm");
//    ui->label2_4->setText("mm");
//    ui->label3_4->setText("mm");
    ui->pushButtonAdd_1->setText("X+");
    ui->pushButtonSubb_1->setText("X-");
    ui->pushButtonAdd_2->setText("Y+");
    ui->pushButtonSubb_2->setText("Y-");
    ui->pushButtonAdd_3->setText("Z+");
    ui->pushButtonSubb_3->setText("Z-");
    ui->pushButtonAdd_4->setText("RX+");
    ui->pushButtonSubb_4->setText("RX-");
    ui->pushButtonAdd_5->setText("RY+");
    ui->pushButtonSubb_5->setText("RY-");
    ui->pushButtonAdd_6->setText("RZ+");
    ui->pushButtonSubb_6->setText("RZ-");
  }
  else {
    isCartesianCoordinate = false;
//    ui->label1->setText("S");
//    ui->label2->setText("L");
//    ui->label3->setText("U");
//    ui->label4->setText("R");
//    ui->label5->setText("B");
//    ui->label6->setText("T");
//    ui->label1_2->setText("degree");
//    ui->label2_2->setText("degree");
//    ui->label3_2->setText("degree");
//    ui->label1_3->setText("S");
//    ui->label2_3->setText("L");
//    ui->label3_3->setText("U");
//    ui->label4_3->setText("R");
//    ui->label5_3->setText("B");
//    ui->label6_3->setText("T");
//    ui->label1_4->setText("degree");
//    ui->label2_4->setText("degree");
//    ui->label3_4->setText("degree");
    ui->pushButtonAdd_1->setText("S+");
    ui->pushButtonSubb_1->setText("S-");
    ui->pushButtonAdd_2->setText("L+");
    ui->pushButtonSubb_2->setText("L-");
    ui->pushButtonAdd_3->setText("U+");
    ui->pushButtonSubb_3->setText("U-");
    ui->pushButtonAdd_4->setText("R+");
    ui->pushButtonSubb_4->setText("R-");
    ui->pushButtonAdd_5->setText("B+");
    ui->pushButtonSubb_5->setText("B-");
    ui->pushButtonAdd_6->setText("T+");
    ui->pushButtonSubb_6->setText("T-");
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
//  pos[0] = (ui->textEdit1->toPlainText()).toDouble(p);
//  pos[1] = (ui->textEdit2->toPlainText()).toDouble(p+1);
//  pos[2] = (ui->textEdit3->toPlainText()).toDouble(p+2);
//  pos[3] = (ui->textEdit4->toPlainText()).toDouble(p+3);
//  pos[4] = (ui->textEdit5->toPlainText()).toDouble(p+4);
//  pos[5] = (ui->textEdit6->toPlainText()).toDouble(p+5);
//  vel = (ui->textEditVel->toPlainText()).toUInt(&v);
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



void motorosudp::MainWindow::on_comboBoxMode_activated(int index)
{
  if(index==1)
  {
//    isSimulation = true;
//    timer->start(20);
//    if(ui->pushButtonSERVO->text()=="SERVO OFF")
//    {
//      socket->TurnOnServo();
//      ui->pushButtonSERVO->setText("SERVO ON");
//    }
//    if(ui->pushButtonConnect->text()=="DISCONNECT")
//    {
//      ui->pushButtonConnect->setText("CONNECT");
//      ui->labelConnectStatus->setText("NOT CONNECTED YET");
//      socket->CloseMotoman();
//      delete socket;
//    }
//    ui->groupBoxUDP->setEnabled(false);
//    ui->groupBoxCommand->setEnabled(true);
//    ui->pushButtonSERVO->setEnabled(false);
//    ui->pushButton_2->setEnabled(false);
//    ui->pushButtonMOVL_2->setEnabled(false);
//    ui->groupBoxControl->setEnabled(true);
//    ui->groupBoxDisplay->setEnabled(false);
//    ui->groupBoxGoalPosition->setEnabled(true);
//    ui->checkBoxReadRobotState->setCheckState(Qt::CheckState::Unchecked);
//    ui->checkBoxReadRobotState->setCheckState(Qt::CheckState::Unchecked);
//  }
//  else {
//    isSimulation = false;
//    timer->stop();
//    isReadRobotState = false;
//    ui->groupBoxCommand->setEnabled(false);
//    ui->groupBoxControl->setEnabled(false);
//    ui->groupBoxDisplay->setEnabled(false);
//    ui->groupBoxGoalPosition->setEnabled(false);
//    ui->checkBoxDrawEndPoints->setEnabled(false);
//    ui->checkBoxReadRobotState->setEnabled(false);
//    ui->groupBoxUDP->setEnabled(true);
  }
}

void motorosudp::MainWindow::on_pushButton_clicked()
{
  qnode.updateInteractiveMarkers(true);
  //qnode.publishPosition();
//    ui->textEdit1->setText(ui->textBrowser1->toPlainText());
//    ui->textEdit2->setText(ui->textBrowser2->toPlainText());
//    ui->textEdit3->setText(ui->textBrowser3->toPlainText());
//    ui->textEdit4->setText(ui->textBrowser4->toPlainText());
//    ui->textEdit5->setText(ui->textBrowser5->toPlainText());
//    ui->textEdit6->setText(ui->textBrowser6->toPlainText());
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
  socket->FileReceiveCommand(jobname);
 // socket->JobFile2ByteArray("/home/tapati/motoman_ws/src/motorosudp/TEST04072.JBI");
//  qDebug() << socket->tx_file_buffer->toHex();
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

void motorosudp::MainWindow::on_pushButton_6_clicked()
{
    socket->GetJobFile("/home/tapati/motoman_ws/src/motorosudp/TEST0407.JBI");
}

void motorosudp::MainWindow::on_pushButton_5_clicked()
{
   socket->JobFile2ByteArray("/home/tapati/motoman_ws/src/motorosudp/TEST04072.JBI");
   qDebug() << socket->tx_file_buffer->toHex();
}

void motorosudp::MainWindow::on_pushButton_7_clicked()
{
    char jobname[13] = {'T','E','S','T','0','4','0','7','2','.','J','B','I'};
    socket->FileDeleteCommand(jobname);
}

void motorosudp::MainWindow::on_pushButton_8_clicked()
{
    //socket->ConnectToPLC();
}

void motorosudp::MainWindow::on_pushButton_9_clicked()
{

  //testTimer->start(20);
}
void motorosudp::MainWindow::InitObjectPosition(){
//  object_position[0]=243317;
//  object_position[1]=-211198;
//  object_position[2]=-121998;
//  object_position[3]=1800000;
//  object_position[4]=0;
//  object_position[5]=0;
//  int final_position[2];
//  final_position[0] = 0;
//  final_position[1] = 0;
//  alpha = (final_position[1]-first_position[1])/(final_position[0]-first_position[0]);
}
void motorosudp::MainWindow::sendDataPosition()
{

    //object_position[0]+=1234;
//  if(object_position[1]<0)
//    object_position[1]+=2000;

   // socket->WriteVarPosition(45,object_position[0],object_position[1],object_position[2],object_position[3],object_position[4],object_position[5]);

}

void motorosudp::MainWindow::on_pushButton_10_clicked()
{
    InitObjectPosition();
}

void motorosudp::MainWindow::on_pushButton_11_clicked()
{
    //testTimer->stop();
    InitObjectPosition();
    //object_position[1]+=50000;
    //socket->WriteVarPosition(45,object_position[0],object_position[1],object_position[2],object_position[3],object_position[4],object_position[5]);
}

void motorosudp::MainWindow::on_pushButtonAddPoint_clicked()
{
    points_list->push_back("C"+QString::number(index_points));
    points_value->push_back(current_joints_radian);
    ui->listWidgetPoints->addItem("C"+QString::number(index_points));
    std::cout << "C" << index_points;
    for (int i = 0;i<6;i++) {
      std::cout << " " << current_joints_radian.at(i);
    }
    std::cout << std::endl;
    index_points++;
}

void motorosudp::MainWindow::on_pushButtonRemovePoint_clicked()
{
  //std::cout << ui->listWidgetPoints->currentRow() << std::endl;
  if(ui->listWidgetPoints->currentRow()!=-1)
  {
    points_list->erase(points_list->begin()+ui->listWidgetPoints->currentRow());
    points_value->erase(points_value->begin()+ui->listWidgetPoints->currentRow());
    ui->listWidgetPoints->takeItem(ui->listWidgetPoints->currentRow());
  }

}

void motorosudp::MainWindow::on_pushButtonCheckPoint_clicked()
{

}

void motorosudp::MainWindow::on_pushButtonModifyPoint_clicked()
{
    dialogPosition = new DialogPosition();
    dialogPosition->show();
}



void motorosudp::MainWindow::on_CameraOn_Button_clicked()
{
    mOpenCV_videoCapture->start(QThread::HighPriority);
}

void motorosudp::MainWindow::on_CameraOff_Button_clicked()
{
    mOpenCV_videoCapture->pipe.stop();
    mOpenCV_videoCapture->terminate();
}

void motorosudp::MainWindow::on_StreamOption_comboBox_currentIndexChanged(const QString &arg1)
{
    if(ui->StreamOption_comboBox->currentText()=="Color Stream")
    {
         mOpenCV_videoCapture->StreamOption = 0;
    }
    else if(ui->StreamOption_comboBox->currentText()=="Depth Stream")
    {
        mOpenCV_videoCapture->StreamOption = 1;
    }
}

void motorosudp::MainWindow::on_Background_Button_clicked()
{
  mOpenCV_videoCapture->MaskSingal = 0;
  QPixmap OriginalPix(*ui->CameraFrame->pixmap());

  mOpenCV_videoCapture->BackgroundImage = OriginalPix.toImage();

  if(Find_Mask_Ready)
  {
      band = new Resizable_rubber_band(ui->CameraFrame);
      band->move(100, 100);
      band->resize(50, 50);
      band->setMinimumSize(30, 30);
      Find_Mask_Ready = 0;
  }
}

void motorosudp::MainWindow::on_Mask_Button_clicked()
{
    QImage Background = mOpenCV_videoCapture->BackgroundImage;

    QPoint center;
    center = band->pos();
    if (!Find_Mask_Ready)
    {
        mOpenCV_videoCapture->MaskTLPoint = cv::Point((center.x() ),(center.y() ));
        mOpenCV_videoCapture->MaskBRPoint = cv::Point((center.x() + band->width()),(center.y() + band->height()));
        mOpenCV_videoCapture->MaskSingal = 1;
        band->close();
        Find_Mask_Ready = 1;

    }
}

Resizable_rubber_band::Resizable_rubber_band(QWidget *parent) : QWidget(parent) {
    //tell QSizeGrip to resize this widget instead of top-level window
    setWindowFlags(Qt::SubWindow);
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    QSizeGrip* grip1 = new QSizeGrip(this);
    QSizeGrip* grip2 = new QSizeGrip(this);
    layout->addWidget(grip1, 0, Qt::AlignLeft | Qt::AlignTop);
    layout->addWidget(grip2, 0, Qt::AlignRight | Qt::AlignBottom);
    rubberband = new QRubberBand(QRubberBand::Rectangle, this);
    rubberband->move(0, 0);
    rubberband->show();
    show();
}

void Resizable_rubber_band::resizeEvent(QResizeEvent *) {
    rubberband->resize(size());
}
void Resizable_rubber_band::mousePressEvent(QMouseEvent *event)
{
    if(rubberband->geometry().contains(event->pos()))
    {
        rubberband_offset = event->pos() - rubberband->pos();
        move_rubberband = true;
    }
}

void Resizable_rubber_band::mouseMoveEvent(QMouseEvent *event)
{
    if(move_rubberband)
    {
        rubberband->move(event->pos() - rubberband_offset);
    }
}

void Resizable_rubber_band::mouseReleaseEvent(QMouseEvent *event)
{
    move_rubberband = false;
}


void motorosudp::MainWindow::on_ChooseObject_Button_clicked()
{
    mOpenCV_videoCapture->ChooseOBject ++;
}

void motorosudp::MainWindow::on_Size_Button_clicked()
{
    mOpenCV_videoCapture->Mearsure_Ready = !mOpenCV_videoCapture->Mearsure_Ready;
}

void motorosudp::MainWindow::on_pushButtonConnectSerial_clicked()
{
  if(qnode.connectSerial()){
    ui->labelSerialStatus->setText("Connected");
    qnode.sendFirstDataToSerial(ui->spinBoxEncoderType->value(),ui->doubleSpinBoxRatio->value());
    timerSample->start(ui->spinBoxSampleTime->value());
  }
  else{
    QMessageBox::critical(this,"Error","Can not connect to serial");
  }
}
void motorosudp::MainWindow::SampleHandler(){
  ui->lineEditSpeed->setText(QString::number(qnode.getVelocity()*15));
}
