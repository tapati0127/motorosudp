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
#include <time.h>

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
  setWindowIcon(QIcon("://images/motomini.png"));
  qnode.init();
  timerReal = new QTimer;
  timerSimulation = new QTimer;
  timerSample = new QTimer;
  timerTimeout = new QTimer;
  connect(timerReal,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  connect(timerSimulation,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  connect(timerSample,SIGNAL(timeout()),this,SLOT(SampleHandler()));
  connect(timerTimeout,SIGNAL(timeout()),this,SLOT(TimeoutHandler()));
  connect(&qnode,SIGNAL(dataReceive()),this,SLOT(SerialDataReceive()));
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
  //object_position = new std::vector <std::vector<int>>;
  index_points=0;
  index_step=0;
  jogspeed=1;
  isVarPosition=false;
  isPublishMarker=false;
  current_object_position.resize(6);
//##############Frame Capture#############///

  mOpenCV_videoCapture = new Capture(this);

  QObject::connect(mOpenCV_videoCapture,SIGNAL(newPixmapCaptured_Color()), this, SLOT (handleButton()));
  QObject::connect(mOpenCV_videoCapture,SIGNAL(foundObject()), this, SLOT (foundObject()));

}
void motorosudp::MainWindow::TimeoutHandler(){
  timerTimeout->stop();
  QMessageBox::critical(this,"Error","Cannot connect to server: Timeout.");
  ui->pushButtonConnect->setText("CONNECT");
  ui->pushButtonServo->setEnabled(false);
  ui->labelStatusServo->setEnabled(false);
  ui->groupBoxConveyer->setEnabled(false);
  timerSimulation->start(20);
  timerReal->stop();
  isSimulation=true;
  socket->CloseMotoman();
  delete socket;
  ui->labelConnectStatus->setText("NOT CONNECTED YET");
  ui->groupBoxJOG->setEnabled(true);
  timerSample->stop();
  ui->pushButtonSendJOB->setEnabled(false);
  ui->pushButtonStartMasterJOB->setEnabled(false);
  ui->pushButtonGetJobFile->setEnabled(false);
  ui->groupBoxConveyer->setEnabled(false);
  ui->groupBoxServo->setEnabled(false);
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
void getGlobal(std::vector<double> &a){
  a = x;
}
void clearGlobal(){
  x.clear();
}

}  // namespace motorosudp
void motorosudp::MainWindow::RequestPosition()
{
  //current_positions =  qnode.getROSPosition(current_joints_radian);
  if(!isSimulation)
  {
    //socket->GetPulsePosition();
    //socket->GetPosition();
  }
  else {
    if(!isCartesianCoordinate)
    {
      std::vector<double> temp = current_joints_radian;
      if(ui->pushButtonAdd_1->isDown())
      {
        temp.at(0) += M_PI/1000*jogspeed;
      }
      if(ui->pushButtonAdd_2->isDown())
      {
        temp.at(1) += M_PI/1000*jogspeed;
      }
      if(ui->pushButtonAdd_3->isDown())
      {
        temp.at(2) += M_PI/1000*jogspeed;
      }
      if(ui->pushButtonAdd_4->isDown())
      {
        temp.at(3) += M_PI/1000*jogspeed;
      }
      if(ui->pushButtonAdd_5->isDown())
      {
        temp.at(4) += M_PI/1000*jogspeed;
      }
      if(ui->pushButtonAdd_6->isDown())
      {
        temp.at(5) += M_PI/1000*jogspeed;
      }
      if(ui->pushButtonSubb_1->isDown())
      {
        temp.at(0) -= M_PI/1000*jogspeed;
      }
      if(ui->pushButtonSubb_2->isDown())
      {
        temp.at(1) -= M_PI/1000*jogspeed;
      }
      if(ui->pushButtonSubb_3->isDown())
      {
        temp.at(2) -= M_PI/1000*jogspeed;
      }
      if(ui->pushButtonSubb_4->isDown())
      {
        temp.at(3) -= M_PI/1000*jogspeed;
      }
      if(ui->pushButtonSubb_5->isDown())
      {
        temp.at(4) -= M_PI/1000*jogspeed;
      }
      if(ui->pushButtonSubb_6->isDown())
      {
        temp.at(5) -= M_PI/1000*jogspeed;
      }
      if(qnode.jointLimit(temp)){
        current_joints_radian=temp;
      }
      else{
        QMessageBox::critical(this,"Error","Outside the joint limit!");
      }
      //qDebug() << current_joints_radian.at(3);
    }
    else{
      std::vector<double> temp_pos = current_positions;

       // qDebug() << temp_pos.at(0) << temp_pos.at(1) << temp_pos.at(2) << temp_pos.at(3) << temp_pos.at(4) << temp_pos.at(5);

      if(ui->pushButtonAdd_1->isDown())
      {
        temp_pos.at(0) += 0.0006*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonAdd_2->isDown())
      {
        temp_pos.at(1) += 0.0006*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonAdd_3->isDown())
      {
        temp_pos.at(2) += 0.0006*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonAdd_4->isDown())
      {
        temp_pos.at(3) += 0.0075*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonAdd_5->isDown())
      {
        temp_pos.at(4) += 0.0075*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonAdd_6->isDown())
      {
        temp_pos.at(5) += 0.0075*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonSubb_1->isDown())
      {
        temp_pos.at(0) -= 0.0006*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonSubb_2->isDown())
      {
        temp_pos.at(1) -= 0.0006*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonSubb_3->isDown())
      {
        temp_pos.at(2) -= 0.0006*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonSubb_4->isDown())
      {
        temp_pos.at(3) -= 0.0075*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonSubb_5->isDown())
      {
        temp_pos.at(4) -= 0.0075*jogspeed;
        catesianCaculate(temp_pos);
      }
      if(ui->pushButtonSubb_6->isDown())
      {
        temp_pos.at(5) -= 0.0075*jogspeed;
        catesianCaculate(temp_pos);
      }

    }
    if(ui->radioButtonInteractiveMarker->isChecked()){
      std::vector<double> a;
      getGlobal(a);
      //std::cout << a.size() << std::endl;
      if(a.size()==6){
        qnode.getJointsPosition(a,true,current_joints_radian);
      }
    }
    qnode.publishJoint(current_joints_radian);
    if(ui->checkBoxTrajectory->checkState() == Qt::CheckState::Checked)
          {
            if(current_joints_radian!=pre_current_joints_radian)
            qnode.publishMarker(current_joints_radian);
          };
    pre_current_joints_radian=current_joints_radian;
    current_positions =  qnode.getROSPosition(current_joints_radian);
    for (int i=0;i<current_positions.size();i++) {
      current_positions.at(i)=round(current_positions.at(i)*1000000)/1000000;
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

     // std::vector<double> test;
    //qnode.getJointsPosition(current_positions,true,test);
     // timerSimulation->stop();
  }
}

void motorosudp::MainWindow::DataReceiveHandler()
{
    timerTimeout->stop();
    socket->ReceiveData();
    if(socket->GetReceiveType()==socket->GET_PULSE)
          {for(int i = 0;i<6;i++) {current_joints_radian.at(i)=socket->Pulse2Joint(*(socket->GetCurrentPulse()+i),i)*M_PI/180;};
      current_positions =  qnode.getROSPosition(current_joints_radian);
      current_positions_int.resize(3);
      current_positions_int.at(0)=current_positions.at(0)*1000000;
      current_positions_int.at(1)=current_positions.at(1)*1000000;
      current_positions_int.at(2)=(current_positions.at(2)-0.143)*1000000;
      //std::cout <<"Int Position" <<current_positions_int.at(0) << " " << current_positions_int.at(1) << " " <<current_positions_int.at(2) << std::endl;
      qnode.publishJoint(current_joints_radian);
      if(ui->checkBoxTrajectory->checkState() == Qt::CheckState::Checked)
            {
              if(current_joints_radian!=pre_current_joints_radian)
              qnode.publishMarker(current_joints_radian);
            };
      pre_current_joints_radian=current_joints_radian;

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
    }



}
void motorosudp::MainWindow::on_pushButtonConnect_clicked()
{
  if(ui->pushButtonConnect->text()=="CONNECT")
  {
    ui->pushButtonConnect->setText("DISCONNECT");
    ui->pushButtonServo->setEnabled(true);
    ui->labelStatusServo->setEnabled(true);
    ui->groupBoxConveyer->setEnabled(true);
   // ui->groupBoxsafety->setEnabled(true);
    //timerReal->start(20);
    timerSimulation->stop();
    isSimulation=false;
    socket = new MotoUDP(QHostAddress(ui->lineEditrobotAddress->text()),10040);
    socket->ConnectMotoman();
    connect(socket->client,SIGNAL(readyRead()),this,SLOT(DataReceiveHandler()));
    ui->labelConnectStatus->setText("CONNECTED SUCCESSFULLY");
    ui->groupBoxJOG->setEnabled(false);
    ui->groupBoxConveyer->setEnabled(true);
    ui->groupBoxServo->setEnabled(true);
    ui->pushButtonSendJOB->setEnabled(true);
    ui->pushButtonStartMasterJOB->setEnabled(true);
    ui->pushButtonGetJobFile->setEnabled(true);

    socket->GetPulsePosition();
    //socket->GetPosition();
    timerSample->start(2);
    mode = 1;
    time = clock();
    timerTimeout->start(1000);
    connect(socket,SIGNAL(transferAllData()),this,SLOT(fileTransmitDone()));
    connect(socket,SIGNAL(receiveAllData()),this,SLOT(fileReceiveDone()));
    connect(socket,SIGNAL(transferError()),this,SLOT(fileTransmitError()));
    connect(socket,SIGNAL(receiveError()),this,SLOT(fileReceiveError()));
  }
  else {
    timerSample->stop();
    ui->pushButtonConnect->setText("CONNECT");
    ui->pushButtonServo->setEnabled(false);
    ui->labelStatusServo->setEnabled(false);
    ui->groupBoxConveyer->setEnabled(false);
    timerSimulation->start(20);
    //timerReal->stop();
    isSimulation=true;
    socket->CloseMotoman();
    delete socket;
    //ui->groupBoxsafety->setEnabled(false);
    ui->labelConnectStatus->setText("NOT CONNECTED YET");
    ui->groupBoxJOG->setEnabled(true);
    ui->pushButtonSendJOB->setEnabled(false);
    ui->pushButtonStartMasterJOB->setEnabled(false);
    ui->pushButtonGetJobFile->setEnabled(false);
    ui->groupBoxConveyer->setEnabled(false);
    ui->groupBoxServo->setEnabled(false);
  }
}

void motorosudp::MainWindow::on_comboBoxCoordinate_currentIndexChanged(int index)
{
  timerSimulation->stop();
  if(index==1)
  {
    isCartesianCoordinate = true;
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
  timerSimulation->start(20);
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









/*//void motorosudp::MainWindow::on_pushButtonMOVL_2_clicked()
//{
////  QString a;
////  a.resize(32);
////  a = "TEST0407";

//  char jobname[32] = {'T','E','S','T','0','4','0','7','2','.','J','B','I'};
////  socket->SelectJob(jobname);
////  socket->FileReceiveCommand(jobname);
//  socket->FileTransmitCommand(jobname);
//}*/


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



void motorosudp::MainWindow::on_pushButtonAddPoint_clicked()
{
    //points_list->push_back("C"+QString::number(index_points));
    points_value->push_back(current_joints_radian);
    int b = points_value->size()-1;
    QString a = "C"+QString::number(b);
    ui->listWidgetPoints->addItem(a);
    std::cout << "C" << b;
    for (int i = 0;i<6;i++) {
      std::cout << " " << current_joints_radian.at(i);
    }
    std::cout << std::endl;
    qnode.publishPose(current_joints_radian,b*3,0.05);
    qnode.publishText(current_joints_radian,b*3+30000,a.toStdString());
    //index_points++;
}

void motorosudp::MainWindow::on_pushButtonRemovePoint_clicked()
{
  int i = ui->listWidgetPoints->currentRow();
  if(i!=-1)
  {
    for(int j=0;j<points_value->size();j++){
      qnode.delete3Marker(3*j);
      qnode.deleteMarker(3*j+30000);
    }
    points_value->erase(points_value->begin()+i);

    ui->listWidgetPoints->clear();
    for(int j=0;j<points_value->size();j++){
      QString a = "C"+QString::number(j);
      ui->listWidgetPoints->addItem(a);
      qnode.publishPose(points_value->at(j),j*3,0.05);
      qnode.publishText(points_value->at(j),j*3+30000,a.toStdString());
    }

   // points_list->erase(points_list->begin()+i);


  }
}



//void motorosudp::MainWindow::on_pushButtonModifyPoint_clicked()
//{

//  int i = ui->listWidgetPoints->currentRow();
//  if(i!=-1)
//  {
//    DialogPosition a;
//    a.getJoints(points_value->at(i));
//    a.exec();
//    qDebug() << "before export";
//    a.exportJoints(points_value->at(i));
//    qDebug() << "exported";
//    qnode.publishPose(points_value->at(i),i*3,0.05);
//    QString b = "C"+QString::number(i);
//    qnode.publishText(points_value->at(i),i*3+30000,b.toStdString());
//  }


//}

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
  if(!qnode.connectSerial()){
    QMessageBox::critical(this,"Error","Can not connect to serial");

  }
  else{
    ui->labelSerialStatus->setText("Connected");
    isVarPosition=false;
    int pos[6]={0,0,0,0,0,0};
    qnode.sendFirstDataToSerial(ui->spinBoxEncoderType->value(),ui->doubleSpinBoxRatio->value(),pos,false);
  }
}
void motorosudp::MainWindow::SampleHandler(){
  /*timerSample->stop();
  switch(mode){
  case 1:
    if(isVarPosition){
      //for (int i=0;i<6;i++) {
     // current_object_position.at(i)=(*(qnode.getObjectPosition()+i));}
      std::vector<int> temp = current_object_position;
      temp.at(1)+=ui->spinBoxDelayDy->value();
    socket->WriteVarPosition(32,temp);
//    std::cout << "Write Var: " << current_object_position.at(0) << " "
//                                << current_object_position.at(1) << " "
//                                << current_object_position.at(2) << " "
//                                   << current_object_position.at(3) << " "
//                                      << current_object_position.at(4) << " "
//                                      << current_object_position.at(5) << std::endl;
    };
    timerSample->start(14);
    mode=2;
    break;
  case 2:
    if(isVarPosition){
    distance = sqrt(pow(current_positions_int.at(1)-current_object_position.at(1),2)+
               pow(current_positions_int.at(2)-current_object_position.at(2),2));

    //ui->doubleSpinBoxDistance->setValue(distance);
    if(distance<ui->spinBoxPickingDistance->value()) {
      socket->WriteByte(33,1);
      std::cout << "Picking" << std::endl;
      timerSample->start(14);
      qnode.sendFirstDataToSerial(0,0,qnode.getObjectPosition(),false);
      isVarPosition=false;
      isBlock=false;
    }
    else if(current_object_position.at(1)>ui->spinBoxStartY->value()&&isFirstByte)
    {
        socket->WriteByte(32,1);
        timerSample->start(14);
        isFirstByte=false;
      std::cout << "Start tracking" << std::endl;
    }
    else if(current_object_position.at(1)>ui->spinBoxStopY->value()){
      socket->WriteByte(33,1);
      std::cout << "Stop Tracking" << std::endl;
      timerSample->start(14);
      //qnode.sendFirstDataToSerial(0,0,qnode.getObjectPosition(),false);
      isVarPosition=false;
      isBlock=false;
    }
    else{
     // std::cout << "GetPulse" << std::endl;
      socket->GetPulsePosition();
      timerSample->start(2);
    }
    }
    else
    {
    //  std::cout << "GetPulse" << std::endl;
      socket->GetPulsePosition();
      timerSample->start(2);

    }
    mode=1;
    break;
  }
  */
  timerSample->stop();

    switch(mode){
    case 1:
      if(object_position.size()>0){
        //for (int i=0;i<6;i++) {
       // current_object_position.at(i)=(*(qnode.getObjectPosition()+i));}
        std::vector<int> temp;
        temp = object_position.back();
        temp.at(1)+=ui->spinBoxDelayDy->value();
        temp.at(2)+=ui->spinBoxEncoderPickingHeight->value();
//        temp.at(3)=1775001;
//        temp.at(4)=42000;
        socket->WriteVarPosition(32,temp);
  //    std::cout << "Write Var: " << current_object_position.at(0) << " "
  //                                << current_object_position.at(1) << " "
  //                                << current_object_position.at(2) << " "
  //                                   << current_object_position.at(3) << " "
  //                                      << current_object_position.at(4) << " "
  //                                      << current_object_position.at(5) << std::endl;

      };
      timerSample->start(14);
      mode=2;
      break;
    case 2:
      if(object_position.size()>0){
        std::vector<int> temp;
        temp = object_position.back();
      distance = sqrt(pow(current_positions_int.at(1)-temp.at(1),2)+
                 pow(current_positions_int.at(2)-temp.at(2),2));
       ui->lineEditDistance->setText(QString::number(distance));
      //ui->doubleSpinBoxDistance->setValue(distance);
      if(distance<ui->spinBoxPickingDistance->value()){
        socket->WriteByte(33,1);
        std::cout << "Picking " << distance  <<   std::endl;
        timerSample->start(14);
       // qnode.sendFirstDataToSerial(0,0,qnode.getObjectPosition(),false);
        isVarPosition=false;
        object_position.pop_back();
        //isBlock=false;
      }
      else if(object_position.back().at(1)>ui->spinBoxStartY->value()&&isFirstByte)
      {
          socket->WriteByte(32,1);
          timerSample->start(14);
          isFirstByte=false;
        std::cout << "Start tracking at " << object_position.back().at(1)<<  std::endl;
      }
      else if(object_position.back().at(1)>ui->spinBoxStopY->value()){
        socket->WriteByte(33,1);
        std::cout << "Stop Tracking and pick at " << object_position.back().at(1) << std::endl;
        timerSample->start(14);
        //qnode.sendFirstDataToSerial(0,0,qnode.getObjectPosition(),false);
        isVarPosition=false;
        isBlock=false;
        object_position.pop_back();
      }
      else{
       // std::cout << "GetPulse" << std::endl;
        socket->GetPulsePosition();
        timerSample->start(2);
      }
      }
      else
      {
      //  std::cout << "GetPulse" << std::endl;
        socket->GetPulsePosition();
        timerSample->start(2);

      }
      mode=1;
      break;
    }
}

void motorosudp::MainWindow::on_comboBox_currentIndexChanged(int index)
{
    switch(index)
    {
    case 0:
      jogspeed=0.5;
      break;
    case 1:
      jogspeed=1;
      break;
    case 2:
      jogspeed=2;
      break;
    }

}


void motorosudp::MainWindow::SerialDataReceive(){
/*
 // isVarPosition=true;
  int32_t posi[6];
  memcpy(posi,qnode.getObjectPosition(),sizeof(posi));
  if(posi[1]>110000){
    qnode.sendFirstDataToSerial(0,0,posi,false);
    isVarPosition=false;
  }
  memcpy(current_object_position.data(),posi,sizeof(posi));
  ui->lineEditX_3->setText(QString::number(posi[0]));
  ui->lineEditY_3->setText(QString::number(posi[1]));
  ui->lineEditZ_3->setText(QString::number(posi[2]));
  ui->lineEditRX_3->setText(QString::number(posi[3]));
  ui->lineEditRY_3->setText(QString::number(posi[4]));
  ui->lineEditRZ_3->setText(QString::number(posi[5]));
  std::vector<double> temp;
  temp.push_back((double)current_object_position.at(0)/1000000);
  temp.push_back((double)current_object_position.at(1)/1000000);
  temp.push_back((double)current_object_position.at(2)/1000000);
  temp.push_back((double)current_object_position.at(3)/10000*M_PI/180);
  temp.push_back((double)current_object_position.at(4)/10000*M_PI/180);
  temp.push_back((double)current_object_position.at(5)/10000*M_PI/180);
  //std::cout << "Receive from Serial" << std::endl;
  qnode.publishPose(temp,1000000);
  //QString b = "OBJECT";
  //qnode.publishText(points_value->at(i),i*3+30000,b.toStdString());
*/
  int32_t posi[6];
  memcpy(posi,qnode.getObjectPosition(),sizeof(posi));
  double ratio = ui->doubleSpinBoxRatio->value();
  int PPR = ui->spinBoxEncoderType->value();
//  for(int i=0;i<6;i++){
//     std::cout << "POS" << i << " " <<  posi[i] << std::endl;
//  }

  double delta = (posi[1])*ratio/(4*PPR);
  speed=delta/0.005;
 // std::cout << speed << std::endl;
  if(object_position.size()>0){
  for(u_int i=0;i<object_position.size();i++){
    object_position.at(i).at(1)+=delta*1000;
  };
  std::vector<double> temp;
  temp.push_back(double(object_position.back().at(0))/1000000);
  temp.push_back(double(object_position.back().at(1))/1000000);
  temp.push_back(double(object_position.back().at(2))/1000000+0.143);
  temp.push_back(double(object_position.back().at(3))/10000*M_PI/180);
  temp.push_back(double(object_position.back().at(4))/10000*M_PI/180);
  temp.push_back(double(object_position.back().at(5))/10000*M_PI/180);

 ui->lineEditX_3->setText(QString::number(object_position.back().at(0))+" um");
 ui->lineEditY_3->setText(QString::number(object_position.back().at(1))+" um");
 ui->lineEditZ_3->setText(QString::number(object_position.back().at(2))+" um");
 ui->lineEditRX_3->setText(QString::number(object_position.back().at(3))+" um");
 ui->lineEditRY_3->setText(QString::number(object_position.back().at(4))+" um");
 ui->lineEditRZ_3->setText(QString::number(object_position.back().at(5))+" um");

  qnode.publishPose(temp,1000000);
  }
  ui->lineEditSpeed->setText(QString::number(speed));

}

void motorosudp::MainWindow::on_pushButtonStartConveyer_clicked()
{
  if(ui->pushButtonStartConveyer->text()=="START"){
  std::vector<uint16_t> data;
  data.push_back(1);
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,1,1,data);
  ui->pushButtonStartConveyer->setText("STOP");
  ui->labelStatusConveyer->setText("Status: ON");
  }
  else{
    std::vector<uint16_t> data;
    data.push_back(0);
    socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,1,1,data);
    ui->pushButtonStartConveyer->setText("START");
    ui->labelStatusConveyer->setText("Status: OFF");
  }
}


void motorosudp::MainWindow::foundObject(){
/*  if(!isBlock){
  float pose[6];
  int pos[6];
  memcpy(pose,&mOpenCV_videoCapture->x_robot,4);
  memcpy(pose+1,&mOpenCV_videoCapture->y_robot,4);
  memcpy(pose+2,&mOpenCV_videoCapture->z_robot,4);
  memcpy(pose+3,&mOpenCV_videoCapture->Rx,4);
  memcpy(pose+4,&mOpenCV_videoCapture->Ry,4);
  memcpy(pose+5,&mOpenCV_videoCapture->Rz,4);
  for(int i=0;i<3;i++){
     pos[i]=pose[i]*1000;
  }
  pos[2]+=4000;
  for(int i=3;i<6;i++){
     pos[i]=pose[i]*10000;
  }
  memcpy(current_object_position.data(),pos,sizeof(pos));
  if(qnode.COM.isOpen()){
      qnode.sendFirstDataToSerial(ui->spinBoxEncoderType->value(),(ui->doubleSpinBoxRatio->value())*1000,pos,true);
      isVarPosition=true;
      isFirstByte=true;
      std::cout << "CV Found object" << std::endl;
    }
  isBlock=true;
  };*/
  std::cout << "CV Found object" << std::endl;
  float pose[6];
  std::vector<int> pos;
  memcpy(pose,&mOpenCV_videoCapture->x_robot,4);
  memcpy(pose+1,&mOpenCV_videoCapture->y_robot,4);
  memcpy(pose+2,&mOpenCV_videoCapture->z_robot,4);
  memcpy(pose+3,&mOpenCV_videoCapture->Rx,4);
  memcpy(pose+4,&mOpenCV_videoCapture->Ry,4);
  memcpy(pose+5,&mOpenCV_videoCapture->Rz,4);
  pose[2]+=4;
  for(int i=0;i<3;i++){
    pos.push_back(pose[i]*1000);
  }
  //pos[2]+=4000;
  for(int i=3;i<6;i++){
     pos.push_back(pose[i]*10000);
  }
  object_position.push_back(pos);
  isVarPosition=true;
  isFirstByte=true;
}


void motorosudp::MainWindow::on_radioButtonInteractiveMarker_clicked(bool checked)
{
  if(checked)
  {
    qnode.updateInteractiveMarkers(current_joints_radian);
    ui->groupBoxJOG->setEnabled(false);
//    x=current_joints_radian;
  }
  else{
    qnode.deleteInteractiveMarkers();
    clearGlobal();
     ui->groupBoxJOG->setEnabled(true);
  }
}


void motorosudp::MainWindow::on_pushButtonHoming_clicked()
{
  if(isSimulation){
    for(int i=0;i<current_joints_radian.size();i++)
    current_joints_radian.at(i)=0;
    qnode.publishJoint(current_joints_radian);
  }
  else{
    timerSample->stop();
    int a[6] = {0,0,0,0,0,0};
    socket->WritePulse(1,0,1000,a);
    timerSample->start(20);
  }
}



void motorosudp::MainWindow::on_pushButtonServoHoming_clicked()
{
  std::vector<uint16_t> data;
  data.push_back(1);
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,3,1,data);
  data.at(0)=0;
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,3,1,data);
}

void motorosudp::MainWindow::on_pushButtonStartProgram_clicked()
{
  std::vector<uint16_t> data;
  data.push_back(1);
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,2,1,data);
  data.at(0)=0;
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,2,1,data);
}

void motorosudp::MainWindow::on_pushButtonStopProgram_clicked()
{
  std::vector<uint16_t> data;
  data.push_back(1);
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,4,1,data);
  data.at(0)=0;
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,4,1,data);
  data.at(0)=1;
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,5,1,data);
  data.at(0)=0;
  socket->ConnectToPLC(QHostAddress("192.168.1.144"),10001,5,1,data);
}
void motorosudp::MainWindow::fileReceiveDone(){
  QMessageBox::information(this,"Notification","Completely Receive!");
  socket->GetJobFile(ui->lineEditPath->text()+ui->lineEditName->text());
  timerSample->start(15);

}
void motorosudp::MainWindow::fileTransmitDone(){
  QMessageBox::information(this,"Notification","Completely Send!");
  char job[4]={'T','P','T','4'};
  socket->SelectJob(job);
  timerSample->start(300);
}
void motorosudp::MainWindow::fileReceiveError(){
  QMessageBox::critical(this,"Error","Fail! Please check again!");
  timerSample->start(15);

}
void motorosudp::MainWindow::fileTransmitError(){
  QMessageBox::critical(this,"Error","Fail! Please check again!");
  timerSample->start(15);
}
void motorosudp::MainWindow::on_pushButtonGetJobFile_clicked()
{
  timerSample->stop();
  QString a = ui->lineEditName->text();
  std::string b = a.toStdString();
  char jobname[b.size()];
  for(int i=0;i<b.size();i++){
    jobname[i]=b.at(i);
    //std::cout << (u_int8_t) b.at(i) << std::endl;
  }
  socket->FileReceiveCommand(jobname,b.size());
}

bool GetJobFile(QString path, QString &content){
  std::cout << "Function" << std::endl;
  //"/home/tapati/motoman_ws/src/motorosudp/TEST0407.JBI"
  QFile file(path);
  if(file.open(QIODevice::ReadWrite)){
    //file.write(*buffer);
    QTextStream stream(&file);
    content = stream.readAll();
    file.close();
    return true;
  }
  return false;
}
void motorosudp::MainWindow::on_pushButtonOpenJOB_clicked()
{
  //timerSimulation->stop();
  std::cout << "Display1" << std::endl;
  QString b;
    if(!GetJobFile(ui->lineEditPath->text()+ui->lineEditName->text(),b))
    {QMessageBox::critical(this,"Error","Cannot open the file, please check again");}
    else{
    ui->textEditJOB->setPlainText(b);}
}

void motorosudp::MainWindow::on_pushButtonSendJOB_clicked()
{
   timerSample->stop();
//  QString a = ui->textEditJOB->toPlainText();
//  socket->tx_file_buffer->resize(sizeof(a));
//  memcpy(socket->tx_file_buffer->data(),a.data(),sizeof(a));
   socket->JobFile2ByteArray("/home/tapati/motoman_ws/src/motorosudp/TPT4.JBI");
  char jobname[8] = {'T','P','T','4','.','J','B','I'};
  socket->FileDeleteCommand(jobname,8);
  QTimer::singleShot(100, this, SLOT(deleteFile()));

}
void motorosudp::MainWindow::deleteFile(){
  char jobname[8] = {'T','P','T','4','.','J','B','I'};
  socket->FileTransmitCommand(jobname,8);
}

void motorosudp::MainWindow::on_pushButtonServo_clicked()
{
  timerSample->stop();
  if(ui->pushButtonServo->text()=="SERVO ON")
  {
    ui->pushButtonServo->setText("SERVO OFF");
    socket->TurnOnServo();
    ui->labelStatusServo->setText("Status: ON");
  }
  else{
    ui->pushButtonServo->setText("SERVO ON");
    socket->TurnOffServo();
    ui->labelStatusServo->setText("Status: OFF");
  }
  timerSample->start(100);
}


void motorosudp::MainWindow::on_checkBoxTrajectory_stateChanged(int arg1)
{
  if(ui->checkBoxTrajectory->checkState() != Qt::CheckState::Checked)
        {
          qnode.deleteAllMarker();
          for(int j=0;j<points_value->size();j++){
            QString a = "C"+QString::number(j);
            qnode.publishPose(points_value->at(j),j*3,0.05);
            qnode.publishText(points_value->at(j),j*3+30000,a.toStdString());
          }
          //do something
        };
}
void motorosudp::MainWindow::catesianCaculate(std::vector<double> tem){
  std::vector<double> temp;
  if(qnode.getJointsPosition(tem,true,temp)){
    current_joints_radian=temp;
  }
  else if(qnode.getJointsPosition(tem,false,temp)){
    current_joints_radian=temp;
  }
  else{
    QMessageBox::critical(this,"Error","Outside the joint limit!");
  }
}

void motorosudp::MainWindow::on_pushButtonStartMasterJOB_clicked()
{
  timerSample->stop();
  socket->StartJob();
  timerSample->start(300);
}
const double PULSE_PER_DEGREE_S = 34816/30;
const double PULSE_PER_DEGREE_L = 102400/90;
const double PULSE_PER_DEGREE_U = 51200/90;
const double PULSE_PER_DEGREE_RBT = 10204/30;
std::vector<int> Joint2Pulse(std::vector<double> joint)
{
  std::vector<int> t;

    t.push_back(int32_t(PULSE_PER_DEGREE_S*joint.at(0)*180/M_PI));
    t.push_back(int32_t(PULSE_PER_DEGREE_L*joint.at(1)*180/M_PI));
    t.push_back(int32_t(PULSE_PER_DEGREE_U*joint.at(2)*180/M_PI));
    t.push_back(int32_t(PULSE_PER_DEGREE_RBT*joint.at(3)*180/M_PI));
    t.push_back(int32_t(PULSE_PER_DEGREE_RBT*joint.at(4)*180/M_PI));
    t.push_back(int32_t(PULSE_PER_DEGREE_RBT*joint.at(5)*180/M_PI));
    return t;
}
int countDigit(long long n)
{
    int count = 0;
    while (n != 0) {
        n = n / 10;
        ++count;
    }
    return count;
}
void motorosudp::MainWindow::on_pushButtonGeneratePath_clicked()
{
    std::string header ="/JOB\n//NAME TEST0407\n//POS\n///NPOS 3,0,0,0,0,0\n///TOOL 0\n///POSTYPE PULSE\n///PULSE\n";
    points_list->resize(points_value->size());
    std::string display,display_;
    for(int i=0;i<points_value->size();i++){
      std::string pos;
      pos="C";
      for(int j=5;j>countDigit(j);j--){
        pos+="0";
      }
      pos+=std::to_string(i);
      display_+="MOVJ ";
      display_+=pos;
      display_+=" ";
      display_+="VJ=1.00\n";
      pos+="=";
      pos+=std::to_string(Joint2Pulse(points_value->at(i)).at(0));
      pos+=",";
      pos+=std::to_string(Joint2Pulse(points_value->at(i)).at(1));
      pos+=",";
      pos+=std::to_string(Joint2Pulse(points_value->at(i)).at(2));
      pos+=",";
      pos+=std::to_string(Joint2Pulse(points_value->at(i)).at(3));
      pos+=",";
      pos+=std::to_string(Joint2Pulse(points_value->at(i)).at(4));
      pos+=",";
      pos+=std::to_string(Joint2Pulse(points_value->at(i)).at(5));
      pos+="\n";
      display+=pos;
    }
    std::string header_ ="//INST\n///DATE 2018/07/05 16:46\n///ATTR SC,RW\n///GROUP1 RB1\nNOP\n";
//    QString temp;
//    temp.fromStdString(header);
    ui->textEditJOB->setText(QString::fromStdString(header+display+header_+display_+"END"));

}

void motorosudp::MainWindow::on_pushButtonCheck_clicked()
{
  int i = ui->listWidgetPoints->currentRow();
  if(i!=-1)
  {
    current_joints_radian=points_value->at(i);
  }
}
