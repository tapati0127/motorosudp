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
  connect(timer,SIGNAL(timeout()),this,SLOT(RequestPosition()));
  ui->setupUi(this);
  qnode.init();
  QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

}

MainWindow::~MainWindow() {
  delete ui;
  delete timer;
}


}  // namespace motorosudp
void motorosudp::MainWindow::RequestPosition()
{
    socket->GetPulsePosition();
}
void motorosudp::MainWindow::DataReceiveHandler()
{
    socket->ReceiveData();
    QByteArray buffer;
    buffer = *(socket->Get_rx_buffer());
    if(socket->GetReceiveType(buffer)==socket->GET_PULSE){
        //std::cout << (socket->ByteArray2Hex(buffer)).toStdString() << std::endl;
        qnode.publishJoint(socket->ByteArray2Joint(&buffer));
        if(ui->checkBoxReadRobotState_2->checkState()== Qt::CheckState::Checked)
        {
          qnode.publishMarker(socket->ByteArray2Joint(&buffer));
        }
    }
}
void motorosudp::MainWindow::on_pushButtonConnect_clicked()
{
  if(ui->pushButtonConnect->text()=="CONNECT")
  {
    ui->pushButtonConnect->setText("DISCONNECT");
    ui->groupBoxVisual->setEnabled(true);
    ui->checkBoxReadRobotState->setEnabled(true);
    ui->checkBoxReadRobotState_2->setEnabled(true);
    socket = new MotoUDP(QHostAddress(ui->comboBoxIPadress->currentText()),ui->textEditPort->toPlainText().toInt());
    socket->ConnectMotoman();
    connect(socket->client,SIGNAL(readyRead()),this,SLOT(DataReceiveHandler()));
  }
  else {
    timer->stop();
    ui->pushButtonConnect->setText("CONNECT");
    ui->groupBoxVisual->setEnabled(false);
    ui->checkBoxReadRobotState->setEnabled(false);
    ui->checkBoxReadRobotState_2->setEnabled(false);
    socket->CloseMotoman();
    delete socket;

  }
}

void motorosudp::MainWindow::on_checkBoxReadRobotState_stateChanged(int arg1)
{
  if(arg1==2){
    timer->start(20);
  }
  else {
    timer->stop();  }
}

void motorosudp::MainWindow::on_pushButtonClearMarker_clicked()
{
  qnode.deleteMarker();
}
