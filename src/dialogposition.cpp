#include "../include/motorosudp/dialogposition.h"
#include "ui_dialogposition.h"
#include "QDebug"

DialogPosition::DialogPosition(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::DialogPosition)
{
  ui->setupUi(this);
  //connect(this,SIGNAL(accepted()),this,SLOT(hello()));


}

DialogPosition::~DialogPosition()
{
  delete ui;
}
void DialogPosition::getJoints(std::vector<double> joints){
  this->joints=joints;
  ui->doubleSpinBox1->setValue(joints.at(0)*180/M_PI);
  ui->doubleSpinBox2->setValue(joints.at(1)*180/M_PI);
  ui->doubleSpinBox3->setValue(joints.at(2)*180/M_PI);
  ui->doubleSpinBox4->setValue(joints.at(3)*180/M_PI);
  ui->doubleSpinBox5->setValue(joints.at(4)*180/M_PI);
  ui->doubleSpinBox6->setValue(joints.at(5)*180/M_PI);
}
void DialogPosition::on_comboBoxCoordinate_currentIndexChanged(const QString &arg1)
{

}
void DialogPosition::exportJoints(std::vector<double> &j){
  j=this->joints;
}

void DialogPosition::on_buttonBox_accepted()
{
  qDebug() << "Out";
  if(ui->comboBoxCoordinate->currentIndex()==0){
      this->joints.at(0)=(ui->doubleSpinBox1->value()*M_PI/180);
     this->joints.at(1)=(ui->doubleSpinBox2->value()*M_PI/180);
     this->joints.at(2)=(ui->doubleSpinBox3->value()*M_PI/180);
     this->joints.at(3)=(ui->doubleSpinBox4->value()*M_PI/180);
      this->joints.at(4)=(ui->doubleSpinBox5->value()*M_PI/180);
     this->joints.at(5)=(ui->doubleSpinBox6->value()*M_PI/180);}
}
