#include "../include/motorosudp/dialogposition.h"
#include "ui_dialogposition.h"

DialogPosition::DialogPosition(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::DialogPosition)
{
  ui->setupUi(this);
}

DialogPosition::~DialogPosition()
{
  delete ui;
}

void DialogPosition::on_buttonBox_accepted()
{

}
