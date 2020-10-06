#include "../include/motorosudp/dialogplay.h"
#include "ui_dialogplay.h"

DialogPlay::DialogPlay(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::DialogPlay)
{
  ui->setupUi(this);
}

DialogPlay::~DialogPlay()
{
  delete ui;
}

void DialogPlay::on_pushButton_clicked()
{

}
