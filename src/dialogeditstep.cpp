#include "../include/motorosudp/dialogeditstep.h"
#include "ui_dialogeditstep.h"

DialogEditStep::DialogEditStep(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::DialogEditStep)
{
  ui->setupUi(this);
}

DialogEditStep::~DialogEditStep()
{
  delete ui;
}

void DialogEditStep::on_buttonBox_accepted()
{

}
