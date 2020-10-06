#ifndef DIALOGEDITSTEP_H
#define DIALOGEDITSTEP_H

#include <QDialog>

namespace Ui {
class DialogEditStep;
}

class DialogEditStep : public QDialog
{
  Q_OBJECT

public:
  explicit DialogEditStep(QWidget *parent = nullptr);
  ~DialogEditStep();

private Q_SLOT:
  void on_buttonBox_accepted();

private:
  Ui::DialogEditStep *ui;
};

#endif // DIALOGEDITSTEP_H
