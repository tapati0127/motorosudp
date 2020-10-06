#ifndef DIALOGPOSITION_H
#define DIALOGPOSITION_H

#include <QDialog>

namespace Ui {
class DialogPosition;
}

class DialogPosition : public QDialog
{
  Q_OBJECT

public:
  explicit DialogPosition(QWidget *parent = nullptr);
  ~DialogPosition();

private Q_SLOT:
  void on_buttonBox_accepted();

private:
  Ui::DialogPosition *ui;
};

#endif // DIALOGPOSITION_H
