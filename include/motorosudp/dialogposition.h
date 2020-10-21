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
  void getJoints(std::vector<double> joints);
  void exportJoints(std::vector<double> &joints);



private Q_SLOTS:
  void on_buttonBox_accepted();
  void on_comboBoxCoordinate_currentIndexChanged(const QString &arg1);

private:
  Ui::DialogPosition *ui;
  std::vector<double>joints,position;

};

#endif // DIALOGPOSITION_H
