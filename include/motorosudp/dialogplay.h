#ifndef DIALOGPLAY_H
#define DIALOGPLAY_H

#include <QDialog>

namespace Ui {
class DialogPlay;
}

class DialogPlay : public QDialog
{
  Q_OBJECT

public:
  explicit DialogPlay(QWidget *parent = nullptr);
  ~DialogPlay();

private Q_SLOTS:
  void on_pushButton_clicked();

private:
  Ui::DialogPlay *ui;
};

#endif // DIALOGPLAY_H
