/**
 * @file /include/motorosudp/main_window.hpp
 *
 * @brief Qt based gui for motorosudp.
 *
 * @date November 2010
 **/
#ifndef motorosudp_MAIN_WINDOW_H
#define motorosudp_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "motoudp.h"
#include <QTimer>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace motorosudp {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
  MotoUDP* socket;

public Q_SLOTS:
  void on_pushButtonConnect_clicked();
  void RequestPosition();
  void DataReceiveHandler();
  void on_checkBoxReadRobotState_stateChanged(int arg1);
  void on_pushButtonClearMarker_clicked();

private:
  Ui::MainWindowDesign* ui;
  QNode qnode;
  QTimer* timer;
};

}  // namespace motorosudp

#endif // motorosudp_MAIN_WINDOW_H
