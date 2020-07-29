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
  double current_position[6];
  std::vector <double> current_joints_radian;
  std::vector <double> current_positions;
  int32_t current_pulse[6];
  void GetPositionandMove(u_int16_t type, u_int32_t class_speed);
  void DefaultRealControl();
  void DefaultSimulation();
public Q_SLOTS:
  void on_pushButtonConnect_clicked();
  void RequestPosition();
  void DataReceiveHandler();
  void on_checkBoxReadRobotState_stateChanged(int arg1);
  void on_pushButtonSERVO_clicked();
  void on_pushButtonHOME_clicked();
  void on_comboBoxCoordinate_currentIndexChanged(int index);
  void on_checkBoxDrawEndPoints_stateChanged(int arg1);
  void on_pushButtonMOVJ_clicked();
  void on_pushButtonMOVL_clicked();
  void on_pushButton_2_clicked();
  void on_comboBoxMode_activated(int index);
  void on_pushButton_clicked();
  //void on_pushButtonAdd_1_clicked(bool checked);

private:
  Ui::MainWindowDesign* ui;
  QNode qnode;
  QTimer* timer;
  QTimer* timer_;
  bool isCartesianCoordinate;
  bool isReadRobotState;
  bool isSimulation;

};

}  // namespace motorosudp

#endif // motorosudp_MAIN_WINDOW_H
