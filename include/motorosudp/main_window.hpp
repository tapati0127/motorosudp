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
#include "capture.h"
#include "dialogposition.h"

#include <QTimer>
#include <QRubberBand>
#include <QSizeGrip>
#include <QMouseEvent>
#include <QHBoxLayout>

class Capture;
//################Rubber_band##################################//
class Resizable_rubber_band : public QWidget {
public:
    Resizable_rubber_band(QWidget* parent = 0);

private:
    QRubberBand* rubberband;
    QPoint lastPoint, newPoint;
    bool move_rubberband;
    QPoint rubberband_offset;
    void resizeEvent(QResizeEvent *);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

};
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
  void InitObjectPosition();



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
  void on_comboBoxMode_activated(int index);
  void on_pushButton_clicked();
  void sendDataPosition();
  //void on_pushButtonAdd_1_clicked(bool checked);
  void on_pushButton_3_clicked();
  void on_pushButtonMOVL_2_clicked();
  void on_pushButton_4_clicked();
  void on_pushButton_6_clicked();
  void on_pushButton_5_clicked();
  void on_pushButton_7_clicked();
  void on_pushButton_8_clicked();
  void on_pushButton_9_clicked();
  void on_pushButton_10_clicked();
  void on_pushButton_11_clicked();
  void on_pushButtonAddPoint_clicked();
  void on_pushButtonRemovePoint_clicked();
  void on_pushButtonCheckPoint_clicked();
  void on_pushButtonModifyPoint_clicked();
  void on_CameraOff_Button_clicked();
  void on_CameraOn_Button_clicked();
  void on_StreamOption_comboBox_currentIndexChanged(const QString &arg1);
  void on_Background_Button_clicked();
  void on_Mask_Button_clicked();
  void on_Size_Button_clicked();
  void on_ChooseObject_Button_clicked();
  void handleButton() ;

private:
  Ui::MainWindowDesign* ui;
  QNode qnode;
  QTimer* timerReal;
  QTimer* timerSimulation;
  bool isCartesianCoordinate;
  bool isSimulation;
  std::vector <QString>* points_list;
  std::vector <QString>* teaching_step;
  std::vector <std::vector<double>> *points_value;
  int index_points;
  int index_step;
  DialogPosition* dialogPosition;

  Capture *mOpenCV_videoCapture;
  bool Find_Mask_Ready = 1;
  Resizable_rubber_band *band;




};



}  // namespace motorosudp

#endif // motorosudp_MAIN_WINDOW_H
