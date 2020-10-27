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
#include <time.h>
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
extern std::vector<double> x;
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
 // double current_position[6];
  std::vector <double> current_joints_radian;
  std::vector <int> current_object_position;
  std::vector<std::vector <int>> object_position;
  std::vector <int> current_positions_int;
  std::vector <double> current_positions;
  std::vector <double> pre_current_joints_radian;
  double speed;
  int distance;
  //int32_t current_pulse[6];
  void GetPositionandMove(u_int16_t type, u_int32_t class_speed);
  void DefaultRealControl();
  void DefaultSimulation();
  void InitObjectPosition();
  void catesianCaculate(std::vector<double>tem);

public Q_SLOTS:
  void on_pushButtonConnect_clicked();
  void RequestPosition();
  void DataReceiveHandler();
  void SampleHandler();
  void TimeoutHandler();
  void SerialDataReceive();
  void foundObject();
  void fileReceiveDone();
  void fileTransmitDone();
  void fileReceiveError();
  void fileTransmitError();
  void deleteFile();
  void on_comboBoxCoordinate_currentIndexChanged(int index);





  void sendDataPosition();
  //void on_pushButtonAdd_1_clicked(bool checked);
  void on_pushButtonAddPoint_clicked();
  void on_pushButtonRemovePoint_clicked();
//  void on_pushButtonModifyPoint_clicked();
  void on_CameraOff_Button_clicked();
  void on_CameraOn_Button_clicked();
  void on_StreamOption_comboBox_currentIndexChanged(const QString &arg1);
  void on_Background_Button_clicked();
  void on_Mask_Button_clicked();
  void on_Size_Button_clicked();
  void on_ChooseObject_Button_clicked();
  void handleButton() ;
  void on_pushButtonConnectSerial_clicked();
  void on_comboBox_currentIndexChanged(int index);
  void on_pushButtonStartConveyer_clicked();
  void on_radioButtonInteractiveMarker_clicked(bool checked);
  void on_pushButtonHoming_clicked();
  void on_pushButtonServoHoming_clicked();
  void on_pushButtonStartProgram_clicked();
  void on_pushButtonStopProgram_clicked();
  void on_pushButtonGetJobFile_clicked();
  void on_pushButtonOpenJOB_clicked();
  void on_pushButtonSendJOB_clicked();
  void on_pushButtonServo_clicked();
  void on_checkBoxTrajectory_stateChanged(int arg1);
  void on_pushButtonStartMasterJOB_clicked();
  void on_pushButtonGeneratePath_clicked();
  void on_pushButtonCheck_clicked();

private:
  Ui::MainWindowDesign* ui;
  QNode qnode;
  QTimer* timerReal;
  QTimer* timerSimulation;
  QTimer* timerSample;
  QTimer* timerTimeout;
  bool isCartesianCoordinate;
  bool isSimulation;
  std::vector <QString>* points_list;
  std::vector <QString>* teaching_step;
  std::vector <std::vector<double>> *points_value;
  int index_points;
  int index_step;
  //DialogPosition* dialogPosition;
  double jogspeed;
  bool isVarPosition;
  Capture *mOpenCV_videoCapture;
  bool Find_Mask_Ready = 1;
  Resizable_rubber_band *band;
  uint64 time;
  bool isFirstVar;
  bool isFirstByte;
  uint mode;
  bool isBlock;
  int id;
  bool isPublishMarker;

};



}  // namespace motorosudp

#endif // motorosudp_MAIN_WINDOW_H
