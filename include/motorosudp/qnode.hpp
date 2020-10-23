/**
 * @file /include/motorosudp/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef motorosudp_QNODE_HPP_
#define motorosudp_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <interactive_markers/interactive_marker_server.h>
#include <serial/serial.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace motorosudp {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
  geometry_msgs::Pose pose;
	QNode(int argc, char** argv );
	virtual ~QNode();
  bool init();
  void run();
  void publishJoint(std::vector<double> joints);
  void publishMarker(std::vector<double> joints);
  void publishPose(std::vector<double> joints,int id,double lenght);
  void publishPose(std::vector<double> pos,int id);
  void delete3Marker(int id);
  void deleteMarker(int id);
  void publishText(std::vector<double> joints,int id,std::string text);
  void publishPosition();
  void deleteAllMarker();
  std::vector<double> getROSPosition(std::vector<double> joints);
  bool getJointsPosition(std::vector<double> pos,bool upper_lower,std::vector<double> &joints);
  void updateInteractiveMarkers(std::vector<double> joints);
  void deleteInteractiveMarkers();
  bool connectSerial();
  bool sendFirstDataToSerial(int encodertype, double ratio,int32_t* pos,uint enable);
  int32_t* getObjectPosition();
  bool jointLimit(std::vector<double> joints);
  serial::Serial COM;

Q_SIGNALS:
  void rosShutdown();
  void dataReceive();
  void interactiveSignal(std::vector<double>pos);

private:
	int init_argc;
	char** init_argv;
  ros::Publisher joint_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher interactive_publisher;
  ros::Publisher position_publisher;
  ros::Publisher pose_publisher;
  ros::ServiceClient client;
  moveit_msgs::DisplayRobotState robot_state;
  visualization_msgs::Marker marker;
  robot_model_loader::RobotModelLoader* motomini_model_loader;
  robot_model::RobotModelPtr motomini_model_ptr;
  robot_state::RobotState* motomini_state_ptr;
  const moveit::core::JointModelGroup* motomini_model_group_ptr;
  geometry_msgs::Pose position;
  interactive_markers::InteractiveMarkerServer* server;

  int32_t object_position[6];
  std::vector<double> joint_limit_up;
  std::vector<double> joint_limit_down;

//  interactive_markers::InteractiveMarkerServer server;



};

}  // namespace motorosudp

#endif /* motorosudp_QNODE_HPP_ */
