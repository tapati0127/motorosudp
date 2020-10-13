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
  void publishPosition();
  void deleteMarker();
  std::vector<double> getROSPosition(std::vector<double> joints);
  bool getJointsPosition(std::vector<double> pos,bool upper_lower,std::vector<double> &joints);
  void updateInteractiveMarkers(bool visible);
  bool connectSerial();
  bool sendFirstDataToSerial(int encodertype, double ratio);
  double getVelocity();
Q_SIGNALS:
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  ros::Publisher joint_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher interactive_publisher;
  ros::Publisher position_publisher;
  ros::ServiceClient client;
  moveit_msgs::DisplayRobotState robot_state;
  visualization_msgs::Marker marker;
  robot_model_loader::RobotModelLoader* motomini_model_loader;
  robot_model::RobotModelPtr motomini_model_ptr;
  robot_state::RobotState* motomini_state_ptr;
  const moveit::core::JointModelGroup* motomini_model_group_ptr;
  geometry_msgs::Pose position;
  interactive_markers::InteractiveMarkerServer* server;
  serial::Serial COM;
  double velocity;
//  interactive_markers::InteractiveMarkerServer server;



};

}  // namespace motorosudp

#endif /* motorosudp_QNODE_HPP_ */
