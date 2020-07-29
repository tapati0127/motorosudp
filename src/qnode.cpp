/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/motorosudp/qnode.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace motorosudp {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"motorosudp");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

  //Use RobotModelLoader to look up the robot description
  motomini_model_loader = new robot_model_loader::RobotModelLoader ("robot_description");
  //Construct RobotModel from RobotModelLoader
  motomini_model_ptr = motomini_model_loader->getModel();
  //Construct RobotState class from RobotModel
  motomini_state_ptr = new robot_state::RobotState (motomini_model_ptr);

	// Add your ros communications here.
  joint_publisher = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1);
  marker_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  while (joint_publisher.getNumSubscribers()<1) {
    ;//Wait
  }
  robot_state.state.is_diff = 1;
  robot_state.state.joint_state.header.frame_id = "base_link";
  robot_state.state.joint_state.name = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();

  marker.type = visualization_msgs::Marker::SPHERE;


  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //vis_pub.publish( marker );
	start();
  ROS_INFO("Thread for ROS is started.");
	return true;
}



void QNode::publishJoint(std::vector<double> joints) {
  if ( ros::ok() ) {
    robot_state.state.joint_state.position = joints;
    joint_publisher.publish(robot_state);
	}
  else {
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

}

void QNode::publishMarker(std::vector<double> joints)
{
  motomini_state_ptr->setJointGroupPositions ("motomini_arm", joints);
  tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("tool0"),pose);
  marker.id += 1;
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;
  marker.action = visualization_msgs::Marker::ADD;
  marker_publisher.publish(marker);
  //ROS_INFO("Publisher marker:");
  //std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
}

void QNode::deleteMarker()
{
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_publisher.publish(marker);
  ROS_INFO("Deleted all marker!!!");
}
std::vector<double> QNode::getROSPosition(std::vector<double> joints)
{
  std::vector<double> pos;
  motomini_state_ptr->setJointGroupPositions ("motomini_arm", joints);
  geometry_msgs::Pose pose2;
  tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("tool0"),pose);
  tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("link_1_s"),pose2);
  tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r,p,y);
  pos.push_back(pose.position.x);
  pos.push_back(pose.position.y);
  pos.push_back(pose.position.z-pose2.position.z);
  //pos.push_back(pose.position.z);
  pos.push_back(r);
  pos.push_back(p);
  pos.push_back(y);
 // std::cout << pos.at(0) << " " << pos.at(1) << " " << pos.at(2) << " " << pos.at(3) << " " << pos.at(4) << " " << pos.at(5) << std::endl;
  return pos;
}
}  // namespace motorosudp
