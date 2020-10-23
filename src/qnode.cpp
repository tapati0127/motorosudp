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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_state/conversions.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <serial/serial.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/
extern std::vector<double> x;
namespace motorosudp {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
  init_argv(argv)//,
  //server("simple_marker")
  {
  //interactive_markers::InteractiveMarkerServer server("simple_marker");
}

QNode::~QNode() {
  if(COM.isOpen()){
    int t[6]={0,0,0,0,0,0};
    sendFirstDataToSerial(0,0,t,0);
    COM.close();}
   if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
bool QNode::getJointsPosition(std::vector<double> pos,bool upper_lower,std::vector<double> &joints)
{
  joints.resize(6);
  Eigen::AngleAxisd rollAngle(pos.at(5), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(pos.at(4), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pos.at(3), Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
  Eigen::Matrix3d R = q.matrix();
//  std::cout << R << std::endl;
//  std::cout << R(2,2) << std::endl;
  double xc,yc,zc;
  xc = pos.at(0)-0.04*R(0,2);
  yc = pos.at(1)-0.04*R(1,2);
  zc = pos.at(2)-0.04*R(2,2);
  double temp1 = (zc-0.103)*(zc-0.103)-0.165*0.165*2;
  double temp2 = 2*0.165*0.165;

//  double temp[2];
//  temp[0]= sqrt(xc*xc+yc*yc)-0.02;
//  temp[1]= -sqrt(xc*xc+yc*yc)-0.02;
  double temp;
  double c3,s3;
  double c2,s2;
  double temp3;
  double temp4;
  double temp5;
  double theta[6];
  temp= sqrt(xc*xc+yc*yc)-0.02;
  c3 = (temp*temp+temp1)/temp2;
  temp3=0.165*0.165*2+2*0.165*0.165*c3;
  temp4=0.165+0.165*c3;
  joints.resize(6);
  theta[0]=atan2(yc,xc);
  if(upper_lower){
    if(c3>=-1&&c3<=1)
    {
      s3=-sqrt(1-c3*c3);
      theta[2]=atan2(s3,c3);
      temp5=0.165*s3;
      c2=(temp*temp4+(zc-0.103)*temp5)/temp3;
      s2=(-temp*temp5+(zc-0.103)*temp4)/temp3;
      if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
      {
        theta[1]=atan2(s2,c2);
      }
      else {
        //std::cout << "Cannot find IK" << std::endl;
        return false;
      }
    }
    else{
      //std::cout << "Cannot find IK" << std::endl;
      return false;
    }
  }
  else{
    if(c3>=-1&&c3<=1)
    {
      s3=sqrt(1-c3*c3);
      theta[2]=atan2(s3,c3);
      temp5=0.165*s3;
      c2=(temp*temp4+(zc-0.103)*temp5)/temp3;
      s2=(-temp*temp5+(zc-0.103)*temp4)/temp3;
      if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
      {
        theta[1]=atan2(s2,c2);
      }
      else {
        //std::cout << "Cannot find IK" << std::endl;
        return false;
      }
    }
    else{
      //std::cout << "Cannot find IK" << std::endl;
      return false;
    }
   }

  Eigen::Matrix3d R_;
  double s23 = sin(theta[1] + theta[2]);
  double c23 = cos(theta[1] + theta[2]);
  double c1 = cos(theta[0]);
  double s1 = sin(theta[0]);
  R_(0,0)=-s23*c1;
  R_(0,1)=s1;
  R_(0,2)=c23*c1;

  R_(1,0)=-s23*s1;
  R_(1,1)=-c1;
  R_(1,2)=c23*s1;

  R_(2,0)=c23;
  R_(2,1)=0;
  R_(2,2)=s23;

  R = R_.transpose()*R;
  theta[3] = atan2(-R(1,2),-R(0,2));
  theta[4] = atan2(-sqrt(R(0,2)*R(0,2)+R(1,2)*R(1,2)),R(2,2));
  theta[5] = atan2(-R(2,1),+R(2,0));
//    double theta4 = atan2(R(1,2),R(0,2));
//    double theta5 = atan2(sqrt(R(0,2)*R(0,2)+R(1,2)*R(1,2)),R(2,2));
//    double theta6 = atan2(R(2,1),-R(2,0));
//  std::cout << "KQ" << std::endl;
//  std::cout << theta[0]*180/M_PI << std::endl;
//  std::cout << -(theta[1]-M_PI/2)*180/M_PI << std::endl;
//  std::cout << (theta[2]+M_PI/2)*180/M_PI << std::endl;
//  std::cout << -theta[3]*180/M_PI << std::endl;
//  std::cout << (theta[4]+M_PI/2)*180/M_PI << std::endl;
//  std::cout << -theta[5]*180/M_PI << std::endl;
  joints.at(0)=theta[0];
  joints.at(1)=-(theta[1]-M_PI/2);
  joints.at(2) = (theta[2]+M_PI/2);
  joints.at(3)=-theta[3];
  joints.at(4)=(theta[4]+M_PI/2);
  joints.at(5) = -theta[5];
  if(jointLimit(joints)){
    return true;
  }
  else{
    return false;
  }

}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
//  std::cout << "Feedback" << std::endl;
  tf::Quaternion q(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r,p,y);
  std::vector<double> pos,joints;
  pos.push_back(feedback->pose.position.x);
  pos.push_back(feedback->pose.position.y);
  pos.push_back(feedback->pose.position.z);
  pos.push_back(r);
  pos.push_back(p);
  pos.push_back(y);
  x=pos;
  //std::cout << x.size() << std::endl;

  //std::cout << (motorosudp::x).at(2) << std::endl;
}
void QNode::updateInteractiveMarkers(std::vector<double> joints){
  // create an interactive marker for our server
  server=new interactive_markers::InteractiveMarkerServer("interactive_markers");
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "";
  motomini_state_ptr->setJointGroupPositions ("motomini_arm", joints);
  tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("tool0"),int_marker.pose);
  int_marker.scale = 0.08;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  //box_control.markers.push_back( box_marker );

  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, box_control.orientation);
  box_control.name = "rotate_x";
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(box_control);
   box_control.name = "move_x";
   box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back( box_control);

  orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, box_control.orientation);
   box_control.name = "rotate_z";
   box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back( box_control);
  box_control.name = "move_z";
   box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back( box_control);

  orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, box_control.orientation);
  box_control.name = "rotate_y";
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(box_control);
  box_control.name = "move_y";
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(box_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->clear();
  server->insert(int_marker, processFeedback);

  // 'commit' changes and send to all clients
  server->applyChanges();

}
void QNode::deleteInteractiveMarkers(){
  delete server;
}
bool QNode::init() {
  joint_limit_up.push_back(170*M_PI/180);
  joint_limit_up.push_back(90*M_PI/180);
  joint_limit_up.push_back(90*M_PI/180);
  joint_limit_up.push_back(140*M_PI/180);
  joint_limit_up.push_back(210*M_PI/180);
  joint_limit_up.push_back(360*M_PI/180);
  joint_limit_down.push_back(-170*M_PI/180);
  joint_limit_down.push_back(-85*M_PI/180);
  joint_limit_down.push_back(-50*M_PI/180);
  joint_limit_down.push_back(-140*M_PI/180);
  joint_limit_down.push_back(-30*M_PI/180);
  joint_limit_down.push_back(-360*M_PI/180);
  std::cout << "Start" << std::endl;
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

  static const std::string PLANNING_GROUP = "motomini_arm";
//  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  motomini_model_group_ptr = motomini_state_ptr->getJointModelGroup(PLANNING_GROUP);

	// Add your ros communications here.
  joint_publisher = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1);
  marker_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  position_publisher = n.advertise<geometry_msgs::Pose>( "motomini_position", 1 );
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>( "pose_topic", 1 );
  //interactive_publisher = n.advertise<visualization_msgs::InteractiveMarker>( "interactive_marker", 0 );
//  interactive_markers::InteractiveMarkerServer server("motorosudp");


  // create an interactive marker for our server



  while (joint_publisher.getNumSubscribers()<1) {
    ;//Wait
  }
//  client = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
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
  marker.id=-1;
  //vis_pub.publish( marker );



  start();
  ROS_INFO("Thread for ROS is started.");
  //ros::spin();
  return true;
}
void QNode::run(){
// std::cout << "Start" << std::endl;
  while(ros::ok())
  {
      if(COM.available())
      {
          int size = COM.available();
          uint8_t buffer[24];
          COM.read(buffer,size);
          memcpy(object_position,buffer,24);
          Q_EMIT dataReceive();
//          for (int i=0;i<6;i++) {
//            std::cout << object_position[i] << " ";
//          }
//            std::cout << std::endl;
      }
      ros::spinOnce();
  }
  //ros::spin();
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
  if(marker.id==-2147483648) marker.id=-1;
  marker.id--;
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;
  marker.action = visualization_msgs::Marker::ADD;
  marker_publisher.publish(marker);
  //ROS_INFO("Publisher marker:");
  //std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
}

void QNode::deleteAllMarker()
{
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_publisher.publish(marker);
//  for(int i=-1;i>=marker.id;i--){

//    std::cout << i << std::endl;
//  }
  marker.id=-1;
  ROS_INFO("Deleted all marker!!!");
}
std::vector<double> QNode::getROSPosition(std::vector<double> joints)
{
  std::vector<double> pos;
  motomini_state_ptr->setJointGroupPositions ("motomini_arm", joints);
  geometry_msgs::Pose pose2;
//  std::cout << motomini_state_ptr->getGlobalLinkTransform("tool0"). << std::endl;
  tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("tool0"),pose2);
  //tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("link_1_s"),pose2);
  tf::Quaternion q(pose2.orientation.x,pose2.orientation.y,pose2.orientation.z,pose2.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r,p,y);
  pos.push_back(pose2.position.x);
  pos.push_back(pose2.position.y);
  //pos.push_back(pose.position.z-pose2.position.z);
  pos.push_back(pose2.position.z);
  pos.push_back(r);
  pos.push_back(p);
  pos.push_back(y);

 // std::cout << pos.at(0) << " " << pos.at(1) << " " << pos.at(2) << " " << pos.at(3) << " " << pos.at(4) << " " << pos.at(5) << std::endl;
  return pos;
}
void QNode::publishPosition(){
  position_publisher.publish(position);
}

bool QNode::connectSerial(){

  if(!COM.isOpen()){
    try
      {
      COM.setBaudrate(115200);
      COM.setPort("/dev/ttyUSB0");
      serial::Timeout to = serial::Timeout::simpleTimeout(10);
      COM.setTimeout(to);
      COM.open();
      }
      catch (std::exception &e)
      {
        std::cout << e.what() << '\n';
      }
  }
  return (COM.isOpen());
}
bool QNode::sendFirstDataToSerial(int encodertype, double ratio,int32_t* pos,uint enable)
{
  uint8_t buffer[40];

  mempcpy(buffer,&ratio,sizeof(double));
  memcpy(buffer+8,pos,4);
  memcpy(buffer+12,pos+1,4);
  memcpy(buffer+16,pos+2,4);
  memcpy(buffer+20,pos+3,4);
  memcpy(buffer+24,pos+4,4);
  memcpy(buffer+28,pos+5,4);
  mempcpy(buffer+32,&encodertype,4);
  memcpy(buffer+36,&enable,4);

  if(COM.isOpen())
  {
      try{
          COM.write(buffer,40);
      }
      catch (serial::IOException& e)
      {
          ROS_ERROR("ERROR!");
      }

  }
}
int32_t* QNode::getObjectPosition(){
  return object_position;
}
bool QNode::jointLimit(std::vector<double> joints){
  for (int i=0;i<6;i++) {
    if(joints.at(i)>joint_limit_up.at(i))
      return false;
    if(joints.at(i)<joint_limit_down.at(i))
      return false;
  }
  return true;

}

void QNode::publishPose(std::vector<double> joints,int id,double lenght){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = lenght;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.action = visualization_msgs::Marker::ADD;

  motomini_state_ptr->setJointGroupPositions ("motomini_arm", joints);
  Eigen::Affine3d pose = motomini_state_ptr->getGlobalLinkTransform("tool0");
  // Publish x axis
    Eigen::Affine3d x_pose =
    Eigen::Translation3d(lenght / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
    x_pose = pose * x_pose;
    marker.id = id;
    tf::poseEigenToMsg(x_pose,marker.pose);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_publisher.publish(marker);
    // Publish y axis
    Eigen::Affine3d y_pose =
       Eigen::Translation3d(0, lenght / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
     y_pose = pose * y_pose;
     marker.id = id+1;
     tf::poseEigenToMsg(y_pose,marker.pose);
     marker.color.r = 0.0;
     marker.color.g = 1.0;
     marker.color.b = 0.0;
     marker_publisher.publish(marker);
    // Publish z axis
    Eigen::Affine3d z_pose = Eigen::Translation3d(0, 0, lenght / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    z_pose = pose * z_pose;
    marker.id = id+2;
    tf::poseEigenToMsg(z_pose,marker.pose);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_publisher.publish(marker);
//    for (int i=0;i<3;i++) {
//      std::cout <<"Publish marker: "<< id+i << std::endl;
//    };
}
void QNode::publishPose(std::vector<double> pos,int id){
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(pos.at(3),pos.at(4),pos.at(5));
  geometry_msgs::Pose m;
  m.position.x=pos.at(0);
  m.position.y=pos.at(1);
  m.position.z=pos.at(2);
  m.orientation.x=myQuaternion.getX();
  m.orientation.y=myQuaternion.getY();
  m.orientation.z=myQuaternion.getZ();
  m.orientation.w=myQuaternion.getW();
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(m,pose);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.action = visualization_msgs::Marker::ADD;

  // Publish x axis
    Eigen::Affine3d x_pose =
    Eigen::Translation3d(0.05 / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
    x_pose = pose * x_pose;
    marker.id = id;
    tf::poseEigenToMsg(x_pose,marker.pose);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_publisher.publish(marker);
    // Publish y axis
    Eigen::Affine3d y_pose =
       Eigen::Translation3d(0, 0.05 / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
     y_pose = pose * y_pose;
     marker.id = id+1;
     tf::poseEigenToMsg(y_pose,marker.pose);
     marker.color.r = 0.0;
     marker.color.g = 1.0;
     marker.color.b = 0.0;
     marker_publisher.publish(marker);
    // Publish z axis
    Eigen::Affine3d z_pose = Eigen::Translation3d(0, 0, 0.05 / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    z_pose = pose * z_pose;
    marker.id = id+2;
    tf::poseEigenToMsg(z_pose,marker.pose);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_publisher.publish(marker);
}
void QNode::delete3Marker(int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.action = visualization_msgs::Marker::DELETE;
  marker.id = id;
  marker_publisher.publish(marker);
  marker.id = id+1;
  marker_publisher.publish(marker);
  marker.id = id+2;
  marker_publisher.publish(marker);
//  for (int i=0;i<3;i++) {
//    std::cout <<"Delete marker: "<< id+i << std::endl;
//  };
}
void QNode::deleteMarker(int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.action = visualization_msgs::Marker::DELETE;
  marker.id = id;
  marker_publisher.publish(marker);
 // std::cout <<"Delete marker: "<< id << std::endl;
}
void QNode::publishText(std::vector<double> joints,int id,std::string text){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.z = 0.03;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.text = text;
  motomini_state_ptr->setJointGroupPositions ("motomini_arm", joints);
  tf::poseEigenToMsg(motomini_state_ptr->getGlobalLinkTransform("tool0"),marker.pose);
  marker.id = id;
  marker_publisher.publish(marker);
//  std::cout <<"Publish marker: "<< id << std::endl;
}
}  // namespace motorosudp
