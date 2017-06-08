#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
using namespace Eigen;
using namespace std;
class motion_compensation{
private:
  std_msgs:: Float64 orient_msg;
  ros::Publisher psm_pub1;
  ros::Publisher psm_pub2;
  ros::Publisher psm_pub3;
  ros::Publisher psm_pub4;
  ros::Publisher psm_pub5;
  ros::Publisher psm_pub6;
  ros::Publisher mtm_pub1;
  ros::Publisher mtm_pub2;
  ros::Publisher mtm_pub3;
  ros::Publisher mtm_pub4;
  ros::Publisher mtm_pub5;
  ros::Publisher mtm_pub6;
  ros::Publisher mtm_pub7;
  ros::Publisher pris_pub1;
public:
  motion_compensation(ros::NodeHandle n)
  {
    psm_pub1 = n.advertise<std_msgs::Float64>("/psm_joint_1_position_controller/command",100);
    psm_pub2 = n.advertise<std_msgs::Float64>("/psm_joint_2_position_controller/command",100);
    psm_pub3 = n.advertise<std_msgs::Float64>("/psm_joint_3_position_controller/command",100);
    psm_pub4 = n.advertise<std_msgs::Float64>("/psm_joint_4_position_controller/command",100);
    psm_pub5 = n.advertise<std_msgs::Float64>("/psm_joint_5_position_controller/command",100);
    psm_pub6 = n.advertise<std_msgs::Float64>("/psm_joint_6_position_controller/command",100);
    mtm_pub1 = n.advertise<std_msgs::Float64>("/mtm_joint_1_position_controller/command",100);
    mtm_pub2 = n.advertise<std_msgs::Float64>("/mtm_joint_2_position_controller/command",100);
    mtm_pub3 = n.advertise<std_msgs::Float64>("/mtm_joint_3_position_controller/command",100);
    mtm_pub4 = n.advertise<std_msgs::Float64>("/mtm_joint_4_position_controller/command",100);
    mtm_pub5 = n.advertise<std_msgs::Float64>("/mtm_joint_5_position_controller/command",100);
    mtm_pub6 = n.advertise<std_msgs::Float64>("/mtm_joint_6_position_controller/command",100);
    mtm_pub7 = n.advertise<std_msgs::Float64>("/mtm_joint_7_position_controller/command",100);
    pris_pub1 = n.advertise<std_msgs::Float64>("/prismatic_joint_position_controller/command",100);
  }
void moveRobot(MatrixXd theta_psm, MatrixXd theta_mtm, MatrixXd theta_pris);
VectorXf randv(int x);
};



