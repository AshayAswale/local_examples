#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <tough_common/tough_common_names.h>
#include <tough_common/robot_description.h>
#include <tough_moveit_planners/taskspace_planner.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <tough_footstep/robot_walker.h>

RobotDescription* rd_;
RobotWalker* walking_controller;
RobotStateInformer* state_informer_;

const geometry_msgs::PoseStamped getValvePose(int trial)
{
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.pose.orientation.w = 1;

  if(trial == 1)
  {
    temp_pose.pose.position.x = 2.0;
    temp_pose.pose.position.y = 1.0;
    temp_pose.pose.position.z = 0.3;
  }
  else
  {
    temp_pose.pose.position.x = 0.4;
    temp_pose.pose.position.y = -0.5;
    temp_pose.pose.position.z = 0.3;
  }  
}

geometry_msgs::PoseStamped getWalkGoal(geometry_msgs::PoseStamped &goal_walk)
{
  goal_walk.pose.position.x -= 0.4;
  goal_walk.pose.position.y += 0.5;
  goal_walk.pose.position.z = 0.0;
}

bool walkRobot(const geometry_msgs::PoseStamped& goal_walk)
{
  geometry_msgs::Pose2D msg_out, msg_in;
  msg_in.x = goal_walk.pose.position.x;
  msg_in.y = goal_walk.pose.position.y;
  state_informer_->transformPose(msg_in, msg_out, goal_walk.header.frame_id, rd_->getWorldFrame());

  msg_out.theta = tf::getYaw(goal_walk.pose.orientation);

  ROS_INFO("Starting to walk.");
  return walking_controller->walkToGoal(msg_out, true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_hand_in_traj");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rd_ = RobotDescription::getRobotDescription(nh);
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  walking_controller = new RobotWalker(nh, 1.5, 1.5);


  ArmControlInterface::ArmTaskSpaceData arm_data;

  geometry_msgs::PoseStamped valve_pose = getValvePose(1);
  geometry_msgs::PoseStamped walk_goal = getWalkGoal(valve_pose);

  if(!walkRobot(walk_goal)) 
  {
    ROS_INFO("Walking Failed");
    exit(-1);
  }
  ROS_INFO("Walking Complete");
  ros::Duration(1.0).sleep();

  std::vector<ArmControlInterface::ArmTaskSpaceData> arm_data_vec;

  geometry_msgs::Quaternion quat;
  quat.w = 1;
  arm_data.pose.orientation = quat;

  float radius = 0.1;

  for (double theta = 0; theta <= M_PI; theta += 0.2) 
  {
    arm_data.pose.position.z = valve_pose.pose.position.z + radius * cos(theta);
    arm_data.pose.position.y = valve_pose.pose.position.y - radius * sin(theta);
    tf::Quaternion quat_tf = tf::createQuaternionFromRPY(theta, 0, 0);
    tf::quaternionTFToMsg(quat_tf, quat);
    arm_data.pose.orientation = quat;
    arm_data.time += 0.2;
    arm_data_vec.push_back(arm_data);
  }

  ArmControlInterface arm_controller(nh);
  arm_controller.moveArmInTaskSpace(arm_data_vec);

  ros::Duration(10).sleep();

  return 0;
}
