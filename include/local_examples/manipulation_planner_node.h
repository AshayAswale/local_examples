#ifndef MANIPULATION_PLANNER_NODE_H
#define MANIPULATION_PLANNER_NODE_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <tough_common/robot_state.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_common/robot_description.h>

class ManipulationPlannerNode
{
public:
  int execute(std::string, geometry_msgs::PoseStamped, std::string);

private:
  ros::NodeHandle nh;
  RobotStateInformer* robotStateInformer = RobotStateInformer::getRobotStateInformer(nh);
  std::string link_name = "l_palm";
};

#endif  // MANIPULATION_PLANNER_NODE_H
