#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <local_examples/manipulation_planner_node.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  const std::string PLANNING_GROUP = "L_PELVIS_PALM_10DOF";

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "pelvis";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.2;
  pose.pose.orientation.w = 1.0;

  std::string link_name = "l_palm";

  ManipulationPlannerNode man;
  man.execute(PLANNING_GROUP, pose, link_name);

  return 0;
}
