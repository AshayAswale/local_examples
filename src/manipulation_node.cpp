#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tough_common/robot_state.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_common/robot_description.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  const std::string PLANNING_GROUP = "leftMiddleFingerGroup";
  robot_model_loader::RobotModelLoader robot_model_loader("/atlas/robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  RobotStateInformer* robotStateInformer = RobotStateInformer::getRobotStateInformer(nh);
  ;

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if (!nh.getParam("/move_group/planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "pelvis";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.2;
  pose.pose.orientation.w = 1.0;

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  req.start_state.joint_state.header = std_msgs::Header();
  robotStateInformer->getJointNames(req.start_state.joint_state.name);
  robotStateInformer->getJointPositions(req.start_state.joint_state.position);
  robotStateInformer->getJointVelocities(req.start_state.joint_state.velocity);
  robotStateInformer->getJointEfforts(req.start_state.joint_state.effort);

  req.group_name = "leftMiddleFingerGroup";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("l_hand", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  ros::Publisher display_publisher =
      nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  WholebodyControlInterface wholeBody(nh);
  wholeBody.executeTrajectory(response.trajectory);

  ros::Duration(10).sleep();

  planner_instance.reset();

  return 0;
}
