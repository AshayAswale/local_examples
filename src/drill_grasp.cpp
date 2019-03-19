#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tough_moveit_planners/taskspace_planner.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_footstep/robot_walker.h>
#include <navigation_common/frame_tracker.h>
// #include <tough

bool goal_available_;
geometry_msgs::PoseStamped bag_location_;
ros::Publisher* marker_pub;
RobotWalker* walking_controller;
TaskspacePlanner* taskspace_planner;
WholebodyControlInterface* wholebody_controller;
RobotDescription* rd_;
RobotStateInformer* state_informer_;
FrameTracker* pelvis_frame_tracker_;
ArmControlInterface* arm_controller_;
ChestControlInterface* chest_controller_;
static bool is_executing = false;

// #############################################
// ############## W O R K I N G ################
// ############ 26 - 02 F I N A L ##############
// #############################################

void getUserApproval(const std::string& prompt)
{
  std::string user_input;
  std::cout << "\nEnter Y to " << prompt << std::endl;
  std::cin >> user_input;

  if (user_input == "Y")
  {
  }
  else
  {
    exit(-1);
  }
}

void resetPose()
{
  std::cout << "Resetting the robot.";
  arm_controller_->moveToDefaultPose(RobotSide::RIGHT);
  arm_controller_->moveToDefaultPose(RobotSide::LEFT);
  chest_controller_->resetPose(1.0f);
}

void drawArrow(const geometry_msgs::PoseStamped& goal, uint shape = visualization_msgs::Marker::ARROW)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = goal.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "";
  marker.id = 0;

  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = goal.pose;

  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub->publish(marker);
}

void modifyGoalForWalking(geometry_msgs::PoseStamped& goal_walk)
{
  double yaw = tf::getYaw(goal_walk.pose.orientation);
  // goal_walk.pose.position.x = goal_walk.pose.position.x - 0.5;
  // goal_walk.pose.position.y = goal_walk.pose.position.y + 0.6;

  goal_walk.pose.position.x += (-0.6 * (sin(yaw)) - 0.5 * (cos(yaw)));
  goal_walk.pose.position.y -= (-0.6 * (cos(yaw)) + 0.5 * (sin(yaw)));
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

inline void modifyGoalForManipulation(geometry_msgs::PoseStamped& goal_manipulation)
{
  goal_manipulation.pose.position.z += 0.1;

  goal_manipulation.pose.orientation.w = 0.707;
  goal_manipulation.pose.orientation.y = 0.707;
  goal_manipulation.pose.orientation.x = 0.0;
  goal_manipulation.pose.orientation.z = 0.0;
}

bool planTrajectory(const std::string planning_group, const float tolerance, const geometry_msgs::PoseStamped& goal,
                    moveit_msgs::RobotTrajectory* robot_traj)
{
  ROS_INFO("Planning trajectory for %s", planning_group.c_str());

  taskspace_planner->setAngleTolerance(tolerance);
  taskspace_planner->setPositionTolerance(tolerance);

  if (taskspace_planner->getTrajectory(goal, planning_group, *robot_traj))
  {
    wholebody_controller->executeTrajectory(*robot_traj);
    ros::Duration sleeptime = robot_traj->joint_trajectory.points.back().time_from_start;
    sleeptime.sleep();
    delete robot_traj;
    return true;
  }
  else
  {
    ROS_ERROR("trajectory with planning group %s failed.", planning_group.c_str());
    delete robot_traj;
    return false;
  }
}

void moveHandStep(const geometry_msgs::PoseStamped& goal_manipulation, const float tolerance,
                  const std::string planning_group, bool initialize = false)
{
  geometry_msgs::PoseStamped goal_manipulation_pelvis;

  if (goal_manipulation.header.frame_id == rd_->getPelvisFrame())
  {
    state_informer_->transformPose(goal_manipulation.pose, goal_manipulation_pelvis.pose,
                                   goal_manipulation.header.frame_id, rd_->getPelvisFrame());
    goal_manipulation_pelvis.header = std_msgs::Header();
    goal_manipulation_pelvis.header.frame_id = rd_->getPelvisFrame();

    modifyGoalForManipulation(goal_manipulation_pelvis);
  }
  else
  {
    goal_manipulation_pelvis = goal_manipulation;
  }

  drawArrow(goal_manipulation_pelvis);
  getUserApproval("Move hand here?");
  if (initialize)
  {
    arm_controller_->moveToZeroPose(RobotSide::RIGHT, 4.0f);
    ros::Duration(2.0).sleep();
  }
  moveit_msgs::RobotTrajectory* robot_traj = new moveit_msgs::RobotTrajectory();
  planTrajectory(planning_group, tolerance, goal_manipulation_pelvis, robot_traj);
  robot_traj = new moveit_msgs::RobotTrajectory();
}

void moveArmDownToGoal(const geometry_msgs::PoseStamped& goal_manipulation, const std::string planning_group)
{
  geometry_msgs::PoseStamped goal_manipulation_temp = goal_manipulation;
  drawArrow(goal_manipulation_temp);
  moveHandStep(goal_manipulation_temp, 0.01, planning_group);
}

void moveArmUp(const std::string planning_group, float z_offset = 0.1)
{
  geometry_msgs::PoseStamped goal_manipulation_temp;
  goal_manipulation_temp.header = std_msgs::Header();
  goal_manipulation_temp.header.frame_id = rd_->getPelvisFrame();
  state_informer_->getCurrentPose(rd_->getRightEEFrame(), goal_manipulation_temp.pose, rd_->getPelvisFrame());
  goal_manipulation_temp.pose.position.z += (z_offset - 0.1);
  drawArrow(goal_manipulation_temp);
  moveHandStep(goal_manipulation_temp, 0.01, planning_group);
}

void moveArmDown(const std::string planning_group, float z_offset = 0.1)
{
  geometry_msgs::PoseStamped goal_manipulation_temp;
  goal_manipulation_temp.header = std_msgs::Header();
  goal_manipulation_temp.header.frame_id = rd_->getPelvisFrame();
  state_informer_->getCurrentPose(rd_->getRightEEFrame(), goal_manipulation_temp.pose, rd_->getPelvisFrame());
  goal_manipulation_temp.pose.position.z -= (0.1 + z_offset);
  drawArrow(goal_manipulation_temp);
  moveHandStep(goal_manipulation_temp, 0.01, planning_group);
}

void moveHandsToPlaceGoal(const std::string planning_group)
{
  geometry_msgs::PoseStamped goal_manipulation;
  goal_manipulation.header = std_msgs::Header();
  goal_manipulation.header.frame_id = rd_->getPelvisFrame();
  goal_manipulation.pose.position.x = 0.8;
  goal_manipulation.pose.position.y = -0.4;
  goal_manipulation.pose.position.z = 0.3;

  goal_manipulation.pose.orientation.w = 0.707;
  goal_manipulation.pose.orientation.y = 0.707;
  goal_manipulation.pose.orientation.x = 0.0;
  goal_manipulation.pose.orientation.z = 0.0;

  drawArrow(goal_manipulation);
  moveHandStep(goal_manipulation, 0.01, planning_group);
}

void runTask(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO_ONCE("Running task");
  geometry_msgs::PoseStamped goal_walk = goal;
  geometry_msgs::PoseStamped goal_manipulation_world = goal;
  state_informer_->transformPose(goal.pose, goal_manipulation_world.pose, goal.header.frame_id, rd_->getWorldFrame());
  goal_manipulation_world.header.frame_id = rd_->getWorldFrame();

  bool continue_execute = true;
  std::string planning_group_7DoF = TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP;
  std::string planning_group_10DoF = TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP;

  const std::vector<double> ARM_TOWARDS_CHEST{ 1.16, -0.37, 1.70, -1.15, -2.10, 1.54, 0.06 };

  std::vector<std::vector<double>> arm_pose_vector;
  arm_pose_vector.push_back(ARM_TOWARDS_CHEST);

  geometry_msgs::PoseStamped goal_manipulation_for_drop;
  goal_manipulation_for_drop.header = std_msgs::Header();
  goal_manipulation_for_drop.header.frame_id = rd_->getPelvisFrame();
  goal_manipulation_for_drop.pose.position.x = 0.8;
  goal_manipulation_for_drop.pose.position.y = -0.5;
  goal_manipulation_for_drop.pose.position.z = 0.3;

  goal_manipulation_for_drop.pose.orientation.w = 0.707;
  goal_manipulation_for_drop.pose.orientation.y = 0.707;
  goal_manipulation_for_drop.pose.orientation.x = 0.0;
  goal_manipulation_for_drop.pose.orientation.z = 0.0;

  while (continue_execute)
  {
    int operation_number;
    std::cout << "\n************ ************ ************";
    std::cout << "\nEnter 1 for -> 'Walk to goal'... ";
    std::cout << "\nEnter 2 for -> 'Move toward bag - 7  DoF'... ";
    std::cout << "\nEnter 3 for -> 'Move toward bag - 10 DoF'... ";
    std::cout << "\nEnter 4 for -> 'Move arm down'... ";
    std::cout << "\nEnter 5 for -> 'Move arm up'... ";
    std::cout << "\nEnter 6 for -> 'Retract hand toward chest and reset chest'... ";
    std::cout << "\nEnter 7 for -> 'Reset chest'... ";
    std::cout << "\nEnter 8 for -> 'Arm to drop pose.'... ";
    std::cout << "\nEnter 9 for -> 'Resetting Robot'... ";
    std::cout << "\nEnter 0 to QUIT... \n";

    std::cin >> operation_number;

    switch (operation_number)
    {
      case 1:  // WALKING
        modifyGoalForWalking(goal_walk);
        drawArrow(goal_walk);

        getUserApproval("Walk to goal");
        if (!walkRobot(goal_walk))
        {
          ROS_INFO("Walking Failed");
          exit(-1);
        }
        ROS_INFO("Walking Complete");
        ros::Duration(1.0).sleep();

        while (pelvis_frame_tracker_->isInMotion() == frame_track_status::FRAME_IN_MOTION)
        {
          ROS_INFO("Sleeping for one second.");
          ros::Duration(1.0).sleep();
        }
        break;

      case 2:  // 7 DoF Planning and execution
        drawArrow(goal_manipulation_world);
        moveHandStep(goal_manipulation_world, 0.2, planning_group_7DoF, true);
        break;

      case 3:  // 10 DoF Planning and execution
        drawArrow(goal_manipulation_world);
        moveHandStep(goal_manipulation_world, 0.01, planning_group_10DoF);
        break;

      case 4:  // Move Arm Down
        moveArmDown(planning_group_7DoF);
        break;

      case 5:  // Move Arm Up
        moveArmUp(planning_group_7DoF);
        break;

      case 6:  // Retract Hand toward chest
        arm_controller_->moveArmJoints(RobotSide::RIGHT, arm_pose_vector, 2.0f);
        chest_controller_->resetPose(2.0f);
        break;

      case 7:  // Reset Chest
        chest_controller_->resetPose(2.0f);
        break;

      case 8:  // 10 DoF Planning and execution
        drawArrow(goal_manipulation_for_drop);
        moveHandStep(goal_manipulation_for_drop, 0.01, planning_group_10DoF);
        break;

      case 9:  // Reset the Robot
        chest_controller_->resetPose(2.0f);
        arm_controller_->moveToDefaultPose(RobotSide::RIGHT);
        arm_controller_->moveToDefaultPose(RobotSide::LEFT);
        break;

      case 0:
        continue_execute = false;
        std::cout << "Quit command received, Exiting the code.\n";
        break;

      default:
        std::cout << "Invalid Entry!";
        break;
    }
  }
}

void goalMessageCB(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO_ONCE("Starting task");
  bag_location_ = goal;
  goal_available_ = true;
}

int main(int argc, char** argv)
{
  goal_available_ = false;

  ros::init(argc, argv, "drill_grasp");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher marker_temp_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  marker_pub = &marker_temp_pub;

  rd_ = RobotDescription::getRobotDescription(nh);
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);

  taskspace_planner = new TaskspacePlanner(nh);
  wholebody_controller = new WholebodyControlInterface(nh);
  walking_controller = new RobotWalker(nh, 1.5, 1.5);
  pelvis_frame_tracker_ = new FrameTracker(nh, rd_->getWorldFrame(), rd_->getPelvisFrame());
  arm_controller_ = new ArmControlInterface(nh);
  chest_controller_ = new ChestControlInterface(nh);

  ros::Subscriber subscribe = nh.subscribe("/goal", 1000, goalMessageCB);
  ros::Rate loop_rate(10.0);

  while (ros::ok() && !goal_available_)
  {
    loop_rate.sleep();
  }
  resetPose();
  runTask(bag_location_);
  goal_available_ = false;  // ideally this should be scoped with mutex

  delete taskspace_planner;
  delete wholebody_controller;
  delete walking_controller;
  // delete
  return 0;
}
