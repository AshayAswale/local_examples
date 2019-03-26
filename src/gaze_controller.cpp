#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>

#include <tough_controller_interface/head_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>

RobotStateInformer* state_informer_;
ChestControlInterface* chestControl_;
HeadControlInterface* headController_;
geometry_msgs::PointStamped object_point_;
RobotDescription* rd_;
bool value_initialized_ = false;

float getYaw(const geometry_msgs::Point& object_point)
{
  float yaw = atan(object_point.y / object_point.x);
  return yaw;
}

float getPitch(const geometry_msgs::Point& object_point)
{
  float pitch = atan(object_point.z / object_point.x);
  return pitch;
}

void gazeControl()
{
  geometry_msgs::Point object_point_head;

  float yaw_error = 0, roll_error = 0, pitch_error = 0;
  state_informer_->transformPoint(object_point_.point, object_point_head, rd_->getWorldFrame(),
                                  TOUGH_COMMON_NAMES::ROBOT_HEAD_FRAME_TF);
  yaw_error = getYaw(object_point_head);
  pitch_error = -1 * getPitch(object_point_head);

  double head_position = state_informer_->getJointPosition("neck_ry");

  ROS_INFO_STREAM(head_position);

  headController_->moveHead(roll_error, pitch_error, yaw_error);
  ros::Duration(0.25).sleep();
}

void moveGazeOnce()
{
  geometry_msgs::Point object_point_torso;
  // ROS_INFO_STREAM("Before: " << object_point_.point);
  state_informer_->transformPoint(object_point_.point, object_point_torso, rd_->getWorldFrame(), rd_->getPelvisFrame());
  // ROS_INFO_STREAM("After: " << object_point_torso);
  float yaw, roll = 0, pitch = 0;
  yaw = getYaw(object_point_torso);
  ROS_INFO_STREAM(yaw);

  chestControl_->controlChest(roll, pitch, yaw);
  ros::Duration(1.0).sleep();
}

void callback(const geometry_msgs::PointStamped clicked_point)
{
  ROS_INFO("In Callback.");
  object_point_ = clicked_point;
  value_initialized_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gaze_controller");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  headController_ = new HeadControlInterface(nh);
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  chestControl_ = new ChestControlInterface(nh);
  rd_ = RobotDescription::getRobotDescription(nh);

  ros::Subscriber subsciber = nh.subscribe("/clicked_point", 1000, callback);

  while (!value_initialized_)
  {
    ROS_INFO("Waiting...");
    ros::Duration(1.0).sleep();
  }

  moveGazeOnce();

  while (ros::ok())
  {
    gazeControl();
  }

  return 0;
}
