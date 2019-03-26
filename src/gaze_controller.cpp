#include "ros/ros.h"
#include <tough_controller_interface/head_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <geometry_msgs/PointStamped.h>
#include <tough_common/robot_state.h>

RobotStateInformer* state_informer_;
ChestControlInterface* chestControl_;
geometry_msgs::PointStamped object_point_;
bool value_initialized_ = false;

float getYaw(const geometry_msgs::Point& object_point_)
{
  float yaw;
  yaw = atan(object_point_.y / object_point_.x);
  return yaw;
}

void gazeControl()
{
  geometry_msgs::Point object_point_torso;
  // ROS_INFO_STREAM("Before: " << object_point_.point);
  state_informer_->transformPoint(object_point_.point, object_point_torso, "\world", "\pelvis");
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

  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  chestControl_ = new ChestControlInterface(nh);

  ros::Subscriber subsciber = nh.subscribe("/clicked_point", 1000, callback);

  while (!value_initialized_)
  {
    ROS_INFO("Waiting...");
    ros::Duration(1.0).sleep();
  }

  while (ros::ok())
  {
    gazeControl();
  }

  return 0;
}
