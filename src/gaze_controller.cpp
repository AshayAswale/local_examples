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
bool execute_chest_once = false;

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
  execute_chest_once = false;
}

void gazeControl()
{
  if (execute_chest_once)
    moveGazeOnce();

  geometry_msgs::Point object_point_head;

  float yaw_error = 0, roll_error = 0, pitch_error = 0;
  state_informer_->transformPoint(object_point_.point, object_point_head, rd_->getWorldFrame(),
                                  TOUGH_COMMON_NAMES::ROBOT_HEAD_FRAME_TF);

  yaw_error = getYaw(object_point_head);
  pitch_error = -1 * getPitch(object_point_head);

  double pitch_current = state_informer_->getJointPosition("neck_ry");
  double yaw_current = state_informer_->getJointPosition("back_bkz");  // should be taken from rd_
  
  static float yaw_error_derivative = 0.0, pitch_error_derivative = 0.0;

  float roll = 0, pitch = 0, yaw = 0;
  float kp_pitch = 1, kp_yaw = 1, kd_yaw = 1, kd_pitch = 1;

  pitch = pitch_current + kp_pitch * pitch_error + kd_pitch*pitch_error_derivative;
  yaw = yaw_current + kp_yaw * yaw_error + kd_yaw * yaw_error_derivative;
  ROS_INFO_STREAM("yaw: "<<yaw<<"   pitch: "<<pitch);

  // if (yaw < 0.15 && yaw > -0.15)
  headController_->moveHead(0, pitch_error, 0);
  chestControl_->controlChest(0, 0, yaw);
  
  static float yaw_error_old = yaw_error, pitch_error_old = pitch_error;
  yaw_error_derivative = (yaw_error - yaw_error_old) / 0.25f;
  pitch_error_derivative = (pitch_error - pitch_error_old) / 0.25f;


  ros::Duration(0.25).sleep();
}

void callback(const geometry_msgs::PointStamped clicked_point)
{
  ROS_INFO("In Callback.");
  object_point_ = clicked_point;
  value_initialized_ = true;
  execute_chest_once = true;
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

  while (ros::ok())
  {
    gazeControl();
  }

  return 0;
}
