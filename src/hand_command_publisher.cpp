#include <ros/ros.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

ros::Publisher *left_hand_msg_pub, *right_hand_msg_pub;
std_msgs::Float64MultiArray hand_msg_close, hand_msg_open;

void insertDataToMultiArray()
{ 
  for (int i = 0; i < 3; i++)
    hand_msg_close.data.push_back(2.95);
  for (int j = 3; j < 6; j++)
    hand_msg_close.data.push_back(0);
  for (int k = 6; k < 9; k++)
    hand_msg_close.data.push_back(0.5);  
  for (int i = 0; i < 9;i++)
    hand_msg_open.data.push_back(0);
}

void callback(ihmc_msgs::HandDesiredConfigurationRosMessage hand_desired_config)
{
  if (hand_desired_config.hand_desired_configuration == 2)
  {
    if(hand_desired_config.robot_side == 0)
      left_hand_msg_pub->publish(hand_msg_close);
    else
      right_hand_msg_pub->publish(hand_msg_close);      
  }
  else if(hand_desired_config.hand_desired_configuration == 1)
  {
    if(hand_desired_config.robot_side == 0)
      left_hand_msg_pub->publish(hand_msg_open);
    else
      right_hand_msg_pub->publish(hand_msg_open);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_message_publisher");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  hand_msg_close.layout.dim.push_back(std_msgs::MultiArrayDimension());  
  hand_msg_close.layout.dim[0].size = 9;
  hand_msg_close.layout.dim[0].stride = 1;
  hand_msg_close.layout.dim[0].label = "publisher";

  hand_msg_open = hand_msg_close;   //Same content for both open and close message.

  insertDataToMultiArray();

  ros::Publisher publish_left = nh.advertise<std_msgs::Float64MultiArray>("/left_reflex_hand_desired_config", 100);
  left_hand_msg_pub = &publish_left;

  ros::Publisher publish_right = nh.advertise<std_msgs::Float64MultiArray>("/right_reflex_hand_desired_config", 100);
  right_hand_msg_pub = &publish_right;
  // ros::Publisher publish_ihmc = nh.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/atlas/control/hand_desired_configuration", 100);
  ros::Subscriber subscribe = nh.subscribe("/ihmc_ros/atlas/control/hand_desired_configuration", 100, callback);

  ros::waitForShutdown();
  return 0;
}