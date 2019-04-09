#include <ros/ros.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

ros::Publisher* hand_msg_pub;


void callback(ihmc_msgs::HandDesiredConfigurationRosMessage hand_desired_config)
{
    std_msgs::Float64MultiArray hand_msg;
    hand_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
    hand_msg.layout.dim[0].size = 9;
    hand_msg.layout.dim[0].stride = 1;
    hand_msg.layout.dim[0].label = "publisher";
    
  if (hand_desired_config.hand_desired_configuration == 2)
  {
    for (int i = 0; i < 3; i++)
      hand_msg.data.push_back(2.95);
    for (int j = 3; j < 6; j++)
      hand_msg.data.push_back(0);
    for (int k = 6; k < 9; k++)
      hand_msg.data.push_back(0.5);
  }
  else if(hand_desired_config.hand_desired_configuration == 1)
  {
    for (int i = 0; i < 9;i++)
      hand_msg.data.push_back(0);
  }

  hand_msg_pub->publish(hand_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_message_publisher");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher publish = nh.advertise<std_msgs::Float64MultiArray>("/reflex_hand_desired_config", 100);
  hand_msg_pub = &publish;
  ros::Subscriber subscribe = nh.subscribe("/ihmc_ros/atlas/control/hand_desired_configuration", 100, callback);

  ros::waitForShutdown();
  return 0;
}
