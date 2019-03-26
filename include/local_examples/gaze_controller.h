#ifndef GAZE_CONTROLLER_H
#define GAZE_CONTROLLER_H

#include "ros/ros.h"
#include <tough_controller_interface/head_control_interface.h>
#include <geometry_msgs/PointStamped.h>

class GazeController
{
private:
  void callback(const geometry_msgs::PointStamped point);
};

#endif