#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <tf/transform_datatypes.h>
#include <map>
using namespace std;

move_base_msgs::MoveBaseGoal setStaticGoal(int stop_id )
{
  move_base_msgs::MoveBaseGoal goal;
  cout<<"Go to "<<stop_id<<endl;

if (stop_id == 1)
  {
    //we'll send the robot to picked stop
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.7; //TODO
    goal.target_pose.pose.position.y= 2.8;
    goal.target_pose.pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0,0,3.14);
    quat.normalize();
    goal.target_pose.pose.orientation.x = quat[0];
    goal.target_pose.pose.orientation.y = quat[1];
    goal.target_pose.pose.orientation.z = quat[2];
    goal.target_pose.pose.orientation.w = quat[3];
  }
  else if (stop_id == 2)
  {
    //we'll send the robot to picked stop
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.7; //TODO
    goal.target_pose.pose.position.y= 2.8;
    goal.target_pose.pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    quat.normalize();
    goal.target_pose.pose.orientation.x = quat[0];
    goal.target_pose.pose.orientation.y = quat[1];
    goal.target_pose.pose.orientation.z = quat[2];
    goal.target_pose.pose.orientation.w = quat[3];

  }
  else if (stop_id == 3)
  {
    //we'll send the robot to picked stop
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.7; //TODO
    goal.target_pose.pose.position.y= 5.0;
    goal.target_pose.pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    quat.normalize();
    goal.target_pose.pose.orientation.x = quat[0];
    goal.target_pose.pose.orientation.y = quat[1];
    goal.target_pose.pose.orientation.z = quat[2];
    goal.target_pose.pose.orientation.w = quat[3];
  }
  else if (stop_id == 4)
  {
    //we'll send the robot to picked stop
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.4; //TODO
    goal.target_pose.pose.position.y= 15.0;
    goal.target_pose.pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0,0,1.57);
    quat.normalize();
    goal.target_pose.pose.orientation.x = quat[0];
    goal.target_pose.pose.orientation.y = quat[1];
    goal.target_pose.pose.orientation.z = quat[2];
    goal.target_pose.pose.orientation.w = quat[3];
  }
  // else print a Warning
  else if (stop_id == 5)
  {
    //we'll send the robot to picked stop
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.3; //TODO
    goal.target_pose.pose.position.y= 10.0;
    goal.target_pose.pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0,0,-1.57);
    quat.normalize();
    goal.target_pose.pose.orientation.x = quat[0];
    goal.target_pose.pose.orientation.y = quat[1];
    goal.target_pose.pose.orientation.z = quat[2];
    goal.target_pose.pose.orientation.w = quat[3];
  }
  else if (stop_id == 0)
  {
    //we'll send the robot to picked stop
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.0; //TODO
    goal.target_pose.pose.position.y=  0.0;
    goal.target_pose.pose.position.z = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    quat.normalize();
    goal.target_pose.pose.orientation.x = quat[0];
    goal.target_pose.pose.orientation.y = quat[1];
    goal.target_pose.pose.orientation.z = quat[2];
    goal.target_pose.pose.orientation.w = quat[3];
  }
  return goal;
}
