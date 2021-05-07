#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <strings.h>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2_ros/message_filter.h"
#include <ap0_com/ActionsNSockets.h>
#include "std_msgs/Int8.h"
#include <ap0_com/SocketClient.h>

using namespace std;

int main(int argc,char** argv)
{
  ros::init(argc, argv, "apollo_master_interupt");
  ros::NodeHandle n;

  int msg;
  bool success;

  HandleActionSocket hac;
  hac.batteryNode();

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    // recieve id ( goal_id or interrupt) from GUI
    msg = hac.mysocket.listenToGui();

    // check state and retry with this id //TODO
    success = hac.actionTo(msg);

    loop_rate.sleep();
    ros::spinOnce();

  }

  return 0;
}
