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
#include "setGoalParams.cpp"
#include <ap_msgs/GetPermission.h>
#include "tf2_ros/message_filter.h"
#include "SocketClient.h"
#include "std_msgs/Int8.h"

using namespace std;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;


// Called once when a tour becomes active
void activeCb()
{
  ROS_INFO("Tour received");
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result)
{
    /*
     * Publish Robot msg only on success and error
     * Do not publish on no final state (e.g. PENDING) and on PREEMPTED
     */
    if(state.state_==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("SUCCEEDED");
        // mysocket.informGoalReached(); //TODO
    }
    else if(state.state_==actionlib::SimpleClientGoalState::ABORTED ||
        state.state_==actionlib::SimpleClientGoalState::REJECTED
    )
    {
        ROS_INFO("NOT SUCCEEDED");
    }
}


// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("In Feedback - sending goal"); //TODO
}

//////////////

// Action function
bool actionTo( int stop_id )
{
    ROS_INFO("Received an action");

    if (stop_id >= 1 and stop_id <=5)
    {
      move_base_msgs::MoveBaseGoal goal;
      goal = setStaticGoal(stop_id);
      ROS_INFO("Sending goal");
      ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    }
    else if (stop_id == 8) //TODO
    {
      ac-> cancelGoal();
      ROS_INFO("Action wait - cancel / interupt id = 8 ");
    }
    else if (stop_id  == 0) //TODO
    {
      ac-> cancelGoal();
      ROS_INFO("Action wait - cancel / interupt id = 0 ");
    }
    else
    {
      ROS_INFO("Warning: Invalid stop_id");
    }

}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "d_com");
  ros::NodeHandle n;

  int msg, old_goal;
  bool success = true;

  // action client
  ac=new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
  ac->waitForServer();

  // create CONNECTION
  SocketClient mysocket;
  mysocket.informGoalReached();

  ros::Rate loop_rate(2);
  while(ros::ok())
  {
    // recieve id ( goal_id or interrupt) from GUI
    msg = mysocket.listenToGui();

    // check state and retry with this id //TODO
    success = actionTo(msg);

  }

  return 0;
}
