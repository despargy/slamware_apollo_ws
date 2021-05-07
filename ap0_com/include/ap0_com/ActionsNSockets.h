#ifndef ACTIONSNSOCKETS_H
#define ACTIONSNSOCKETS_H

#include <ap0_com/SocketClient.h>
#include <std_msgs/Int8.h>
#include <string>

class HandleActionSocket
{
  public:
    int achievedId, tryId;;
    SocketClient mysocket;
    double  battery_msg;

    ros::Subscriber battery_sub ;
    ros::NodeHandle nbat;
    std::string battery_topic;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
    HandleActionSocket();
    void activeCb();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr& result);
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    bool actionTo( int stop_id );
    void batteryNode();
    void batteryPerCallback(const std_msgs::Int8& bat);

};
#endif
