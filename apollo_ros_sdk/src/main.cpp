/**
* Slamtec Robot Go Action and Get Path Demo
*
* Created By Jacky Li @ 2014-8-8
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/
#include <iostream>
#include <regex>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>
#include <rpos/features/motion_planner/velocity_control_move_action.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <apollo_ros_sdk/ApolloRobot.h>
#include <apollo_ros_sdk/ServerWorker.h>

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;

std::string ipaddress = "";
const char *ipReg = "\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}";


void showHelp(std::string appName)
{
    std::cout << "SLAMWARE console demo." << std::endl << \
        "Usage: " << appName << " <slamware_address>" << std::endl;
}

bool parseCommandLine(int argc, const char * argv[])
{
    bool opt_show_help = false;

    for (int pos=1; pos<argc; ++pos )
    {
        const char * current = argv[pos];
        if(strcmp(current, "-h") == 0) {
            opt_show_help = true;
        } else {
            ipaddress = current;
        }
    }

    std::regex reg(ipReg);
    if(! opt_show_help && ! std::regex_match(ipaddress, reg))
    {
        opt_show_help = true;
    }

    if (opt_show_help)
    {
        showHelp("moveandpathdemo");
        return false;
    }

    return true;
}

// void vCallback(const geometry_msgs::Twist& vel)
// {
//   ROS_INFO("I heard a velocity");
//   // velocityAction.setVelocity(vel.linear.x, vel.linear.y, vel.angular.z); //vel x , y, omega
// }

int main(int argc, const char * argv[])
{
    ros::init(argc, (char**) argv, "apollo_ros_sdk_server_node");

    std::cout<<"Apollo Server Node Initializing"<<std::endl;
    if(! parseCommandLine(argc, argv) )
    {
        return 1;
    }


    ///////////////////// START OF CONNECTION ESTABLISHED ////////////////////////////////

    SlamwareCorePlatform sdp;
    try {
        sdp = SlamwareCorePlatform::connect(argv[1], 1445);
        std::cout <<"SDK Version: " << sdp.getSDKVersion() << std::endl;
        std::cout <<"SDP Version: " << sdp.getSDPVersion() << std::endl;
    } catch(ConnectionTimeOutException& e) {
        std::cout <<e.what() << std::endl;
        return 1;
    } catch(ConnectionFailException& e) {
        std::cout <<e.what() << std::endl;
        return 1;
    }
    std::cout <<"Connection Successfully" << std::endl;

///////////////////// END OF CONNECTION ESTABLISHED ////////////////////////////////

  ApolloRobot apolloRobot(&sdp);
  apolloRobot.velocityNode();
  // apolloRobot.batteryNode(); // AP0_COM INSTEAD

  ros::Rate rate(10); //apolloRobot.scan_pub_period
  while(ros::ok())
  {

    apolloRobot.getLaserScanData();
    apolloRobot.getOdomData();
    apolloRobot.getBattery();

    rate.sleep();
    ros::spinOnce();

  }



    return 0;
}
