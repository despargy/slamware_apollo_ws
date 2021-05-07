#ifndef APOLLOROBOT_H
#define APOLLOROBOT_H

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <std_msgs/Int8.h>

class ApolloRobot
{
  public:
    ros::NodeHandle n;
    ros::Subscriber vel_sub ;
    ros::Subscriber battery_sub ;

    rpos::robot_platforms::SlamwareCorePlatform* sdpPtr;
    rpos::actions::RPOS_CORE_API VelocityControlMoveAction velocityAction;
    rpos::features::RPOS_CORE_API MotionPlanner motionPlanner;

    rpos::core::Pose robotPose;

    std::string map_frame = "/map";
    std::string odom_frame = "/odom";
    std::string laser_frame = "/laser";
    std::string robot_frame = "/base_link";

    std::string vel_topic = "/cmd_vel";
    std::string inter_scan_topic = "/inter/scan";
    std::string odom_topic = "/odom";
    std::string battery_topic = "/apollo/battery";

    double scan_pub_period;
    float C_FLT_PI ;
    float C_FLT_2PI ;
    std::uint32_t compensatedAngleCnt_;
    float absAngleIncrement_;
    tf::TransformBroadcaster tfBrdcstr_;

    ros::Publisher pubLaserScan_;
    ros::Publisher pubRobotPose_;
    ros::Publisher pubBatteryPercentage_;

public:
    ApolloRobot(rpos::robot_platforms::SlamwareCorePlatform* sdpPtr_);
    void velocityCallback(const geometry_msgs::TwistPtr& vel);
    void velocityNode();
    void batteryPerCallback(const std_msgs::Int8& bat);
    void batteryNode();
    void getLaserScanData();
    void getOdomData();
    void getBattery();
    void getImuData();

    bool askBumpers(rpos::core::RPOS_CORE_API Action action_);
  private:
      void fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
          , sensor_msgs::LaserScan& msgScan
          ) const;
      void compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
          , sensor_msgs::LaserScan& msgScan
          ) const;
      float calcAngleInNegativePiToPi_(float angle) const;
      std::uint32_t calcCompensateDestIndexBySrcAngle_(float srcAngle
          , bool isAnglesReverse
          ) const;
      bool isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const;

};
#endif
