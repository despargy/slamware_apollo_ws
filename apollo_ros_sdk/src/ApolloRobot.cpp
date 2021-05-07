#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <iostream>
#include <regex>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>
#include <rpos/features/motion_planner/velocity_control_move_action.h>
#include <rpos/features/motion_planner/feature.h>
#include <geometry_msgs/Twist.h>
#include <apollo_ros_sdk/ApolloRobot.h>
#include <boost/assert.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>

ApolloRobot::ApolloRobot(rpos::robot_platforms::SlamwareCorePlatform* sdpPtr_)
{
  // INIT PUBLISHERS
  pubLaserScan_ = n.advertise<sensor_msgs::LaserScan>(inter_scan_topic, 10); //srvParams.scan_simple_topic
  pubRobotPose_ = n.advertise<nav_msgs::Odometry>(odom_topic, 10);
  pubBatteryPercentage_ = n.advertise<std_msgs::Int8>(battery_topic, 10);

  // ROBOT PLATOFORM
  ROS_INFO("Constructor ApolloRobot");
  sdpPtr = sdpPtr_ ;
  velocityAction = sdpPtr->velocityControl() ;

  // FIX VAR from slamware ros sdk
  C_FLT_PI = ((float)M_PI);
  C_FLT_2PI = (C_FLT_PI * 2);

  scan_pub_period = 0.1f;
  compensatedAngleCnt_ = 360u ;
  absAngleIncrement_ = (C_FLT_2PI / compensatedAngleCnt_);
}

void ApolloRobot::velocityNode()
{
  vel_sub = n.subscribe(vel_topic, 1000, &ApolloRobot::velocityCallback,this);
}

void ApolloRobot::velocityCallback(const geometry_msgs::TwistPtr& vel)
{
  ROS_INFO("I heard a velocity");

  velocityAction = sdpPtr->velocityControl();
  // velocityAction = motionPlanner.(*sdpPtr)velocityControl();

  velocityAction.setVelocity(vel->linear.x, vel->linear.y, vel->angular.z); //vel x , y, omega
  // ROS_INFO("STATUS %s",velocityAction.getStatus());
  std::cout<<"Callback Status "<<velocityAction.getStatus()<<std::endl;
  // velocityAction.cancel();
  // std::cout<<"Callback Reason "<<velocityAction.getReason()<<std::endl;

}

/* THIS IS HANDLED AP0_COM INSTEAD
void ApolloRobot::batteryNode()
{
  battery_sub = n.subscribe(battery_topic,1000, &ApolloRobot::batteryPerCallback, this);
}

void ApolloRobot::batteryPerCallback(const std_msgs::Int8& bat)
{
  ROS_INFO("My batter is %d",bat.data);
}
*/

void ApolloRobot::getBattery()
{
  std_msgs::Int8 batPercentageMsg;
  batPercentageMsg.data = sdpPtr->getBatteryPercentage();
  pubBatteryPercentage_.publish(batPercentageMsg);
}

void ApolloRobot::getLaserScanData()
{

  ros::Time startScanTime = ros::Time::now();
  rpos::features::system_resource::LaserScan tLs = sdpPtr->getLaserScan();
  ros::Time endScanTime = ros::Time::now();
  double dblScanDur = (endScanTime - startScanTime).toSec();

  const auto& points = tLs.getLaserPoints();
  if (points.size() < 2)
  {
      ROS_ERROR("laser points count: %u, too small, skip publish.", (unsigned int)points.size());
      return;
  }

  const auto laserPose = (tLs.getHasPose() ? tLs.getLaserPointsPose() : robotPose);


  sensor_msgs::LaserScan msgScan;
  msgScan.header.stamp = startScanTime;
  msgScan.header.frame_id = laser_frame;
  fillRangeMinMaxInMsg_(points, msgScan);

  bool angle_compensate = true; //TODO

  if (angle_compensate)
  {
      compensateAndfillRangesInMsg_(points, msgScan);
  }
  else
  {
      //msgScan.intensities.resize(points.size());
      msgScan.ranges.resize(points.size());

      for (size_t i = 0; i < points.size(); ++i)
      {
          if (!points[i].valid())
          {
              msgScan.ranges[i] = std::numeric_limits<float>::infinity();
          }
          else
          {
              msgScan.ranges[i] = points[i].distance();
          }
      }
      msgScan.angle_min =  points.front().angle();
      msgScan.angle_max =  points.back().angle();
      msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
  }
  BOOST_ASSERT(2 <= msgScan.ranges.size());
  msgScan.scan_time = dblScanDur;
  msgScan.time_increment = dblScanDur / (double)(msgScan.ranges.size() - 1);

//   {
//       tf::Transform laserTrans;
//       laserTrans.setOrigin(tf::Vector3(laserPose.x(), laserPose.y(), 0.0));
//       tf::Quaternion qLaserTrans = tf::createQuaternionFromYaw(laserPose.yaw());
//       laserTrans.setRotation(qLaserTrans);
//       tfBrdcstr_.sendTransform(tf::StampedTransform(laserTrans, startScanTime, map_frame, laser_frame));
//   }
  pubLaserScan_.publish(msgScan);
}


void ApolloRobot::getOdomData()
{
  // bool fixed_odom_map_tf = false; //TODO
  // // send TF transform
  // if (fixed_odom_map_tf) // only for debug rosrun
  // {
  //     tf::Transform tfIdenty;
  //     tfIdenty.setOrigin(tf::Vector3 (0.0, 0.0, 0.0));
  //     tfIdenty.setRotation(tf::Quaternion(0, 0, 0, 1));
  //
  //     // tfBrdcstr_.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), odom_frame, robot_frame));
  //
  //     // TF UNABLED
  //     tfBrdcstr_.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(),  robot_frame, laser_frame));
  // }

  const rpos::core::Pose robotPose = sdpPtr->getPose();
  // wkDat->robotPose = robotPose; // MISSING
  // std::cout<<robotPose.yaw()<<std::endl;

  // publish odom transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(robotPose.x(), robotPose.y(), 0.0));
  tf::Quaternion q = tf::createQuaternionFromYaw(robotPose.yaw());
  transform.setRotation(q);
  tfBrdcstr_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, robot_frame));

  // send TF transform
  nav_msgs::Odometry msgRobotPose;
  msgRobotPose.header.frame_id = odom_frame;
  msgRobotPose.header.stamp = ros::Time::now();
  // sltcToRosMsg(robotPose, msgRobotPose.pose.pose); // MISSING
  msgRobotPose.pose.pose.position.x = robotPose.x();
  msgRobotPose.pose.pose.position.y = robotPose.y();
  msgRobotPose.pose.pose.position.z = robotPose.z();

  tf::Quaternion q1;
  q1.setRPY(robotPose.roll(), robotPose.pitch(), robotPose.yaw());
  tf::quaternionTFToMsg(q1, msgRobotPose.pose.pose.orientation);
  // msgRobotPose.pose.pose.orientation = q;

  // msgRobotPose.pose.pose.orientation.setRPY(robotPose.yaw(), 0, 0);// robotPose.yaw();
  // msgRobotPose.pose.pose.orientation. = robotPose.pitch();
  // msgRobotPose.pose.pose.orientation.y = robotPose.roll();

  pubRobotPose_.publish(msgRobotPose);

}

bool ApolloRobot::askBumpers(rpos::core::RPOS_CORE_API Action action_)
{
  std::string reason = action_.getReason();
  if (reason == "blocked")
    return true;
  else
    return false;
}

void ApolloRobot::getImuData()
{
  rpos::core::Imu imu_;
  sdpPtr->getImuInRobotCoordinate(imu_);
  rpos::core::Quaternion q = imu_.quaternion();
  std::cout <<"Quaternion: \n"<< q.x() <<" \n"<< q.y()<<" \n"<< q.z()<<" \n"<< q.w()<<'\n';
  rpos::core::Vector3f v = imu_.eulerAngle();
  std::cout <<"Vector: \n"<< v << '\n';
}
//////////////////////////// EXTRA FOR LASER - START  //////////
void ApolloRobot::fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
        , sensor_msgs::LaserScan& msgScan
        ) const
{
    msgScan.range_min = std::numeric_limits<float>::infinity();
    msgScan.range_max = 0.0f;
    for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
    {
        if (cit->valid())
        {
            const float tmpDist = cit->distance();

            if (tmpDist < msgScan.range_min)
            {
                msgScan.range_min = std::max<float>(0.0f, tmpDist);
            }

            if (msgScan.range_max < tmpDist)
            {
                msgScan.range_max = tmpDist;
            }
        }
    }
}


void ApolloRobot::compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
        , sensor_msgs::LaserScan& msgScan
        ) const
{
    BOOST_ASSERT(2 <= laserPoints.size());

    msgScan.ranges.clear();
    msgScan.ranges.resize(compensatedAngleCnt_, std::numeric_limits<float>::infinity());

    const bool isAnglesReverse = (laserPoints.back().angle() < laserPoints.front().angle());
    if (!isAnglesReverse)
    {
        msgScan.angle_min = -C_FLT_PI;
        msgScan.angle_max = C_FLT_PI - absAngleIncrement_;
        msgScan.angle_increment = absAngleIncrement_;
    }
    else
    {
        msgScan.angle_min = C_FLT_PI;
        msgScan.angle_max = (-C_FLT_PI + absAngleIncrement_);
        msgScan.angle_increment = -absAngleIncrement_;
    }

    std::vector<float> tmpSrcAngles(compensatedAngleCnt_);
    for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
    {
        if (cit->valid())
        {
            const float srcAngle = calcAngleInNegativePiToPi_(cit->angle());
            const std::uint32_t destIdx = calcCompensateDestIndexBySrcAngle_(srcAngle, isAnglesReverse);
            BOOST_ASSERT(destIdx < compensatedAngleCnt_);
            const float destAngle = calcAngleInNegativePiToPi_(msgScan.angle_min + msgScan.angle_increment * destIdx);

            const bool shouldWrite = (std::isinf(msgScan.ranges[destIdx])
                || isSrcAngleMoreCloseThanOldSrcAngle_(srcAngle, destAngle, tmpSrcAngles[destIdx])
                );
            if (shouldWrite)
            {
                msgScan.ranges[destIdx] = cit->distance();
                tmpSrcAngles[destIdx] = srcAngle;
            }
        }
    }

    //ROS_INFO("compensatedAngleCnt_: %u, isAnglesReverse: %s.", compensatedAngleCnt_, (isAnglesReverse ? "true": "false"));
}

float ApolloRobot::calcAngleInNegativePiToPi_(float angle) const
{
    float fRes = std::fmod(angle + C_FLT_PI, C_FLT_2PI);
    if (fRes < 0.0f)
        fRes += C_FLT_2PI;
    fRes -= C_FLT_PI;

    if (fRes < -C_FLT_PI)
        fRes = -C_FLT_PI;
    else if (C_FLT_PI <= fRes)
        fRes = -C_FLT_PI;
    return fRes;
}

std::uint32_t ApolloRobot::calcCompensateDestIndexBySrcAngle_(float srcAngle
    , bool isAnglesReverse
    ) const
{
    BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);

    float fDiff = (isAnglesReverse ? (C_FLT_PI - srcAngle) : (srcAngle + C_FLT_PI));
    fDiff = std::max<float>(0.0f, fDiff);
    fDiff = std::min<float>(fDiff, C_FLT_2PI);

    std::uint32_t destIdx = static_cast<std::uint32_t>(std::round(fDiff / absAngleIncrement_));
    if (compensatedAngleCnt_ <= destIdx)
        destIdx = 0;
    return destIdx;
}

bool ApolloRobot::isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const
{
    BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
    BOOST_ASSERT(-C_FLT_PI <= destAngle && destAngle < C_FLT_PI);
    BOOST_ASSERT(-C_FLT_PI <= oldSrcAngle && oldSrcAngle < C_FLT_PI);

    float newDiff = std::abs(destAngle - srcAngle);
    if (C_FLT_2PI <= newDiff)
        newDiff = 0.0f;
    else if (C_FLT_PI < newDiff)
        newDiff = C_FLT_2PI - newDiff;

    float oldDiff = std::abs(destAngle - oldSrcAngle);
    if (C_FLT_2PI <= oldDiff)
        oldDiff = 0.0f;
    else if (C_FLT_PI < oldDiff)
        oldDiff = C_FLT_2PI - oldDiff;

    return (newDiff < oldDiff);
}
//////////////////////////// EXTRA FOR LASER - END  //////////

//////////////////////////// EXTRA FOR ODOM - START  //////////

//////////////////////////// EXTRA FOR ODOM - END  //////////
