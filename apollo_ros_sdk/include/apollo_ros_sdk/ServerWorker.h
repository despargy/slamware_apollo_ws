#ifndef SERVERWORKER_H
#define SERVERWORKER_H

#pragma once

// #include "server_worker_base.h"
#include <rpos/robot_platforms/slamware_core_platform.h>

#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotPoseWorker
    {
    public:
        typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

    public:
        ServerRobotPoseWorker(
              const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotPoseWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        ros::Publisher pubRobotPose_;
    };

    //////////////////////////////////////////////////////////////////////////

        class ServerLaserScanWorker
        {
        public:
          typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

        public:
            ServerLaserScanWorker(
               const std::string& wkName
                , const boost::chrono::milliseconds& triggerInterval
                );
            virtual ~ServerLaserScanWorker();

        protected:
            virtual void doPerform(slamware_platform_t& pltfm);

        private:
            void fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
                , sensor_msgs::LaserScan& msgScan
                ) const;

            float calcAngleInNegativePiToPi_(float angle) const;

            std::uint32_t calcCompensateDestIndexBySrcAngle_(float srcAngle
                , bool isAnglesReverse
                ) const;
            bool isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const;
            void compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
                , sensor_msgs::LaserScan& msgScan
                ) const;

        private:
            std::uint32_t compensatedAngleCnt_;
            float absAngleIncrement_;

            ros::Publisher pubLaserScan_;
        };
#endif
