#include <ServerWorker.h>
    //////////////////////////////////////////////////////////////////////////

    ServerLaserScanWorker::ServerLaserScanWorker(
         const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )

    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubLaserScan_ = nhRos.advertise<sensor_msgs::LaserScan>("/inter/scan", 10); //srvParams.scan_simple_topic
    }

    ServerLaserScanWorker::~ServerLaserScanWorker()
    {
        //
    }

    void ServerLaserScanWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = workData();

        ros::Time startScanTime = ros::Time::now();
        rpos::features::system_resource::LaserScan tLs = pltfm.getLaserScan();
        ros::Time endScanTime = ros::Time::now();
        double dblScanDur = (endScanTime - startScanTime).toSec();

        const auto& points = tLs.getLaserPoints();
        if (points.size() < 2)
        {
            ROS_ERROR("laser points count: %u, too small, skip publish.", (unsigned int)points.size());
            return;
        }

        const auto laserPose = (tLs.getHasPose() ? tLs.getLaserPointsPose() : wkDat->robotPose);
        //ROS_INFO("has laser pose: %s, robotPose: ((%f, %f), (%f)), laserPose: ((%f, %f), (%f))."
        //    , (tLs.getHasPose() ? "true" : "false")
        //    , wkDat->robotPose.x(), wkDat->robotPose.y(), wkDat->robotPose.yaw()
        //    , laserPose.x(), laserPose.y(), laserPose.yaw()
        //    );

        sensor_msgs::LaserScan msgScan;
        msgScan.header.stamp = startScanTime;
        msgScan.header.frame_id = srvParams.laser_frame;
        fillRangeMinMaxInMsg_(points, msgScan);

        if (srvParams.angle_compensate)
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

        {
            tf::Transform laserTrans;
            laserTrans.setOrigin(tf::Vector3(laserPose.x(), laserPose.y(), 0.0));
            tf::Quaternion qLaserTrans = tf::createQuaternionFromYaw(laserPose.yaw());
            laserTrans.setRotation(qLaserTrans);
            tfBrdcst.sendTransform(tf::StampedTransform(laserTrans, startScanTime, srvParams.map_frame, srvParams.laser_frame));
        }
        pubLaserScan_.publish(msgScan);
    }

    void ServerLaserScanWorker::fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
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

    float ServerLaserScanWorker::calcAngleInNegativePiToPi_(float angle) const
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

    std::uint32_t ServerLaserScanWorker::calcCompensateDestIndexBySrcAngle_(float srcAngle
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

    bool ServerLaserScanWorker::isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const
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

    void ServerLaserScanWorker::compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
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

    //////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////

        ServerRobotPoseWorker::ServerRobotPoseWorker( const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            )

        {
            const auto& srvParams = serverParams();
            auto& nhRos = rosNodeHandle();
            pubRobotPose_ = nhRos.advertise<nav_msgs::Odometry>(srvParams.odom_topic, 10);
        }

        ServerRobotPoseWorker::~ServerRobotPoseWorker()
        {
            //
        }

        void ServerRobotPoseWorker::doPerform(slamware_platform_t& pltfm)
        {
            const auto& srvParams = serverParams();
            auto& tfBrdcst = tfBroadcaster();
            auto wkDat = mutableWorkData();

            // send TF transform
            if (srvParams.fixed_odom_map_tf) // only for debug rosrun
            {
                tf::Transform tfIdenty;
                tfIdenty.setOrigin(tf::Vector3 (0.0, 0.0, 0.0));
                tfIdenty.setRotation(tf::Quaternion(0, 0, 0, 1));

                tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.map_frame, srvParams.odom_frame));
                //tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.robot_frame, srvParams.laser_frame));
            }

            // check power
            //int battPercentage = pltfm.getBatteryPercentage();
            //if (battPercentage < 10)
            //    std::cout << "lower power!! Battery: " << battPercentage << "%." << std::endl;

            //const rpos::core::Location location = pltfm.getLocation();
            const rpos::core::Pose robotPose = pltfm.getPose();
            wkDat->robotPose = robotPose;

            // publish odom transform
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(robotPose.x(), robotPose.y(), 0.0));
            tf::Quaternion q = tf::createQuaternionFromYaw(robotPose.yaw());
            transform.setRotation(q);
            tfBrdcst.sendTransform(tf::StampedTransform(transform, ros::Time::now(), srvParams.odom_frame, srvParams.robot_frame));

            // send TF transform
            nav_msgs::Odometry msgRobotPose;
            msgRobotPose.header.frame_id = srvParams.odom_frame;
            msgRobotPose.header.stamp = ros::Time::now();
            sltcToRosMsg(robotPose, msgRobotPose.pose.pose);
            pubRobotPose_.publish(msgRobotPose);
        }

        //////////////////////////////////////////////////////////////////////////
