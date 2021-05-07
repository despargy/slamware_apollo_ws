/*
 * Copyright 2021 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH)
 *		 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <apollo_lidar/apollo_lidar_ros.h>
#include <math.h>


void apollo_lidar_ros::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg0, const sensor_msgs::LaserScan::ConstPtr& msg1)
{
    sensor_msgs::LaserScan scan_in0 = *msg0;
    sensor_msgs::LaserScan scan_in1 = *msg1;

    sensor_msgs::LaserScan scan_out;

    scan_out.header.frame_id = "laser";
    scan_out.header.stamp = ros::Time::now();
    scan_out.angle_min = fmin(scan_in0.angle_min,scan_in1.angle_min);
    scan_out.angle_max = fmax(scan_in0.angle_max,scan_in1.angle_max);
    scan_out.angle_increment = fmax(scan_in0.angle_increment,scan_in1.angle_increment);
    scan_out.time_increment =  fmax(scan_in0.time_increment,scan_in1.time_increment);
    scan_out.scan_time = fmax(scan_in0.scan_time,scan_in1.scan_time);
    scan_out.range_min = fmin(scan_in0.range_min,scan_in1.range_min);
    scan_out.range_max = fmax(scan_in0.range_max,scan_in1.range_max);
    int j = 0;

    scan_out.ranges.resize(scan_in0.ranges.size()+scan_in1.ranges.size());
    for(unsigned int i = 0; i < scan_in0.ranges.size(); ++i)
    {
      //if( (scan_in.ranges[i] == scan_in.ranges[i]) && !isinf(scan_in.ranges[i]) )
      //{
          scan_out.ranges[j] = scan_in0.ranges[i];
          j++;
      //}
    }
     for(unsigned int i = 0; i < scan_in1.ranges.size(); ++i)
    {
          scan_out.ranges[j] = scan_in1.ranges[i];
          j++;
    }
    pub.publish(scan_out);


}

void apollo_lidar_ros::init(ros::NodeHandle node_handle_)
{
    nh = node_handle_;

    ros::NodeHandle n_p("~");
    n_p.param<std::string>("scan_topic0", scan_topic0, "/inter/scan");
    n_p.param<std::string>("scan_topic1", scan_topic1, "/scan");

    laser_sub0 = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_topic0, 1);
    laser_sub1 = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_topic1, 1);
    pub = nh.advertise<sensor_msgs::LaserScan>("apollo/scan",10);
}

void apollo_lidar_ros::run()
{
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), *laser_sub0, *laser_sub1);
    sync.registerCallback(boost::bind(&apollo_lidar_ros::processLaserScan, this, _1, _2));
    ros::spin();
}
