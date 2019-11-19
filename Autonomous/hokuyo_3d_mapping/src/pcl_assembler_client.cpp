/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>

// Services
#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>

// Messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

/* This is a modified periodic_snapshotter.cpp file that acts both as a timer and subscriber, depending on the value of the
assembled_cloud_mode parameter.  It defaults to subscriber.  The node calls a service from the point_cloud2_assembler node
from the laser_assembler package to create the compiled point cloud and then publishes the result.
Timer:
As a timer, the node performs its function every 5 seconds.  To use this function, the assembled_cloud_mode parameter must
be set to time.
Subscriber:
As a subscriber, the node subscribes to start and end times published while tilting the motor and sends those as necessary
to the compilation service to put all of the point clouds together.  One cloud is published for each complete sweep
(i.e. -90 -> +90 -> -90).  This is the default setting of the node.*/

namespace laser_assembler
{

//compilation class created by combine_clouds with modifications to remove timer and work with motor times
class PeriodicSnapshotter {

    public:

    PeriodicSnapshotter() {
        // Create a publisher for the clouds that we assemble
        pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

        // Create the service client for calling the assembler
        client_ = n_.serviceClient<AssembleScans>("assemble_scans");

        // Start the timer that will trigger the processing loop (timerCallback)
        timer_ = n_.createTimer(ros::Duration(12,5), &PeriodicSnapshotter::timerCallback, this);

        // Need to track if we've called the timerCallback at least once
        first_time_ = true;
    }

    void timerCallback(const ros::TimerEvent& e) {
    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet

        if (first_time_) {
            first_time_ = false;
            return;
        }

        // Populate our service request based on our timer callback times
        AssembleScans srv;
        srv.request.begin = e.last_real;
        srv.request.end   = e.current_real;

        // Make the service call
        if (client_.call(srv)) {
            ROS_INFO("Published cloud 2 with %u points", (uint32_t)(srv.response.cloud.points.size()));
            pub_.publish(srv.response.cloud);
        }

        else {
            ROS_ERROR("Error making service call\n") ;
        }
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::ServiceClient client_;
    ros::Timer timer_;
    bool first_time_;
} ;

}

using namespace laser_assembler;

//main
int main(int argc, char **argv)
{
    //initialize and wait for necessary services, etc.
    ros::init(argc, argv, "Cloud_Compiler");
    ros::NodeHandle n;
    ROS_INFO("Waiting for [build_cloud] to be advertised");
    ros::service::waitForService("build_cloud");
    ROS_INFO_STREAM("Found build_cloud! Starting the Cloud Compiler");
    PeriodicSnapshotter snapshotter;
    ros::spin();
    return 0;

}
