/** \author Jeremie Deray. */

#include "ros_msgs_sync/ros_msgs_sync.h"

// ROS headers
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_synchronizer");

  ros_msgs_sync::SubcriberParameters first_sub_param{.queue_size_=5, .transport_hints=ros::TransportHints()};

  ros_msgs_sync::SyncApprox2ImagesWithInfo msg_sync(first_sub_param, 5, 5, 5);

  msg_sync.setQueueSize<3>(10);
  msg_sync.setQueueSize<2>(10);

  ROS_INFO("Starting synchronization !");
  msg_sync.start();

  sensor_msgs::ImageConstPtr image_0;
  sensor_msgs::CameraInfoConstPtr image_info_0;
  sensor_msgs::ImageConstPtr image_1;
  sensor_msgs::CameraInfoConstPtr image_info_1;

  while (ros::ok())
  {
    std::tie(image_0, image_info_0, image_1, image_info_1) = msg_sync.getMessage();

    // Do stuff with messages.

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
