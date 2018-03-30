/** \author Jeremie Deray. */

#include "ros_msgs_sync/ros_msgs_sync.h"

// ROS headers
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_synchronizer");

  ros_msgs_sync::ImageSubcriberParameters first_sub_param{1};

  ros_msgs_sync::SyncApprox2ImagesWithInfo msg_sync(first_sub_param,
                                                    ros_msgs_sync::SubcriberParameters(2),
                                                    3, 4);

  ROS_INFO_STREAM("Param 0 : " << msg_sync.getSubscriberParameters<0>().queue_size);
  ROS_INFO_STREAM("Param 1 : " << msg_sync.getSubscriberParameters<1>().queue_size);
  ROS_INFO_STREAM("Param 2 : " << msg_sync.getSubscriberParameters<2>().queue_size);
  ROS_INFO_STREAM("Param 3 : " << msg_sync.getSubscriberParameters<3>().queue_size);

  ROS_INFO("Starting synchronization !");
  msg_sync.start();

  sensor_msgs::ImageConstPtr image_0;
  sensor_msgs::CameraInfoConstPtr image_info_0;
  sensor_msgs::ImageConstPtr image_1;
  sensor_msgs::CameraInfoConstPtr image_info_1;

  const auto elapsed = ros::Duration(3);
  auto start = ros::Time::now();

  while (ros::ok() && (ros::Time::now()-start < elapsed))
  {
    std::tie(image_0, image_info_0, image_1, image_info_1) = msg_sync.getMessage();

    // Do stuff with messages.

    ros::spinOnce();
  }

  ROS_INFO("Stopping synchronization !");
  msg_sync.stop();
  ros::Duration(3).sleep();

  msg_sync.setQueueSize<0>(10);
  msg_sync.setQueueSize<1>(10);
  msg_sync.getSubscriberParameters<2>().queue_size = 10;
  msg_sync.getSubscriberParameters<3>().queue_size = 10;

  ROS_INFO_STREAM("Param 0 : " << msg_sync.getSubscriberParameters<0>().queue_size);
  ROS_INFO_STREAM("Param 1 : " << msg_sync.getSubscriberParameters<1>().queue_size);
  ROS_INFO_STREAM("Param 2 : " << msg_sync.getSubscriberParameters<2>().queue_size);
  ROS_INFO_STREAM("Param 3 : " << msg_sync.getSubscriberParameters<3>().queue_size);

  ROS_INFO("Starting synchronization !");
  msg_sync.start();

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now()-start < elapsed))
  {
    std::tie(image_0, image_info_0, image_1, image_info_1) = msg_sync.getMessage();

    // Do stuff with messages.

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
