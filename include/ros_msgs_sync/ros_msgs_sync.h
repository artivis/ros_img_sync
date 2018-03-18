/** \author Jeremie Deray. */

#ifndef _ROS_MSGS_SYNC_ROS_MSGS_SYNC_H_
#define _ROS_MSGS_SYNC_ROS_MSGS_SYNC_H_

#include "ros_msgs_sync/message_synchronizer_base.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

namespace ros_msgs_sync{

template <typename... Args>
using MessageSynchronizerApprox =
ros_msgs_sync::MessageSynchronizerBase<message_filters::sync_policies::ApproximateTime, Args...>;

template <typename... Args>
using MessageSynchronizerExact =
ros_msgs_sync::MessageSynchronizerBase<message_filters::sync_policies::ExactTime, Args...>;

/// @todo define some macro / meta to ease these kind of typedef.

using SyncApprox2Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image>;
using SyncApprox3Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image>;
using SyncApprox4Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image>;
using SyncApprox5Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image>;
using SyncApprox6Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image>;
using SyncApprox7Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image>;
using SyncApprox8Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image>;
using SyncApprox9Images = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image,sensor_msgs::Image,
                                                                   sensor_msgs::Image>;

using SyncApproxImagesWithInfo  = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::CameraInfo>;
using SyncApprox2ImagesWithInfo = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::CameraInfo,
                                                                           sensor_msgs::Image,sensor_msgs::CameraInfo>;
using SyncApprox3ImagesWithInfo = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::CameraInfo,
                                                                           sensor_msgs::Image,sensor_msgs::CameraInfo,
                                                                           sensor_msgs::Image,sensor_msgs::CameraInfo>;
using SyncApprox4ImagesWithInfo = ros_msgs_sync::MessageSynchronizerApprox<sensor_msgs::Image,sensor_msgs::CameraInfo,
                                                                           sensor_msgs::Image,sensor_msgs::CameraInfo,
                                                                           sensor_msgs::Image,sensor_msgs::CameraInfo,
                                                                           sensor_msgs::Image,sensor_msgs::CameraInfo>;

using SyncExactImagesWithInfo  = ros_msgs_sync::MessageSynchronizerExact<sensor_msgs::Image,sensor_msgs::CameraInfo>;

} /* namespace ros_msgs_sync */

#endif /* _ROS_MSGS_SYNC_ROS_MSGS_SYNC_H_ */
