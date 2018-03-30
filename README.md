# ros_msgs_sync
## A couple utils to ease ros messages synchronization using the [`message_filters::Synchronizer`](http://wiki.ros.org/message_filters#Policy-Based_Synchronizer_.5BROS_1.1.2B-.5D)

## Package Summary
Provides a template class `MessageSynchronizerBase` which automically register and setup the whole pipeline
used by [`message_filters::Synchronizer`](http://wiki.ros.org/message_filters#Policy-Based_Synchronizer_.5BROS_1.1.2B-.5D) by
means of nasty metaprogramming.

The `MessageSynchronizerBase` can be used as a standalone or else by
inheriting from `MessageSynchronizerBase<my-list-of-msg-type>` one only has to overload
the virtual function `callback()` to manage to synchronized callback messages.

-   Maintainer status: maintained
-   Maintainer: name <deray.jeremie@gmail.com>
-   Author: name <deray.jeremie@gmail.com>
-   License: APACHE-2.0
-   Bug / feature tracker: https://github.com/artivis/ros_msgs_sync/issues
-   Source: git https://github.com/artivis/ros_msgs_sync (branch: master)

[![Build Status](https://travis-ci.org/artivis/ros_msgs_sync.svg?branch=master)](https://travis-ci.org/artivis/ros_msgs_sync)
---

## Quick Start

### Installation

#### Binaries
```terminal
todo...
#$ apt-get install ros-indigo-my-package
```

#### From source
```terminal
$ git clone https://github.com/artivis/ros_msgs_sync
$ catkin build ros_msgs_sync
```

#### Demo Example
The file `example/demo_synchronizer.cpp` together with it's launch file shows how one can use one of the synchronizer available with this package. The demo synchronizes 2 `sensor_msgs::Image` and 2 `sensor_msgs::CameraInfo` messages.

```terminal
$ roslaunch ros_msgs_sync demo_synchronizer.launch
```

## Documentation and Tutorials

As a minimal example, we will synchronize a `sensor_msgs::Image` with an associated `sensor_msgs::Camera` :
```cpp
#include "ros_msgs_sync/ros_msgs_sync.h"
#include <ros/ros.h>

/// Typef the synchronizer
/// Template arguments
/// 1. synchronization policy
/// 2-9. message types to be synchronized
using MySync =
  ros_msgs_sync::MessageSynchronizerBase<message_filters::sync_policies::ApproximateTime,
                                         sensor_msgs::Image, sensor_msgs::CameraInfo>;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_synchronizer");

  std::size_t image_queue = 5;
  std::size_t camera_info_queue = 5;

  MySync msg_sync(image_queue, camera_info_queue);

  msg_sync.start();

  sensor_msgs::ImageConstPtr image;
  sensor_msgs::CameraInfoConstPtr cam_info;

  while (ros::ok())
  {
    std::tie(image, cam_info) = msg_sync.getMessage();

    // Do stuff with messages.

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
```

That's it !  

For more details, an example is available in the `example` folder of the package together with its launch file.

### Notice

##### Topics :
Subscribers are listening to the topics :  
-   `~synchronized_topic_0`
-   `~synchronized_topic_1`
-   ...
-   `~synchronized_topic_N`

thus they need to be remapped, e.g. from a launch file
```xml
<remap from="~synchronized_topic_0" to="my_camera_topic"/>
```

##### Subscribers :

The class automatically instantiate a `image_transport::SubscriberFilter` for image messages - configurable through a `ImageSubcriberParameters` - while uses a `message_filters::Subscriber` for any other message types - configurable through a `SubcriberParameters`.

### More configuration

```cpp
#include "ros_msgs_sync/ros_msgs_sync.h"
#include <ros/ros.h>

/// Typef the synchronizer
/// Template arguments
/// 1. synchronization policy
/// 2-9. message types to be synchronized
using MySync =
  ros_msgs_sync::MessageSynchronizerBase<message_filters::sync_policies::ApproximateTime,
                                         sensor_msgs::Image, sensor_msgs::CameraInfo>;

int main(int argc, char** argv)
{
  // See image_transport::TransportHints documentation for more information.
  image_transport::TransportHints image_transport_hints;

  ros_msgs_sync::ImageSubcriberParameters image_sub_params;
  image_sub_params.queue_size = 5;
  image_sub_params.transport_hints = image_transport_hints;

  // See ros::TransportHints documentation for more information.
  ros::TransportHints transport_hints;

  MySync camera_info_sub_params;
  camera_info_sub_params.queue_size = 5;
  camera_info_sub_params.transport_hints = transport_hints;
  camera_info_sub_params.callback_queue = nullptr;

  ros_msgs_sync::SyncApproxImagesWithInfo msg_sync(image_sub_params, camera_info_sub_params);

  // Set the callback queue size of the synchronizer.
  msg_sync.setSyncQueueSize(5);

  msg_sync.start();

  ...

  // subscribers parameters may be changed later on.
  msg_sync.setQueueSize<0>(10);
  msg_sync.getSubscriberParameters<1>().queue_size = 10;

  // However one need to stop/start the synchronizer
  // for the change to take effects

  msg_sync.restart();

  ...

  return EXIT_SUCCESS;
}
```

### Pre-defined synchronizers

The header `ros_msgs_sync.h` defines the following useful synchronizer:

```cpp
SyncApprox2Images;
SyncApproxNImages; // Where N is in [2 - 9]

SyncApproxImagesWithInfo;  // Sync sensor_msgs::Image + sensor_msgs::CameraInfo
SyncApprox2ImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo X 2
SyncApprox3ImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo X 3
SyncApprox4ImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo X 4

SyncExactImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo
```

Notice the following helper types:  

```cpp
MessageSynchronizerApprox<MsgType0, MsgType1, etc...>;

MessageSynchronizerExact<MsgType0, MsgType1, etc...>;
```

<!--Say one wants to synchronize 3 `sensor_msgs::PointCloud2` :

```cpp
using SyncApprox3PointCloud = MessageSynchronizerApprox<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,sensor_msgs::PointCloud2>;

SyncApprox3PointCloud my_pcl_sync;

```
-->

## Contributing

Please, feel free.
