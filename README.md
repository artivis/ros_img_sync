# ros_msgs_sync
## A couple utils to ease ros messages synchronization using the [`message_filters::Synchronizer`](http://wiki.ros.org/message_filters#Policy-Based_Synchronizer_.5BROS_1.1.2B-.5D)

## Package Summary
Provides a template class `MessageSynchronizerBase` which automically register and setup the whole pipeline
used by [`message_filters::Synchronizer`](http://wiki.ros.org/message_filters#Policy-Based_Synchronizer_.5BROS_1.1.2B-.5D) by
means of nasty metaprogramming.

The `MessageSynchronizerBase` can be used as a standalone or else by
inheriting from `MessageSynchronizerBase<my-list-of-msg-type>` one only has to overload
the virtual function `callback()` to manage to synchronized callback messages.

- Maintainer status: maintained
- Maintainer: name <jeremie.deray@pal-robotics.com>
- Author: name <jeremie.deray@pal-robotics.com>
- License: APACHE-2.0
- Bug / feature tracker: https://github.com/artivis/ros_msgs_sync/issues
- Source: git https://github.com/artivis/ros_msgs_sync (branch: master)

[![Build Status](https://travis-ci.org/name/package_name.svg?branch=master)](https://travis-ci.org/name/package_name)
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
```terminal
$ roslaunch ros_msgs_sync demo_synchronizer.launch
```

## Documentation and Tutorials

Minimal example:
```cpp
#include "ros_msgs_sync/ros_msgs_sync.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_synchronizer");

  ros_msgs_sync::SyncApproxImagesWithInfo msg_sync;

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

The package define the following useful synchronizer:

```cpp
MessageSynchronizerApprox<MsgType0, MsgType1, etc...>;

MessageSynchronizerExact<MsgType0, MsgType1, etc...>;

SyncApproxNImages my_img_synchronizer; // Where N is in [2 - 9]

SyncApproxImagesWithInfo;  // Sync sensor_msgs::Image + sensor_msgs::CameraInfo
SyncApprox2ImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo X 2
SyncApprox3ImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo X 3
SyncApprox4ImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo X 4

SyncExactImagesWithInfo; // Sync sensor_msgs::Image + sensor_msgs::CameraInfo
```

## Contributing

Please, feel free.
