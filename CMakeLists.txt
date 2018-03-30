cmake_minimum_required(VERSION 2.8.3)
project(ros_msgs_sync)

# Check that the compoiler has c++11 support
# Crash otherwise.
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  sensor_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport message_filters roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(demo_synchronizer sample/demo_synchronizer.cpp)
target_link_libraries(demo_synchronizer ${catkin_LIBRARIES})

if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(demo_synchronizer PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET demo_synchronizer PROPERTY CXX_STANDARD 11)
  set_property(TARGET demo_synchronizer PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET demo_synchronizer PROPERTY CXX_EXTENSIONS OFF)
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS demo_synchronizer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
