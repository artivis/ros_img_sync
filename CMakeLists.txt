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
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/sync_image_handler.cpp
  src/sync_image_transport_handler.cpp
  include/${PROJECT_NAME}/impl/sync_impl_transport_handler.cpp
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

## Declare a cpp executable

add_executable(img_trp_sync_save sample/sample_sync_save.cpp)
target_link_libraries(img_trp_sync_save
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(img_sync_disp sample/sample_sync_disp.cpp)
target_link_libraries(img_sync_disp
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

#add_executable(ptcld_sync_save sample/sample_sync_pointcloud.cpp)
#target_link_libraries(ptcld_sync_save
#   ${catkin_LIBRARIES} ${PROJECT_NAME}
#)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS img_trp_sync_save img_sync_disp ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

### Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#)

### Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
