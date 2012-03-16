# Install script for directory: /home/gajan/ros/stacks/sharedLocal/simplePendulum

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/simplePendulum:/opt/ros/electric/stacks/vision_opencv/cv_bridge/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/electric/stacks/image_common/image_transport/lib:/opt/ros/electric/stacks/common_msgs/sensor_msgs/lib:/opt/ros/electric/stacks/ros_comm/tools/rosbag/lib:/opt/ros/electric/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/electric/stacks/pluginlib/lib:/opt/ros/electric/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/electric/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/electric/stacks/ros_comm/utilities/rostime/lib:/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/electric/ros/core/roslib/lib:/opt/ros/electric/ros/tools/rospack/lib:/opt/ros/electric/stacks/orocos_toolchain/ocl/lib:/opt/ros/electric/stacks/orocos_toolchain/log4cpp/../install/lib:/home/gajan/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir/lib:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/types:/opt/ros/electric/stacks/orocos_toolchain/install/lib:/opt/ros/electric/stacks/orocos_toolchain/rtt/install/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum" TYPE SHARED_LIBRARY FILES "/home/gajan/ros/stacks/sharedLocal/simplePendulum/lib/orocos/gnulinux/libsimplePendulum-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so"
         OLD_RPATH "/opt/ros/electric/stacks/vision_opencv/cv_bridge/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/electric/stacks/image_common/image_transport/lib:/opt/ros/electric/stacks/common_msgs/sensor_msgs/lib:/opt/ros/electric/stacks/ros_comm/tools/rosbag/lib:/opt/ros/electric/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/electric/stacks/pluginlib/lib:/opt/ros/electric/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/electric/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/electric/stacks/ros_comm/utilities/rostime/lib:/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/electric/ros/core/roslib/lib:/opt/ros/electric/ros/tools/rospack/lib:/opt/ros/electric/stacks/orocos_toolchain/ocl/lib:/opt/ros/electric/stacks/orocos_toolchain/log4cpp/../install/lib:/home/gajan/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir/lib:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/types:/opt/ros/electric/stacks/orocos_toolchain/install/lib:/opt/ros/electric/stacks/orocos_toolchain/rtt/install/lib::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/simplePendulum:/opt/ros/electric/stacks/vision_opencv/cv_bridge/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/electric/stacks/image_common/image_transport/lib:/opt/ros/electric/stacks/common_msgs/sensor_msgs/lib:/opt/ros/electric/stacks/ros_comm/tools/rosbag/lib:/opt/ros/electric/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/electric/stacks/pluginlib/lib:/opt/ros/electric/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/electric/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/electric/stacks/ros_comm/utilities/rostime/lib:/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/electric/ros/core/roslib/lib:/opt/ros/electric/ros/tools/rospack/lib:/opt/ros/electric/stacks/orocos_toolchain/ocl/lib:/opt/ros/electric/stacks/orocos_toolchain/log4cpp/../install/lib:/home/gajan/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir/lib:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/electric/stacks/orocos_toolchain/install/lib/orocos/gnulinux/ocl/types:/opt/ros/electric/stacks/orocos_toolchain/install/lib:/opt/ros/electric/stacks/orocos_toolchain/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/simplePendulum/libsimplePendulum-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/gajan/ros/stacks/sharedLocal/simplePendulum/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/gajan/ros/stacks/sharedLocal/simplePendulum/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
