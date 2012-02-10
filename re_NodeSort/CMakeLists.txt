cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_classpath("/home/gajan/workspace-java/probcog2835/bin")

add_runtime_classpath("/home/gajan/workspace-java/probcog2835/bin")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/proximity.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/optimization.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/weka_fipm.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/proximity3.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/bnj.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/ssj.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/jython250lib.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/profile.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/yprolog.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/bnj_res.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/jython22.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/tumutils.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/swt_linux_i386/swt.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/yarp_linux_amd64/jyarprpc.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/swt_linux_amd64/swt.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/swt_macosx64/swt.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/swt_macosx/swt.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/jpl/jpl.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/jpl/win32/jpl.jar")
add_runtime_classpath("/home/gajan/workspace-java/probcog2835/lib/swt_win32/swt.jar")

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
rospack_add_java_executable(nodeSort NodeSort)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
