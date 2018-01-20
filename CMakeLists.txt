cmake_minimum_required(VERSION 2.8.3)
project(slamwareAPP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SDK_DIRS "${PROJECT_SOURCE_DIR}/SDK/slamware_sdk_linux-x86_64-gcc4.8")

set(SDK_DEBUG true)
if(SDK_DEBUG)
    set(SDK_INCLUDE_DIR "${SDK_DIRS}/linux-x86_64-debug")
    set(SDK_LIBRARY_DIR "${SDK_DIRS}/linux-x86_64-debug")
else()
    set(SDK_INCLUDE_DIR "${SDK_DIRS}/linux-x86_64-release")
    set(SDK_LIBRARY_DIR "${SDK_DIRS}/linux-x86_64-release")
endif()

find_package(Threads)
find_package(catkin REQUIRED COMPONENTS
  message_generation   roscpp   rospy   sensor_msgs   std_msgs   tf)
add_message_files(FILES
  Pose.msg
  Battery.msg
)

add_service_files(FILES
  GetPoseSrv.srv
  SaveMapSrv.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime)

include_directories(
  include ${catkin_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
   ${SDK_INCLUDE_DIR}/include
   ${PROJECT_SOURCE_DIR}/src
)
message("${INCLUDE_DIRECTORIES}")
message("${PROJECT_SOURCE_DIR}/src",${PROJECT_SOURCE_DIR}/src)
link_directories( ${SDK_LIBRARY_DIR}/lib
                  ${CMAKE_SOURCE_DIR})


# add_compile_options(-std=c++0x)
add_executable(slamwareNode
  src/TaskManager.cpp ./src/main.cpp ./src/slamware.cpp)

target_link_libraries(slamwareNode
        ${catkin_LIBRARIES}
        librpos_robotplatforms_rpslamware.a
        librpos_framework.a
        libbase64.a
        librlelib.a
        libjsoncpp.a
        libcurl.a
        libssl.a
        libcrypto.a
        libboost_atomic.a
        libboost_chrono.a
        libboost_date_time.a
        libboost_regex.a
        libboost_system.a
        libboost_thread.a
        libboost_filesystem.a
        pthread
)
