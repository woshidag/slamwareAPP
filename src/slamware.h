#ifndef SLAMWAREAPP_H_
#define SLAMWAREAPP_H_
/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

 yujikang  woshidag@yeah.net
   2018.1.10
*/
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>
#include "slamwareAPP/Battery.h"
#include "slamwareAPP/Pose.h"

#include "slamwareAPP/SaveMapSrv.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include <fstream>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <json/json.h>

#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/features/location_provider/map.h>
#include <rpos/robot_platforms/slamware_core_platform.h>
// #include <rpos/robot_platforms/slamware_core_platform.h>

#include "TaskManager.h"

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace rpos::system::types;
using namespace rpos::core;
using namespace rpos::robot_platforms::objects;

class slamware {
public:
    slamware();
    slamware(std::string ip);
    ~slamware();

    void init();
    bool connectSlamware();
    void startSlamwareWork();
    //pub
    void publishRobotPose(double publish_period);
    void publishScan(double publish_period);
    void publishMap(double publish_period);
    void publishPlanPath(float publish_period);
    void publishBattery(float publish_period);
    
    // sub callback
    void robotControlCallback (const geometry_msgs::TwistConstPtr &vel);
    void moveToGoalCallback (const geometry_msgs::PoseStamped::ConstPtr& goal);
    void guideCallback (const slamwareAPP::Pose::ConstPtr& pose);
    // service
    bool saveMapService (slamwareAPP::SaveMapSrv::Request& request, slamwareAPP::SaveMapSrv::Response& response);

    //func
    bool uploadMap();
    bool loadPose(rpos::core::Pose & pose);
    bool savePose(const rpos::core::Pose & pose);
    bool isNear(const rpos::core::Pose & p1, const rpos::core::Pose & p2, double range=0.5);
    void savePoseThread(double save_period);

    ros::Publisher scan_pub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher gridmap_pub_;
    ros::Publisher gridmap_info_pub_;
    ros::Publisher global_plan_pub_;

    ros::Publisher battery_pub_;
    ros::Publisher tts_pub_;

    ros::Subscriber robot_vel_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber guide_sub_;

    ros::ServiceServer save_map_service_;

private:
    void loopThreads();

private:
    ros::NodeHandle nh_;
    SlamwareCorePlatform SDP_;

    std::string ip_addres_;
    bool angle_compensate_;
    bool fixed_odom_map_tf_;
    float robot_pose_pub_period_;
    float scan_pub_period_;
    float map_pub_period_;
    float path_pub_period_;
    float battery_period_;
    float save_pose_period_;

    float map_size_down_left_x_;
    float map_size_down_left_y_;
    float map_size_width_;
    float map_size_height_;

    tf::TransformBroadcaster tfB_;

    boost::thread* robot_pose_pub_thread_;
    boost::thread* scan_pub_thread_;
    boost::thread* map_pub_thread_;
    boost::thread* transform_thread_;
    boost::thread* plan_path_pub_thread_;
    
    boost::shared_ptr<boost::thread> battery_pub_thread_;
    boost::shared_ptr<boost::thread> save_pose_thread_;

    std::string robot_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    std::string vel_control_topic_;
    std::string goal_topic_;
    std::string scan_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string map_info_topic_;
    std::string path_topic_;
    std::string battery_topic_;
    std::string guide_topic_;
    std::string tts_topic_;

    std::string save_map_srv_;


    TaskManager task_manager_;
};

#endif