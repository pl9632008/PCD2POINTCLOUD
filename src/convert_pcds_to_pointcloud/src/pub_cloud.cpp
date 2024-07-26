#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <mutex>
#include <iostream>
#include <atomic>

#define BLUE "\033[1;34m"
#define RESET "\033[0m"

ros::Publisher pub_initial;
ros::Publisher pub_cloud;
ros::Publisher pub_pose;
ros::Publisher pub_pose_arr;
ros::Publisher pub_name;
ros::Publisher pub_num;
ros::Subscriber sub_final;
ros::Subscriber sub_vel;
ros::Subscriber sub_done;
ros::Subscriber sub_odometry;
ros::ServiceClient client;

std::atomic<bool> doing_loadandpublish = true;
std::atomic<bool> processing_done = false;
std::atomic<bool> select_map_flag = false;
std::atomic<int> cur_map_cnt = 0;
std::atomic<int> cnt = 0;
float vel_thresh;
int count_thresh;
int test_maps_size;

std::mutex mtx_odometry;
double g_cur_x = 0;
double g_cur_y = 0;
double g_cur_z = 0;
bool init_flag = false;
double distance_thresh = 0;
double g_before_x = 0;
double g_before_y = 0;
double g_before_z = 0;

std::vector<std::string> listFiles(const std::string& directory,const std::string & ext) {
    std::vector<std::string> total_names;
    std::filesystem::path p(directory);
    for(auto & entry : std::filesystem::directory_iterator(p)){
        if(entry.path().extension().string() == ext){
            total_names.push_back(entry.path().string());
        }
    }
    return total_names;
}


void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_odometry);
    g_cur_x = msg->pose.pose.position.x;
    g_cur_y = msg->pose.pose.position.y;
    g_cur_z = msg->pose.pose.position.z;
    
}





void finalNameCallback(const std_msgs::String::ConstPtr& msg){

    std::string final_name = msg->data;

}


void velocityCallback(const std_msgs::Float64::ConstPtr& msg){

    if(doing_loadandpublish){
        select_map_flag = false;
        cnt = 0;
        ROS_INFO("selecting maps now, do not count velocity return");
        return;
    }

   double velocity =  msg->data;

   if(velocity < vel_thresh ){
        cnt++;
   }else{
        select_map_flag = false;
        cnt = 0;
   }
 
    if(cnt > count_thresh){
        select_map_flag = true;
    }
    ROS_INFO("velocity count = %d",cnt.load());

}


void doneCallback(const std_msgs::Bool::ConstPtr& msg) {

    processing_done = msg->data ;
    cur_map_cnt++;
    ROS_INFO("processing_done = %s , already done = %d / %d ", processing_done.load() ? "true":"false", cur_map_cnt.load(), test_maps_size);
}

void loadAndPublish(const std::vector<std::string>& maps, ros::NodeHandle& nh) {

    if(init_flag==false){

        std::lock_guard<std::mutex> lock(mtx_odometry);
        g_before_x = g_cur_x;
        g_before_y = g_cur_y;
        g_before_z = g_cur_z;
        init_flag = true;
    }else {

        std::lock_guard<std::mutex> lock(mtx_odometry);
        double diff_x = g_cur_x - g_before_x;
        double diff_y = g_cur_y - g_before_y;
        double diff_z = g_cur_z - g_before_z;
        double distance = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);

        g_before_x = g_cur_x;
        g_before_y = g_cur_y;
        g_before_z = g_cur_z;


        if(distance < distance_thresh){
            ROS_INFO("moving  %f m, less than %f m, pass loadAndPublish", distance, distance_thresh);
            return;
        }else{

            ROS_INFO("moving  %f m, greater than %f m, doing loadAndPublish", distance, distance_thresh);

        }
    }
 


    std_srvs::SetBool srv;
    srv.request.data = true;  

    ROS_INFO("waiting for find_map start...");

    while(!client.call(srv)){
        ROS_WARN("Please roslaunch fast_lio_localization localization_horizon_test.launch !");
        ros::Duration(1).sleep();
    }
    ROS_INFO(BLUE "Response: success=%s, message=%s" RESET, srv.response.success ? "true" : "false", srv.response.message.c_str());


    ros::Rate rate(100);  

    ros::Duration(0.5).sleep();

    std::vector<std::string> test_maps;

    for(auto map_pcd : maps){
        std::string map_name = std::filesystem::path(map_pcd).stem().string();
        std::vector<float> pose;
        auto get_name_succeed = nh.getParam(map_name, pose);
        if(!get_name_succeed){
            ROS_WARN("%s does not have initial pose in config.yaml, passes this map!" , map_name.c_str());
        }else{
            test_maps.push_back(map_pcd);
        }
    }
    
    test_maps_size = test_maps.size();
    std_msgs::Int32 num_msg;
    num_msg.data = test_maps_size;
    pub_num.publish(num_msg);
    cur_map_cnt = 0;
 
    doing_loadandpublish = true;
    for (const auto& map_pcd : test_maps) {

        std::string map_name = std::filesystem::path(map_pcd).stem().string();
        ROS_INFO("map_name = %s", map_name.c_str());
        std::vector<float> pose;
        nh.getParam(map_name, pose);
         
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(map_pcd, *cloud) < 0) {
            ROS_ERROR_STREAM("Failed to parse pointcloud from file '" << map_pcd << "'");
            continue;
        }
        
        std_msgs::String name_msg;
        name_msg.data = map_name;
        pub_name.publish(name_msg);


        std_msgs::Float64MultiArray pose_array_msg;
        for(int i = 0; i < pose.size(); i++){
            pose_array_msg.data.push_back(pose[i]);
        }
        pub_pose_arr.publish(pose_array_msg);


        // geometry_msgs::PoseWithCovarianceStamped pose_msg;
        // pose_msg.pose.pose.position.x = pose[0];
        // pose_msg.pose.pose.position.y = pose[1];
        // pose_msg.pose.pose.position.z = pose[2];
        // pose_msg.pose.pose.orientation.x = pose[3];
        // pose_msg.pose.pose.orientation.y = pose[4];
        // pose_msg.pose.pose.orientation.z = pose[5];
        // pose_msg.pose.pose.orientation.w = pose[6];
        // pose_msg.header.stamp = ros::Time::now();
        // pose_msg.header.frame_id = "map";
        // pub_pose.publish(pose_msg);    

        // ROS_INFO("Current PoseWithCovarianceStamped: [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f, %.2f]",
        //         pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z,
        //         pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,
        //         pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w);


        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        std::string frame_id;
        nh.param<std::string>("frame_id", frame_id, "");
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = ros::Time::now();  // 设置当前时间戳

        ROS_INFO_STREAM(" * Loaded pointcloud from file: " << map_pcd);
        ROS_INFO_STREAM(" * Number of points: " << cloud->width * cloud->height);
        ROS_INFO_STREAM(" * Total size [bytes]: " << cloud_msg.data.size());
        
        pub_cloud.publish(cloud_msg);
        ROS_INFO_STREAM(" * pub_cloud cloud done!");

        
        while (!processing_done) {
            ros::spinOnce();
            rate.sleep();
        }
        processing_done = false;  // 重置标志，准备发布下一个点云

    }
    doing_loadandpublish = false;
}


int main(int argc, char* argv[]) {
    

    ros::init(argc, argv, "pub_cloud");
    ros::NodeHandle nh;

    std::string map_path;
    if (!nh.getParam("map_path", map_path)) {
        ROS_ERROR("Failed to get parameter 'map_path'");
        return 1;
    }

    ros::param::get("vel_thresh",vel_thresh);
    ros::param::get("count_thresh",count_thresh);
    ros::param::get("distance_thresh",distance_thresh);

    std::string cloud_topic;
    nh.getParam("cloud_topic",  cloud_topic);

    sub_vel = nh.subscribe("/current_velocity",100, velocityCallback);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);
    sub_done = nh.subscribe("/processing_done", 1, doneCallback);
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_topic", 1);
    pub_pose_arr = nh.advertise<std_msgs::Float64MultiArray>("/pose_arr_topic",1);
    pub_name = nh.advertise<std_msgs::String>("/map_name", 1);
    pub_num = nh.advertise<std_msgs::Int32>("/map_num", 1);
    sub_final = nh.subscribe("/final_name", 1, finalNameCallback);
    pub_initial = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    sub_odometry = nh.subscribe<nav_msgs::Odometry>("/Odometry",10,odometryCallback);

    client = nh.serviceClient<std_srvs::SetBool>("/set_bool");

    auto maps = listFiles(map_path, ".pcd");

    if (maps.empty()) {
        ROS_WARN("No PCD files found in directory '%s'", map_path.c_str());
        return 0;
    } else{
        loadAndPublish(maps, nh);
    }

    ros::Rate rate(100);  

    while(ros::ok() ){
        if(select_map_flag&& !doing_loadandpublish){
            loadAndPublish(maps, nh);
            select_map_flag = false;
            cnt=0;
        }
        ros::spinOnce();
        rate.sleep();
    }

}