#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

ros::Publisher done_pub; 
ros::Subscriber sub;
ros::Subscriber sub_pose;
ros::Subscriber sub_name;
ros::Subscriber sub_num;

std::vector<float> pose;
std::mutex mtx;
geometry_msgs::PoseWithCovarianceStamped g_pose;

std_msgs::String g_name;
std::mutex mtx_name;


int num = 0;

void numCallback(const std_msgs::Int32::ConstPtr& msg){
    num = msg->data;
    ROS_INFO("current num : %d", num);

}

void nameCallback(const std_msgs::String::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(mtx_name);
    g_name = *msg;
}


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    std::lock_guard<std::mutex> lock(mtx);
    g_pose = *msg;

}


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("Received a point cloud");

    geometry_msgs::PoseWithCovarianceStamped cur_pose;
    {
        std::lock_guard<std::mutex> lock(mtx);
        cur_pose = g_pose;
    }

    std_msgs::String cur_name;
    {
        std::lock_guard<std::mutex> lock(mtx_name);
        cur_name = g_name;
    }

    ROS_INFO("current name : %s", cur_name.data.c_str());

    ROS_INFO("Current PoseWithCovarianceStamped: [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f, %.2f]",
            cur_pose.pose.pose.position.x, cur_pose.pose.pose.position.y, cur_pose.pose.pose.position.z,
            cur_pose.pose.pose.orientation.x, cur_pose.pose.pose.orientation.y,
            cur_pose.pose.pose.orientation.z, cur_pose.pose.pose.orientation.w);



    ros::Duration(5).sleep();  

    std_msgs::Bool done_msg;
    done_msg.data = true;
    done_pub.publish(done_msg);
    ROS_INFO("done!!!!!!!!!");

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_cloud");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.getParam("cloud_topic", cloud_topic);

    sub = nh.subscribe(cloud_topic, 1, cloudCallback);
    sub_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pose_topic", 1, poseCallback);
    sub_name = nh.subscribe<std_msgs::String>("/map_name", 1, nameCallback);
    sub_num = nh.subscribe<std_msgs::Int32>("/map_num", 1, numCallback);
    done_pub = nh.advertise<std_msgs::Bool>("/processing_done", 1); 


    ros::spin();

    return 0;
}