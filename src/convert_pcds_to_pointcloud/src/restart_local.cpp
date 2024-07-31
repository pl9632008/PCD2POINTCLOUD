#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <atomic>
#include <thread>

std::mutex mtx_odometry;
double g_cur_x = 0;
double g_cur_y = 0;
double g_cur_z = 0;

double g_init_x = 0;
double g_init_y = 0;
double g_init_z = 0;
bool g_init_flag = false;
double distance_thresh;
ros::Subscriber sub_odometry;
ros::Subscriber sub_restart;
int count_thresh;
int cnt = 0;

float vel_thresh;


void killAndRestart(){

    


        std::string restart_cmd2 = "xterm -e  roslaunch convert_pcds_to_pointcloud convert_local.launch";
        std::system(restart_cmd2.c_str());
}


void restartCallback(const std_msgs::Bool::ConstPtr& msg){
    // std::lock_guard<std::mutex> lock(mtx_odometry);

    // bool flag = msg->data;
    // if(flag){

    //     double diff_x = g_cur_x - g_init_x;
    //     double diff_y = g_cur_y - g_init_y;
    //     double diff_z = g_cur_z - g_init_z;
    //     double distance = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);

    //     if(distance > distance_thresh){

    //         g_init_x = 0;
    //         g_init_y = 0;
    //         g_init_z = 0;

    //         std::thread t(killAndRestart);
    //         t.detach();

    //         // std::string kill_cmd = "xterm -e rosnode kill /global_localization /laserMapping /transform_fusion /pub_cloud /sub_odometry /rviz";
    //         // std::system(kill_cmd.c_str());

    //         // // std::string restart_cmd = "xterm -e  roslaunch fast_lio_localization localization_horizon_test.launch";
    //         // // std::system(restart_cmd.c_str());

    //         // std::string restart_cmd2 = "xterm -e  roslaunch convert_pcds_to_pointcloud convert_local.launch";
    //         // std::system(restart_cmd2.c_str());

    //         ROS_ERROR("IN INSTANCE");
    //     }else{

    //         ROS_WARN("GREATER DISTANCE_THRESHOLD");
    //     }


    // }else{

    //     ROS_INFO("NO FLAG!");

    // }



}



void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_odometry);
    if(g_init_flag == false){
        g_init_x = msg->pose.pose.position.x;
        g_init_y = msg->pose.pose.position.y;
        g_init_z = msg->pose.pose.position.z;
        g_init_flag = true;

    }else{

        g_cur_x = msg->pose.pose.position.x;
        g_cur_y = msg->pose.pose.position.y;
        g_cur_z = msg->pose.pose.position.z;
    }


    double vec_x = msg->twist.twist.linear.x;
    double vec_y = msg->twist.twist.linear.y;
    double vec_z = msg->twist.twist.linear.z;



    double diff_x = g_cur_x - g_init_x;
    double diff_y = g_cur_y - g_init_y;
    double diff_z = g_cur_z - g_init_z;
    double distance = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);


    double vec_total = std::sqrt( vec_x*vec_x + vec_y*vec_y  + vec_z*vec_z );
    if( vec_total *3.6 < vel_thresh){
        cnt++;
    }else{
        cnt=0;
    }


    if(cnt > count_thresh ){

            if(distance > distance_thresh){

                g_init_x = 0;
                g_init_y = 0;
                g_init_z = 0;
                cnt = 0;

                std::string kill_cmd = "rosnode kill /global_localization /laserMapping /transform_fusion /pub_cloud /sub_odometry /rviz";
                std::system(kill_cmd.c_str());

                std::thread t(killAndRestart);
                t.detach();
            }


    }





    ROS_INFO("x = %f , y = %f , z = %f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    

    
}



int main(int argc, char* argv[]){

    ros::init(argc, argv, "pub_cloud");
    ros::NodeHandle nh;
    ros::param::get("distance_thresh", distance_thresh);
        ros::param::get("count_thresh",count_thresh);

    ros::param::get("vel_thresh",vel_thresh);

    ros::Subscriber sub_odometry;
    sub_odometry = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, odometryCallback);
    sub_restart = nh.subscribe<std_msgs::Bool>("/restart_flag", 1, restartCallback);
    ros::spin();

}