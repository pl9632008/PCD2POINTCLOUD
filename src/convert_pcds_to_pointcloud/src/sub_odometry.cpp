#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <iostream>

#define BLUE "\033[1;34m"
#define RESET "\033[0m"
ros::Publisher pub_vec;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double vec_x = msg->twist.twist.linear.x;
    double vec_y = msg->twist.twist.linear.y;
    double vec_z = msg->twist.twist.linear.z;

    double vec_total = std::sqrt( vec_x*vec_x + vec_y*vec_y  + vec_z*vec_z );

    ROS_WARN("Current vec_total = %f m/s", vec_total);
    ROS_INFO(BLUE "Current vec_total = %f km/h" RESET, vec_total*3.6);

    std_msgs::Float64 msg_vec;
    msg_vec.data = vec_total*3.6;
    pub_vec.publish(msg_vec);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Odometry", 1, odometryCallback);
    pub_vec = nh.advertise<std_msgs::Float64>("/current_velocity", 1);

    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
