#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from threading import Lock

g_pose = PoseWithCovarianceStamped()
mtx = Lock()

g_name = String()
mtx_name = Lock()

num = 0

def num_callback(msg):
    global num
    num = msg.data
    rospy.loginfo("Current num : %d",num)


def name_callback(msg):
    global g_name
    with mtx_name:
        g_name = msg

def pose_callback(msg):
    global g_pose
    with mtx:
        g_pose = msg
   
def cloud_callback(msg):
    rospy.loginfo("Received a point cloud")

    cur_pose = PoseWithCovarianceStamped()
    with mtx:
        cur_pose = g_pose

    cur_name = String()
    with mtx_name:
        cur_name = g_name
    
    rospy.loginfo("Current name : %s", cur_name.data)

    rospy.loginfo("Current PoseWithCovarianceStamped: [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f, %.2f]",
        cur_pose.pose.pose.position.x, cur_pose.pose.pose.position.y, cur_pose.pose.pose.position.z,
        cur_pose.pose.pose.orientation.x, cur_pose.pose.pose.orientation.y, cur_pose.pose.pose.orientation.z, cur_pose.pose.pose.orientation.w)
    
    rospy.sleep(5)  # 等待5秒模拟处理时间

    done_msg = Bool()
    done_msg.data = True
    done_pub.publish(done_msg)
    rospy.loginfo("done!!!!!!!!!")



def main():
    global done_pub, sub, sub_pose, sub_name

    rospy.init_node('sub_cloud')

    cloud_topic = rospy.get_param('cloud_topic', '/cloud_pcd')

    sub = rospy.Subscriber(cloud_topic, PointCloud2, cloud_callback)
    sub_pose = rospy.Subscriber('pose_topic', PoseWithCovarianceStamped, pose_callback)
    sub_name = rospy.Subscriber("map_name", String, name_callback)
    sub_num = rospy.Subscriber("map_num", Int32, num_callback)
    done_pub = rospy.Publisher('processing_done', Bool, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
