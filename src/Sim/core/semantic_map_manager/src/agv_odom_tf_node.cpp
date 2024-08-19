#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


using namespace std;
ros::Subscriber odom_sub_;

double x=0;
double y=0;
double th=0;
geometry_msgs::Quaternion quat_msg;
int odom_recieved_flag=0;
double main_freq=120;
string odom_frame_name_="map";
string baselink_frame_name_="base_link";
ros::Time time_last;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{   
  if (time_last == odom_msg->header.stamp) return;
   x=odom_msg->pose.pose.position.x;
   y=odom_msg->pose.pose.position.y;
   quat_msg=odom_msg->pose.pose.orientation;

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = odom_msg->header.stamp;
  odom_trans.header.frame_id = odom_frame_name_;
  odom_trans.child_frame_id = baselink_frame_name_;

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = quat_msg;

  //send the transform
  tf::TransformBroadcaster odom_broadcaster_;
  odom_broadcaster_.sendTransform(odom_trans);
  time_last = odom_msg->header.stamp;
  ROS_INFO("odom tf the delay is %f", (ros::Time::now()-odom_msg->header.stamp).toSec());

   odom_recieved_flag=0;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agv_odomtf_node");
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  nh_.getParam("/agv_odomtf_node/odom_frame_name", odom_frame_name_);
  nh_.getParam("/agv_odomtf_node/baselink_frame_name", baselink_frame_name_);


  odom_sub_=nh_.subscribe<nav_msgs::Odometry>("odom", 200, OdomCallback);
  tf::TransformBroadcaster odom_broadcaster;

  
  ros::Rate loop_rate(main_freq);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  time_last = ros::Time::now();
  
  while(ros::ok())
    {
    current_time = ros::Time::now();
           //since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

 if(odom_recieved_flag==1)
 {  
  
//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_name_;
    odom_trans.child_frame_id = baselink_frame_name_;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quat_msg;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    odom_recieved_flag=0;
    last_time = current_time;
}
     ros::spinOnce();
    loop_rate.sleep();
    
    }//end while(ros::ok())

}