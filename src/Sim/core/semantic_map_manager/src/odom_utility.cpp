#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "vehicle_msgs/encoder.h"
#include "vehicle_msgs/decoder.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <people_msgs/People.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Float32MultiArray.h"

using namespace Eigen;
using Json = nlohmann::json;

ros::Timer SetpointpubCBTimer_;
void SetpointpubCB(const ros::TimerEvent& e);
void lines_cb(const visualization_msgs::Marker::ConstPtr& msg);
void TimerCallback(const ros::TimerEvent&);
void pubgoal_cb(const std_msgs::Bool::ConstPtr& msg);
void globalplan_cb(const nav_msgs::Path::ConstPtr& msg);

std::string vehicle_set_path_;
common::VehicleSet vehicle_set_;
ros::Publisher arena_info_dynamic_pub_, surround_traj_pub, arena_info_static_pub_, goal_pub_, sgoal_pub_, static_obst_pub;
bool ParseVehicleSet(common::VehicleSet *p_vehicle_set);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void people_cb(const people_msgs::People::ConstPtr& msg);
void peopleAngleCallback(const std_msgs::Float32MultiArray::ConstPtr& angle_msg);
void resetCallback(const std_msgs::Int32::ConstPtr& msg);

double calculateOverlapPercentage(double A, double B, double C, double D);
double closestDistance(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C); 

std::string target_frame_ = "/map";
std::string people_frame_ = "base_link";
ros::Publisher vis_pub_, peopledens_pub_;
tf::TransformListener* pListener = NULL;
double angularZ_ = 0.0;
double pre_time = 5.0, deltatime = 0.5;
visualization_msgs::Marker::ConstPtr line_msg_;
bool initialized_=false;
int current_leg_ = 0;
double goal_radius_ = 0.5;
double shortterm_dist_ = 3.0;
nav_msgs::Path path_msg_;
ros::Publisher people_marker_publisher_, line_marker_publisher_;
std::vector<vehicle_msgs::PolygonObstacle> static_people_;
int people_static_id_ = 0;

std::vector<double> easting;
std::vector<double> northing;
std::vector<double> height;
std::vector<double> heading;
std::vector<Eigen::Vector3d> initial_waypoints_;
Eigen::Vector3d pos_;

enum meta_state {IDLE, ONGOING, FINISH};
meta_state meta_state_ = IDLE;
enum tracking_state {tIDLE, tONGOING, tFINISH};
tracking_state tstate_ = tIDLE;
int tracking_leg_ = 0;
ros::Time last_people_angle_time_;
ros::Time last_reset_time_;
std::vector<Eigen::Vector2d> angle_list_;
std::vector<Eigen::Vector2d> lines_list_;

int main(int argc, char *argv[]) {
	// initialize ROS
	ros::init(argc, argv, "odom_utility");
	ros::NodeHandle nh("~");
  if (!nh.getParam("vehicle_info_path", vehicle_set_path_) ||
      !nh.getParam("deltatime",deltatime)) {
    	ROS_ERROR("Failed to get param %s", vehicle_set_path_.c_str());
    	assert(false);
  	}
    

    if(!nh.getParam("easting", easting) ||
    !nh.getParam("northing", northing) ||
    !nh.getParam("height", height) ||
    !nh.getParam("heading", heading))
    {
      ROS_WARN("Error loading parameters!");
      exit(-1);
    }

    for (size_t i = 0; i < easting.size(); i++) {

      Eigen::Vector3d pos;
      pos(0) = easting[i];
      pos(1) = northing[i];
      pos(2) = 0.0;
      initial_waypoints_.push_back(pos);
    }

  	ParseVehicleSet(&vehicle_set_);
    pListener = new (tf::TransformListener);
    pos_ << 0.0, 0.0, 0.0;
    last_people_angle_time_ = ros::Time::now();
    last_reset_time_ = ros::Time::now();

  	SetpointpubCBTimer_ = nh.createTimer(ros::Duration(0.1), SetpointpubCB);  
  	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odom_cb);
    ros::Subscriber people_sub = nh.subscribe<people_msgs::People>("/people", 10, people_cb);
    ros::Subscriber lines_sub = nh.subscribe<visualization_msgs::Marker>("/line_markers", 10, lines_cb);
    ros::Subscriber pubgoal_sub = nh.subscribe<std_msgs::Bool>("/pub_goal", 10, pubgoal_cb);
    ros::Subscriber globalplan_sub = nh.subscribe<nav_msgs::Path>("/path_planning_node/GlobalPlanner/plan", 10, globalplan_cb);
    ros::Subscriber people_angle_sub_= nh.subscribe("/angle_list", 1, peopleAngleCallback);
    ros::Subscriber reset_sub_= nh.subscribe("/reset_planner", 100, resetCallback);

  	arena_info_dynamic_pub_ =
      nh.advertise<vehicle_msgs::ArenaInfoDynamic>("/arena_info_dynamic", 10);
    arena_info_static_pub_ =
      nh.advertise<vehicle_msgs::ArenaInfoStatic>("/arena_info_static", 10);
  	surround_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis/parking_surround_trajs", 10);
    static_obst_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis/static_obst", 10);
    vis_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_odom_util", 0 );
    goal_pub_= nh.advertise<geometry_msgs::PoseStamped>("/shortterm_goal", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/3.0), TimerCallback);
    sgoal_pub_= nh.advertise<geometry_msgs::PoseStamped>("/shortterm_goal1", 1);
    peopledens_pub_= nh.advertise<std_msgs::Float32>("/people_density", 1);
    people_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/pp_markers", 1);
    line_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/lineangle_markers", 1);

  	ros::spin();
	ros::waitForShutdown();
 
		// ros::spinOnce();
	// }
	return 0;
}

void resetCallback(const std_msgs::Int32::ConstPtr& msg)
{
  last_reset_time_ = ros::Time::now();
}

void peopleAngleCallback(const std_msgs::Float32MultiArray::ConstPtr& angle_msg)
{
  angle_list_.clear();
  visualization_msgs::Marker marker_msg;
  marker_msg.ns = "people_lines";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.05;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  marker_msg.header.frame_id = "base_link";
  marker_msg.header.stamp = ros::Time::now();

  geometry_msgs::Point p_start;
  p_start.x = 0.0;
  p_start.y = 0.0;
  p_start.z = 0.0;

  for (int i=0; i<angle_msg->data.size()/2; i++)
  {
    Eigen::Vector2d peopleangle(-angle_msg->data[2*i], -angle_msg->data[2*i+1]);
    if (angle_msg->data[2*i] < angle_msg->data[2*i+1])
      peopleangle << -angle_msg->data[2*i+1], -angle_msg->data[2*i];

    angle_list_.push_back(peopleangle);

    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = 10.0*cos(peopleangle(0)/180.0*M_PI);
    p_end.y = 10.0*sin(peopleangle(0)/180.0*M_PI);
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
    marker_msg.points.push_back(p_start);
    p_end.x = 10.0*cos(peopleangle(1)/180.0*M_PI);
    p_end.y = 10.0*sin(peopleangle(1)/180.0*M_PI);
    marker_msg.points.push_back(p_end);
  }

  // people_marker_publisher_.publish(marker_msg);
  last_people_angle_time_ = ros::Time::now();
}

void globalplan_cb(const nav_msgs::Path::ConstPtr& msg)
{
  tstate_ = tONGOING;
  tracking_leg_ = 0;
  path_msg_ = *msg;
  Eigen::Vector3d startpos(path_msg_.poses[tracking_leg_].pose.position.x,
                           path_msg_.poses[tracking_leg_].pose.position.y,
                           0.0);  
  int leg_no_ = tracking_leg_;
  for (int i = tracking_leg_+1; i < path_msg_.poses.size(); ++i)
  {
    Eigen::Vector3d endpos(path_msg_.poses[i].pose.position.x,
                           path_msg_.poses[i].pose.position.y,
                           0.0);  
    if ((startpos - endpos).norm() > shortterm_dist_)
    {
      break;
    }

    leg_no_ = i;
  }
  tracking_leg_ = leg_no_;
  sgoal_pub_.publish(path_msg_.poses[tracking_leg_]);

}

void TimerCallback(const ros::TimerEvent&)
{
  if (meta_state_ == ONGOING)
  {
    if ((initial_waypoints_[current_leg_]-pos_).norm()<goal_radius_)
    {
      current_leg_++;
      if (current_leg_ == initial_waypoints_.size())
      {
        meta_state_ = FINISH;
        return;
      }
      geometry_msgs::PoseStamped posemsg;
      posemsg.header.stamp = ros::Time::now();
      posemsg.header.frame_id = "map";
      posemsg.pose.position.x = initial_waypoints_[current_leg_](0);
      posemsg.pose.position.y = initial_waypoints_[current_leg_](1);
      posemsg.pose.position.z = 0.0;
      tf2::Quaternion quat_tf;
      quat_tf.setRPY (0.0, 0.0, heading[current_leg_]/180.0*3.1415927);
      posemsg.pose.orientation.x = quat_tf.x();
      posemsg.pose.orientation.y = quat_tf.y();
      posemsg.pose.orientation.z = quat_tf.z();
      posemsg.pose.orientation.w = quat_tf.w();  
      goal_pub_.publish(posemsg);
    }
  }

  if (tstate_ == tONGOING)
  {
    Eigen::Vector3d startpos(path_msg_.poses[tracking_leg_].pose.position.x,
                             path_msg_.poses[tracking_leg_].pose.position.y,
                             0.0);      
    if ((startpos - pos_).norm()<goal_radius_)
    {
      if (tracking_leg_ == path_msg_.poses.size()-1)
      {
        tstate_ = tFINISH;
        return;
      }
      int leg_no_ = tracking_leg_;
      for (int i = tracking_leg_+1; i < path_msg_.poses.size(); ++i)
      {
        Eigen::Vector3d endpos(path_msg_.poses[i].pose.position.x,
                               path_msg_.poses[i].pose.position.y,
                               0.0);  
        if ((startpos - endpos).norm() > shortterm_dist_)
        {
          break;
        }

        leg_no_ = i;
      }
      tracking_leg_ = leg_no_;
      sgoal_pub_.publish(path_msg_.poses[tracking_leg_]);      
    }
  }
}

void pubgoal_cb(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
  {
    current_leg_ = 0;
    meta_state_ = ONGOING;
    geometry_msgs::PoseStamped posemsg;
    posemsg.header.stamp = ros::Time::now();
    posemsg.header.frame_id = "map";
    posemsg.pose.position.x = initial_waypoints_[0](0);
    posemsg.pose.position.y = initial_waypoints_[0](1);
    posemsg.pose.position.z = 0.0;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY (0.0, 0.0, heading[0]/180.0*3.1415927);
    posemsg.pose.orientation.x = quat_tf.x();
    posemsg.pose.orientation.y = quat_tf.y();
    posemsg.pose.orientation.z = quat_tf.z();
    posemsg.pose.orientation.w = quat_tf.w();        
    goal_pub_.publish(posemsg);

  }
}

double calculateOverlapPercentage(double A, double B, double C, double D) 
{
  double intersection = std::min(B, D) - std::max(A, C);
  double smaller_range = std::min(B-A, D-C);
  
  if (intersection <= 0) {
      return 0.0;
  }
  
  return (double)intersection / smaller_range * 100;
}

void lines_cb(const visualization_msgs::Marker::ConstPtr& msg)
{
  ros::Time time_now = ros::Time::now();
  if (!initialized_)
  {
    initialized_ = true;
    line_msg_ = msg;
  }
  else if ((time_now-line_msg_->header.stamp).toSec()<0.05 ||
            (time_now-last_reset_time_).toSec()<1.0) return;

    visualization_msgs::Marker marker_msg;
    marker_msg.ns = "angles_lines";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    marker_msg.scale.x = 0.05;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.header.frame_id = "base_link";
    marker_msg.header.stamp = ros::Time::now();

    geometry_msgs::Point p_start;
    p_start.x = 0.0;
    p_start.y = 0.0;
    p_start.z = 0.0;
    
  line_msg_ = msg;

  int num_segs = line_msg_->points.size()/2;
  vehicle_msgs::ArenaInfoStatic static_msg;
  static_msg.header.stamp = time_now;
  static_msg.header.frame_id = target_frame_;

  visualization_msgs::MarkerArray people_visualize;
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  people_visualize.markers.push_back(m);

  int ii = 1;
  for (auto people_obst: static_people_)
  {
    // static_msg.obstacle_set.obs_polygon.push_back(people_obst);

    visualization_msgs::Marker traj;
    traj.action = visualization_msgs::Marker::ADD;
    traj.id = ii++;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.pose.orientation.w = 1.00;
    traj.color.r = 0.00;
    traj.color.g = 0.00;
    traj.color.b = 0.00;
    traj.color.a = 1.00;
    traj.scale.x = 0.1;
    traj.scale.y = 0.1;
    traj.scale.z = 0.1;
    traj.header.frame_id = "map";
    traj.header.stamp = people_obst.header.stamp;
    geometry_msgs::Point point1;
    for(auto pt: people_obst.polygon.points){
      point1.x = pt.x;
      point1.y = pt.y;
      point1.z = pt.z;

      traj.points.push_back(point1);
    }
    point1.x = people_obst.polygon.points[0].x;
    point1.y = people_obst.polygon.points[0].y;
    point1.z = people_obst.polygon.points[0].z;
    traj.points.push_back(point1);
    people_visualize.markers.push_back(traj);
  }
  static_obst_pub.publish(people_visualize);

  static_people_.clear();

  lines_list_.clear();

  for (int i = 0; i < num_segs; ++i)
  {
    tf::Vector3 startpt(line_msg_->points[2*i].x, line_msg_->points[2*i].y, 0.0);
    tf::Vector3 endpt(line_msg_->points[2*i+1].x, line_msg_->points[2*i+1].y, 0.0);
    tf::Stamped<tf::Point> pt_local(startpt, line_msg_->header.stamp, line_msg_->header.frame_id);
    tf::Stamped<tf::Point> pt_target(startpt, line_msg_->header.stamp, line_msg_->header.frame_id);
    tf::Stamped<tf::Point> endpt_local(endpt, line_msg_->header.stamp, line_msg_->header.frame_id);
    tf::Stamped<tf::Point> endpt_target(endpt, line_msg_->header.stamp, line_msg_->header.frame_id);

    double angle1 = atan(line_msg_->points[2*i].y/line_msg_->points[2*i].x)/M_PI*180.0;
    double angle2 = atan(line_msg_->points[2*i+1].y/line_msg_->points[2*i+1].x)/M_PI*180.0;

    if (angle1 > angle2)
    {
      double ang_tmp = angle1;
      angle1 = angle2;
      angle2 = ang_tmp;
    }

    if (abs(angle1)<45.0 && abs(angle2)<45.0)
    {
      marker_msg.points.push_back(p_start);
      geometry_msgs::Point p_end;
      p_end.x = 10.0*cos(angle1/180.0*M_PI);
      p_end.y = 10.0*sin(angle1/180.0*M_PI);
      p_end.z = 0;
      marker_msg.points.push_back(p_end);
      marker_msg.points.push_back(p_start);
      p_end.x = 10.0*cos(angle2/180.0*M_PI);
      p_end.y = 10.0*sin(angle2/180.0*M_PI);
      marker_msg.points.push_back(p_end);      
    }
    if ((ros::Time::now()-last_people_angle_time_).toSec()<0.3) //people check using image detection
    {


      bool is_people = false;
      if (abs(angle1)<45.0 && abs(angle2)<45.0)
      {
        for (auto pp : angle_list_)
        {
          // ROS_INFO("[ODOM_UTILITY] angle1: %f, %f, %f, %f", angle1, angle2, pp(0), pp(1));
          if (calculateOverlapPercentage(angle1, angle2, pp(0), pp(1))>75.0)
          {
            // ROS_INFO("[ODOM_UTILITY] removing static obstacles!!!!!!!!!");
            is_people = true;
            break;
          }
        }
        if (is_people) continue;
      }
    }

    try{
      pListener->transformPoint(target_frame_, ros::Time::now()-ros::Duration(0.10),
                          pt_local, line_msg_->header.frame_id, pt_target);
      pListener->transformPoint(target_frame_, ros::Time::now()-ros::Duration(0.10),
                          endpt_local, line_msg_->header.frame_id, endpt_target);
    }
    catch (tf::TransformException ex){
        // ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        return;
     }

    Eigen::Vector2d A(pt_target.getX(), pt_target.getY());
    Eigen::Vector2d B(endpt_target.getX(), endpt_target.getY());
    
    lines_list_.push_back(A);
    lines_list_.push_back(B);

    if (closestDistance(A, B, pos_.head(2)) > 7.0) continue; // if the line is too far, dun publish it
    // but the line is still used to filter human detections

    vehicle_msgs::PolygonObstacle polygon_msg;
    polygon_msg.header.stamp = time_now;
    polygon_msg.header.frame_id = target_frame_;
    polygon_msg.id = i;
    geometry_msgs::Point32 p;
    p.x = pt_target.getX();
    p.y = pt_target.getY();
    p.z = pt_target.getZ();

    geometry_msgs::Point32 p1;
    p1.x = endpt_target.getX();
    p1.y = endpt_target.getY();
    p1.z = endpt_target.getZ();

    polygon_msg.polygon.points.push_back(p);
    polygon_msg.polygon.points.push_back(p1);
    polygon_msg.polygon.points.push_back(p);
    static_msg.obstacle_set.obs_polygon.push_back(polygon_msg);

  }

  static_msg.obstacle_set.header.stamp = time_now;
  static_msg.obstacle_set.header.frame_id = target_frame_;
  arena_info_static_pub_.publish(static_msg);
  // line_marker_publisher_.publish(marker_msg);

}

void people_cb(const people_msgs::People::ConstPtr& msg)
{
  // remove old static people
  while (static_people_.size()>0 && (ros::Time::now() -static_people_[0].header.stamp).toSec()>1.0)
  {
    static_people_.erase(static_people_.begin());
  }
  
  // visualization_msgs::Marker m;
  // m.type = visualization_msgs::Marker::ARROW;
  // m.action = visualization_msgs::Marker::DELETEALL;
  // m.id = 0;
  // m.scale.x = 0.02;
  // m.scale.y = 0.04;
  // m.scale.z = 1;
  // vis_pub_.publish(m);

  // tf::Matrix3x3 skew_sym(0, -angularZ_, 0, angularZ_, 0, 0, 0, 0, 0);
  // // ros::Time time_now = ros::Time::now()-ros::Duration(0.1);
  // tf::StampedTransform transform;
  // try{
  //     // listener_.waitForTransform("/base_link", "/map", msg->header.stamp, ros::Duration(3.0));

  //     pListener->lookupTransform(target_frame_,people_frame_,  ros::Time(0), transform);
  //         // std::cout << "transform exist\n";
  // }
  // catch (tf::TransformException ex){
  //         ROS_ERROR("%s",ex.what());
  //         ros::Duration(1.0).sleep();
  //         return;
  //  }

  // // listener_.lookupTransform(target_frame_,people_frame_,  msg->header.stamp, transform);
  // geometry_msgs::Twist interframe_twist;
  // // listener_.lookupTwist(target_frame_, people_frame_, time_now, ros::Duration(0.1), interframe_twist);
  // pListener->lookupTwist(people_frame_, target_frame_, people_frame_, tf::Point(0,0,0), 
  //                       people_frame_, msg->header.stamp, ros::Duration(0.2), interframe_twist);

  // tf::Vector3 omega_b_a, v_b_a;
  // tf::vector3MsgToTF(interframe_twist.angular, omega_b_a);
  // tf::vector3MsgToTF(interframe_twist.linear, v_b_a);
  visualization_msgs::MarkerArray surtrajs;
  int people_count = 0;
  for (int i = 0; i < msg->people.size(); ++i)
  {
    tf::Vector3 p_in_a(msg->people[i].position.x, msg->people[i].position.y, msg->people[i].position.z);
    tf::Vector3 v_in_a(msg->people[i].velocity.x, msg->people[i].velocity.y, msg->people[i].velocity.z);
    Eigen::Vector3d peoplepos(msg->people[i].position.x, msg->people[i].position.y, 0.0);
    Eigen::Vector3d peoplevel(msg->people[i].velocity.x, msg->people[i].velocity.y, 0.0);


    
    if ((peoplepos-pos_).norm()<5.0)
    {
      if (peoplevel.norm()<0.1)
      {
        vehicle_msgs::PolygonObstacle polygon_msg;
        polygon_msg.header.stamp = ros::Time::now();
        polygon_msg.header.frame_id = target_frame_;
        polygon_msg.id = people_static_id_;
        Eigen::Matrix<double,2,4> dimension;
        dimension<< 0.1, 0.1, -0.1, -0.1,
                    0.1, -0.1, -0.1, 0.1;

        for (size_t j = 0; j < 5; j++)
        {
          geometry_msgs::Point32 p;
          p.x = peoplepos(0) + dimension(0,j%4);
          p.y = peoplepos(1) + dimension(1,j%4);
          p.z = 0.0;
          polygon_msg.polygon.points.push_back(p);
        }
        static_people_.push_back(polygon_msg);
      }
      people_static_id_++;
      if (people_static_id_>1000)
      {
        people_static_id_ = 0;
      }
      

      //filter those people too close to obstacles
      bool not_good = false;
      for (int j=0; j<lines_list_.size()/2; j++)
      {
        if (closestDistance(lines_list_[2*j], lines_list_[2*j+1], peoplepos.head(2)) < 0.2) 
        {
          not_good = true;
          break;
        }
      }
      if (not_good) continue;
    }
    else continue; //only publish those people close to the robot
    
    people_count++;
    // // tf::Vector3 v_in_world = -omega_b_a.cross(p_in_a) + transform.getBasis()*v_in_a - v_b_a;
    // tf::Vector3 v_in_world = transform.getBasis()*v_in_a -transform.getBasis()*skew_sym*p_in_a ;
    // tf::Vector3 p_in_world = transform.getBasis()*p_in_a + transform.getOrigin();

    visualization_msgs::Marker traj;
    traj.action = visualization_msgs::Marker::ADD;
    traj.id = i;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.pose.orientation.w = 1.00;
    traj.color.r = 0.00;
    traj.color.g = 0.00;
    traj.color.b = 0.00;
    traj.color.a = 1.00;
    traj.scale.x = 0.1;
    traj.scale.y = 0.1;
    traj.scale.z = 0.1;
    traj.header.frame_id = "map";
    traj.header.stamp =ros::Time::now();// + ros::Duration(1.0);
    geometry_msgs::Point point1;
    for(double t = 0.0; t<=pre_time; t += deltatime){
      point1.x = p_in_a.getX() + v_in_a.getX()*t;
      point1.y = p_in_a.getY() + v_in_a.getY()*t;
      point1.z = p_in_a.getZ() + v_in_a.getZ()*t;
      traj.points.push_back(point1);
    }
  surtrajs.markers.push_back(traj);
    // visualization_msgs::Marker m;
    // m.type = visualization_msgs::Marker::ARROW;
    // m.action = visualization_msgs::Marker::ADD;
    // m.id = i;  // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
    // m.ns = "people";
    // m.color.a = 1.0; // Don't forget to set the alpha!
    // m.color.r = 0.0;
    // m.color.g = 1.0;
    // m.color.b = 0.0;

    // m.scale.x = 0.15;  
    // m.scale.y = 0.05;
    // m.scale.z = 0.05;
    // m.header.stamp = ros::Time::now();
    // m.header.frame_id = target_frame_;

    // geometry_msgs::Point p;
    // p.x = p_in_world.getX();
    // p.y = p_in_world.getY();
    // p.z = p_in_world.getZ();
    // m.points.push_back(p);
    // tf::Vector3 endpt = p_in_world + 0.5*v_in_world;
    // p.x = endpt.getX();
    // p.y = endpt.getY();
    // p.z = endpt.getZ();
    // m.points.push_back(p);
    // vis_pub_.publish(m);
  }

  surround_traj_pub.publish(surtrajs);
  double people_dens = (double)people_count / (M_PI * 3.0 * 3.0);
  std_msgs::Float32 msgg;
  msgg.data = people_dens;
  peopledens_pub_.publish(msgg);
}

void SetpointpubCB(const ros::TimerEvent& e)
{

  //publish sur trajs;
  visualization_msgs::MarkerArray surtrajs;
  // for(int i = 0; i < vehicle_num; i++){
  //   visualization_msgs::Marker traj;
  //   traj.action = visualization_msgs::Marker::ADD;
  //   traj.id = i;
  //   traj.type = visualization_msgs::Marker::LINE_STRIP;
  //   traj.pose.orientation.w = 1.00;
  //   traj.color.r = 0.00;
  //   traj.color.g = 0.00;
  //   traj.color.b = 0.00;
  //   traj.color.a = 1.00;
  //   traj.scale.x = 0.1;
  //   traj.scale.y = 0.1;
  //   traj.scale.z = 0.1;
  //   traj.header.frame_id = "map";
  //   traj.header.stamp =ros::Time().fromSec(cur_time);
  //   geometry_msgs::Point point1;
  //   for(double t = 0.0; t<=pre_time; t += deltatime){
  //     common::State state = getState(cur_time + t, inittime, i);
  //     point1.x = state.vec_position[0];
  //     point1.y = state.vec_position[1];
  //     point1.z = state.angle;
  //     traj.points.push_back(point1);
  //   }
  // surtrajs.markers.push_back(traj);

  // }

  surround_traj_pub.publish(surtrajs);

}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	// tf2::Quaternion quat_tf;
	// tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
	// double roll,pitch,yaw;
	// double yaw = tf::getYaw(msg->pose.pose.orientation);
  // angularZ_ = msg->twist.twist.angular.z;

	std::string frame_id="map";
	ros::Time timestamp = ros::Time::now();
	vehicle_msgs::ArenaInfoDynamic msg1;
    msg1.header.frame_id = frame_id;
    msg1.header.stamp = timestamp;
    // msg.vehicle_set.header.frame_id = frame_id;
    // msg.vehicle_set.header.stamp = timestamp;

    // vehicle_msgs::Vehicle vehicle_msg;

    // vehicle_msg.header.frame_id = frame_id;
    // vehicle_msg.header.stamp = timestamp;
    // vehicle_msg.id.data = 0;
    // vehicle_msg.subclass.data = "car";
    // vehicle_msg.type.data = "VehicleWithBicycleKinematics";
    // vehicle_msgs::VehicleParam param;
    // msg.vehicle_set.vehicles.push_back(vehicle_msg);

    // common::State state1 = vehicle_set_.vehicles[0].state();
    // // state1.time_stamp = timestamp;
    // state1.vec_position(0) = msg->pose.pose.position.x;
    // state1.vec_position(1) = msg->pose.pose.position.y;

    // Eigen::Quaternionf orientation_W_B(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
    //                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // Eigen::Vector3f velocity_world(msg->twist.twist.linear.x, msg->twist.twist.linear.y, 0.0f);
    // Eigen::Vector3f velocity_body = orientation_W_B.inverse() *velocity_world;    

    // std::cout<<"velocity body x: "<<msg->twist.twist.linear.x<<" velocity body y: "<<msg->twist.twist.linear.y<<std::endl;
    // double angvel = msg->twist.twist.angular.z;
    // state1.steer = 0.0;//atan(0.3/(velocity_body(0)/angvel));
    // state1.angle = yaw;
    // state1.velocity = msg->twist.twist.linear.x;
    // vehicle_set_.vehicles[0].set_state(state1);

  // Vecf<2> vec_position{Vecf<2>::Zero()};
  // decimal_t angle{0.0};  // heading angle
  // decimal_t curvature{0.0};
  // decimal_t velocity{0.0};
  // decimal_t acceleration{0.0};
  // decimal_t steer{0.0};  // steering angle

    tf::StampedTransform transform;
    try{
        // listener_.waitForTransform("/base_link", "/map", msg->header.stamp, ros::Duration(3.0));

        pListener->lookupTransform("/map","/base_link", ros::Time::now()-ros::Duration(0.1), transform);
            // std::cout << "transform exist\n";
    }
    catch (tf::TransformException ex){
            // ROS_ERROR("ODOM_CB: %s",ex.what());
            ros::Duration(0.1).sleep();
            return;
     }    

    // std::cout<<"POS body x: "<<transform.getOrigin().getX()<<"POS body y: "<<transform.getOrigin().getY()<<std::endl;
    // std::cout<<"YAW body: "<<tf::getYaw(transform.getRotation())<<std::endl;

    common::State state1 = vehicle_set_.vehicles[0].state();
    // state1.time_stamp = timestamp;
    pos_(0) = transform.getOrigin().getX();
    pos_(1) = transform.getOrigin().getY();

    state1.vec_position(0) = transform.getOrigin().getX();
    state1.vec_position(1) = transform.getOrigin().getY();

    // std::cout<<"velocity body x: "<<msg->twist.twist.linear.x<<" velocity body y: "<<msg->twist.twist.linear.y<<std::endl;
    double angvel = msg->twist.twist.angular.z;
    state1.steer = 0.0;//atan(0.3/(velocity_body(0)/angvel));
    state1.angle = tf::getYaw(transform.getRotation());
    Eigen::Vector2d velll(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    state1.velocity = velll.norm();
    vehicle_set_.vehicles[0].set_state(state1);

    vehicle_msgs::Encoder::GetRosVehicleSetFromVehicleSet(vehicle_set_, timestamp, frame_id,
                                   &msg1.vehicle_set);

    arena_info_dynamic_pub_.publish(msg1);

  // // transforming velocity
  // if (sim_type_!="rotors" && sim_type_!="vicon_dji_mini"){ //for real flight or using DJI simulator, velocity is already in global ENU frame
  //   geoVec3toEigenVec3(msg->twist.twist.linear, current_.vel);
  // } else { //for simulation with ROTORS, convert body frame velocity to global frame
  //   Eigen::Quaternionf orientation_W_B(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
  //                                      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  //   Eigen::Vector3f velocity_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  //   Eigen::Vector3f velocity_world = orientation_W_B *velocity_body;    
  //   current_.vel = velocity_world;
  // }
}

bool ParseVehicleSet(common::VehicleSet *p_vehicle_set) {
  printf("\n[ArenaLoader] Loading vehicle set\n");

  std::fstream fs(vehicle_set_path_);
  Json root;
  fs >> root;

  Json vehicles_json = root["vehicles"];
  Json info_json = vehicles_json["info"];
  // ~ allow loading part of the vehicles for debugging purpose
  for (int i = 0; i < static_cast<int>(info_json.size()); ++i) {
    common::Vehicle vehicle;
    vehicle.set_id(info_json[i]["id"].get<int>());
    vehicle.set_subclass(info_json[i]["subclass"].get<std::string>());
    vehicle.set_type(info_json[i]["type"].get<std::string>());

    Json state_json = info_json[i]["init_state"];
    common::State state;
    state.vec_position(0) = state_json["x"].get<double>();
    state.vec_position(1) = state_json["y"].get<double>();
    state.angle = state_json["angle"].get<double>();
    state.curvature = state_json["curvature"].get<double>();
    state.velocity = state_json["velocity"].get<double>();
    state.acceleration = state_json["acceleration"].get<double>();
    state.steer = state_json["steer"].get<double>();
    vehicle.set_state(state);

    Json params_json = info_json[i]["params"];
    common::VehicleParam param;
    param.set_width(params_json["width"].get<double>());
    param.set_length(params_json["length"].get<double>());
    param.set_wheel_base(params_json["wheel_base"].get<double>());
    param.set_front_suspension(params_json["front_suspension"].get<double>());
    param.set_rear_suspension(params_json["rear_suspension"].get<double>());
    auto max_steering_angle = params_json["max_steering_angle"].get<double>();
    param.set_max_steering_angle(max_steering_angle * kPi / 180.0);
    param.set_max_longitudinal_acc(
        params_json["max_longitudinal_acc"].get<double>());
    param.set_max_lateral_acc(params_json["max_lateral_acc"].get<double>());

    param.set_d_cr(param.length() / 2 - param.wheel_base() / 2 );
    vehicle.set_param(param);

    p_vehicle_set->vehicles.insert(
        std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
  }
  // p_vehicle_set->print();

  fs.close();
  return true;
}

double closestDistance(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C) 
{
    Eigen::Vector2d AB = B - A;
    Eigen::Vector2d AC = C - A;

    double t = AC.dot(AB) / AB.dot(AB);
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    Eigen::Vector2d closestPoint = {A(0) + t * AB(0), A(1) + t * AB(1)};
    Eigen::Vector2d CP = C - closestPoint;

    return CP.norm();
}

// 00112 void TransformListener::transformTwist(const std::string& target_frame,
// 00113     const geometry_msgs::TwistStamped& msg_in,
// 00114     geometry_msgs::TwistStamped& msg_out) const
// 00115 {
// 00116   tf::Vector3 twist_rot(msg_in.twist.angular.x,
// 00117                         msg_in.twist.angular.y,
// 00118                         msg_in.twist.angular.z);
// 00119   tf::Vector3 twist_vel(msg_in.twist.linear.x,
// 00120                         msg_in.twist.linear.y,
// 00121                         msg_in.twist.linear.z);
// 00122 
// 00123   tf::StampedTransform transform;
// 00124   lookupTransform(target_frame,msg_in.header.frame_id,  msg_in.header.stamp, transform);
// 00125 
// 00126 
// 00127   tf::Vector3 out_rot = transform.getBasis() * twist_rot;
// 00128   tf::Vector3 out_vel = transform.getBasis()* twist_vel + transform.getOrigin().cross(out_rot);
// 00129 
// 00130   geometry_msgs::TwistStamped interframe_twist;
// 00131   lookupVelocity(target_frame, msg_in.header.frame_id, msg_in.header.stamp, ros::Duration(0.1), interframe_twist); //\todo get rid of hard coded number
// 00132 
// 00133   msg_out.header.stamp = msg_in.header.stamp;
// 00134   msg_out.header.frame_id = target_frame;
// 00135   msg_out.twist.linear.x =  out_vel.x() + interframe_twist.twist.linear.x;
// 00136   msg_out.twist.linear.y =  out_vel.y() + interframe_twist.twist.linear.y;
// 00137   msg_out.twist.linear.z =  out_vel.z() + interframe_twist.twist.linear.z;
// 00138   msg_out.twist.angular.x =  out_rot.x() + interframe_twist.twist.angular.x;
// 00139   msg_out.twist.angular.y =  out_rot.y() + interframe_twist.twist.angular.y;
// 00140   msg_out.twist.angular.z =  out_rot.z() + interframe_twist.twist.angular.z;
// 00141 
// 00142   }*/

 // void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
 //                  const tf::Point & reference_point, const std::string& reference_point_frame, 
 //                  const ros::Time& time, const ros::Duration& averaging_interval, 
 //                  geometry_msgs::Twist& twist) const
 // {
   
 //   ros::Time latest_time, target_time;
 //   getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); 
 
 //   if (ros::Time() == time)
 //     target_time = latest_time;
 //   else
 //     target_time = time;
 
 //   ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);
   
 //   ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
 //   ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
 //   StampedTransform start, end;
 //   lookupTransform(observation_frame, tracking_frame, start_time, start);
 //   lookupTransform(observation_frame, tracking_frame, end_time, end);
 
 
 //   tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
 //   tf::Quaternion quat_temp;
 //   temp.getRotation(quat_temp);
 //   tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
 //   tfScalar ang = quat_temp.getAngle();
   
 //   double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
 //   double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
 //   double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();
 
 
 //   tf::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(), 
 //                        (delta_y)/corrected_averaging_interval.toSec(),
 //                        (delta_z)/corrected_averaging_interval.toSec());
 //   tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());
 
 
 //   // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)
 
 
 //   //correct for the position of the reference frame
 //   tf::StampedTransform inverse;
 //   lookupTransform(reference_frame,tracking_frame,  target_time, inverse);
 //   tf::Vector3 out_rot = inverse.getBasis() * twist_rot;
 //   tf::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);
 
 
 //   //Rereference the twist about a new reference point
 //   // Start by computing the original reference point in the reference frame:
 //   tf::Stamped<tf::Point> rp_orig(tf::Point(0,0,0), target_time, tracking_frame);
 //   transformPoint(reference_frame, rp_orig, rp_orig);
 //   // convert the requrested reference point into the right frame
 //   tf::Stamped<tf::Point> rp_desired(reference_point, target_time, reference_point_frame);
 //   transformPoint(reference_frame, rp_desired, rp_desired);
 //   // compute the delta
 //   tf::Point delta = rp_desired - rp_orig;
 //   // Correct for the change in reference point 
 //   out_vel = out_vel + out_rot * delta;
 //   // out_rot unchanged   
 
 //   /*
 //     printf("KDL: Rotation %f %f %f, Translation:%f %f %f\n", 
 //          out_rot.x(),out_rot.y(),out_rot.z(),
 //          out_vel.x(),out_vel.y(),out_vel.z());
 //   */   
 
 //   twist.linear.x =  out_vel.x();
 //   twist.linear.y =  out_vel.y();
 //   twist.linear.z =  out_vel.z();
 //   twist.angular.x =  out_rot.x();
 //   twist.angular.y =  out_rot.y();
 //   twist.angular.z =  out_rot.z();
 
 // }