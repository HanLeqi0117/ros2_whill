#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

class WHILL_ODOM
{
public:
  WHILL_ODOM();

private:
  ros::NodeHandle nh_;
  void joyintstateCallback(const sensor_msgs::JointState::ConstPtr& joyst);
  ros::Publisher pub_odom_;
  ros::Subscriber joy_sub_;
  double confineRadian(double rad);
	tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_;

  ros::Time past_time;
  double pose_x_;
  double pose_y_;
  double pose_theta_;
  
  double wheel_radius_;
  double wheel_tread_;

  double max_dt_;
  bool publish_tf_;
};

WHILL_ODOM::WHILL_ODOM(){
  ros::NodeHandle ph_("~");
  ph_.param("wheel_radius", wheel_radius_, 0.1325);
  ph_.param("wheel_tread", wheel_tread_, 0.496);
  ph_.param("publish_tf", publish_tf_, false);
  ph_.param("max_dt", max_dt_, 0.2);
  pub_odom_ = ph_.advertise<nav_msgs::Odometry>("/odometory", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joyst", 10, &WHILL_ODOM::joyintstateCallback, this);
  pose_x_=pose_y_=pose_theta_=0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0.0;
  odom_.twist.twist.angular.x = 0.0;
  odom_.twist.twist.angular.y = 0.0;
  odom_.pose.pose.position.z = 0;
  odom_.child_frame_id="base_link";
  double max_error=0.1;
  double C=(max_error/3)*(max_error/3);
  double Dif=0.4;
  double twist_X_conv=C/2;
  double axis_z_conv=C/2/Dif/Dif;
  odom_.twist.covariance={twist_X_conv,0,0,0,0,0,
                           0,0,0,0,0,0,
                           0,0,0,0,0,0,
                           0,0,0,0,0,0,
                           0,0,0,0,0,0,
                           0,0,0,0,0,axis_z_conv};
  past_time =ros::Time::now();
}

void WHILL_ODOM::joyintstateCallback(const sensor_msgs::JointState::ConstPtr& joyst)
{
    ros::Duration delta_time = ros::Time::now() - past_time;
    past_time =ros::Time::now();
    double dt=delta_time.toSec();
    if(dt>max_dt_) return;
    double angle_vel_r = joyst->velocity[1];
    double angle_vel_l = -joyst->velocity[0];

    double vr = angle_vel_r * wheel_radius_;
    double vl = angle_vel_l * wheel_radius_;

    double delta_L = (vr + vl) / 2.0;
    double delta_theta = (vr - vl) / (wheel_tread_);

    pose_x_ += delta_L * dt * cos(pose_theta_ + delta_theta * dt);
    pose_y_ += delta_L * dt * sin(pose_theta_ + delta_theta * dt);

    odom_.header=joyst->header;
    odom_.header.frame_id="odom";
    pose_theta_ = pose_theta_ + delta_theta * dt;
    pose_theta_ = confineRadian(pose_theta_);
    //std::cout<<"yama_ver theta"<<pose_theta_<<"[rad]"<<"yama_ver delta_theta"<<delta_theta<<"[rad/s]"<<"radius"<<wheel_radius_<<"tread"<<wheel_tread_<<std::endl;
    //std::cout<<"original dt"<<dt<<std::endl;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_theta_);    
    odom_.pose.pose.position.x = pose_x_;
    odom_.pose.pose.position.y = pose_y_;
    odom_.pose.pose.orientation = odom_quat;

    //velocity
    odom_.twist.twist.linear.x = delta_L;

    odom_.twist.twist.angular.z = delta_theta;
    pub_odom_.publish(odom_);
    if (publish_tf_)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header=joyst->header;
        odom_trans.header.frame_id="odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = pose_x_;
        odom_trans.transform.translation.y = pose_y_;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;  
        odom_broadcaster.sendTransform(odom_trans);
    }
}
double WHILL_ODOM::confineRadian(double rad)
{
    if (rad >= M_PI)
    {
        rad -= 2.0 * M_PI;
    }
    if (rad <= -M_PI)
    {
        rad += 2.0 * M_PI;
    }
    return rad;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "WHILL_ODOM");
  WHILL_ODOM WHILL_ODOM;

  ros::spin();
}
