
//#include<iostream>

//#include <tf2/LinearMath/Quaternion.h>

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <unistd.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>

using namespace cv;
using namespace std;

#include "SubtUtil.h"

ros::Time landing_request;
ros::Time takeoff_request;

float ucmd, vcmd, wcmd, rcmd;

mavros_msgs::State             g_current_state;
geometry_msgs::TwistStamped    velcmd;
geometry_msgs::PoseStamped     Local, Pos;
geometry_msgs::TwistStamped    Localvel;
// nav_msgs::Odometry             Odom;

std_msgs::Float32MultiArray    body_vel;
std_msgs::Float32MultiArray    hover_pt;
std_msgs::UInt8                mission;
tf::Quaternion                 quat;

std_msgs::Float32MultiArray    tele_flag;
geometry_msgs::Twist           tele_cmd;
std_msgs::Float32MultiArray    GoalAction;

nav_msgs::Path path_cur;
visualization_msgs::MarkerArray TargetArray;

mavros_msgs::RCIn               rc_in;

float   cmd_x = 0.0;
float   cmd_y = 0.0;
float   cmd_z = 0.0;
float   cmd_r = 0.0;

float   cmd_RCx = 0.0;
float   cmd_RCy = 0.0;
float   cmd_RCz = 0.0;
float   cmd_RCr = 0.0;

float   Cur_Pos_m[3];
float   Cur_Vel_mps[3];
float   Cur_Att_rad[3];
float   Cur_Att[3];

float   angle_err = 0.0;

float   takeoff_x = 0.0;
float   takeoff_y = 0.0;
float   init_heading = 0.0;

float   hover[3];
float   hover_heading = 0.0;

int     flag_takeoff = 0;
int     flag_landing = 0;

uint8_t MotorAct = 0;
int     flag_armed = 0;
int     count_ros = 0;
int     goal_service = 0;

float   goal[4];
float   goal_velx;
float   goal_velz;

float   t_cur = 0.0;

int     flag_PosAvailable = 0;

void callback_cmd_flag(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    tele_flag = *msg;
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    tele_cmd = *msg;
}

void callback_goal(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    GoalAction = *msg;

    goal_service = GoalAction.data[0];

    goal[0] = GoalAction.data[1];
    goal[1] = GoalAction.data[2];
    goal[2] = GoalAction.data[3];
    goal[3] = GoalAction.data[4] *D2R; // deg 2 rad

    goal_velx = GoalAction.data[5];
    goal_velz = GoalAction.data[6];
}

void callback_rc_in(const mavros_msgs::RCIn::ConstPtr& msg_input)
{
    cmd_RCy =  (msg_input->channels[0] - PWM_ROL)/PWM_LEN*VELX_MAX;
    cmd_RCx = -(msg_input->channels[1] - PWM_PIT)/PWM_LEN*VELX_MAX;
    cmd_RCz =  (msg_input->channels[2] - PWM_THR)/PWM_LEN*VELZ_MAX;
    cmd_RCr =  (msg_input->channels[3] - PWM_YAW)/PWM_LEN*VELR_MAX;
}

void callback_state(const mavros_msgs::State::ConstPtr& msg)
{
    g_current_state = *msg;

    system("clear");
    std::cout << "\n[Team CoSTAR] state_cb(), -----------";
    std::cout << "\n          g_current_state.connected = " << ((g_current_state.connected) ? "OK!" : "Not yet!");
    std::cout << "\n          g_current_state.armed     = " << ((g_current_state.armed ) ? "OK!" : "Not yet!");
    std::cout << "\n          g_current_state.guided    = " << ((g_current_state.guided) ? "OK!" : "Not yet!");
    std::cout << "\n          g_current_state.mode      = " << g_current_state.mode;
    std::cout << "\n          Cur   X Y Z r             = " << Cur_Pos_m[0] << ", "<< Cur_Pos_m[1] << ", "<< Cur_Pos_m[2] << ", "<<  Cur_Att_rad[2]*R2D;
    std::cout << "\n          velocity auto             = " << cmd_x << ", "<< cmd_y << ", "<< cmd_z  << ", "<< cmd_r;
    std::cout << "\n          velocity output           = " << velcmd.twist.linear.x << ", "<<velcmd.twist.linear.y << ", "<<velcmd.twist.linear.z <<  ", "<<velcmd.twist.angular.z;
    std::cout << "\n          goal service              = " << goal_service;
    std::cout << "\n[Team CoSTAR] ------------------------\n";
}
/*
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg_input)
{
    Odom = *msg_input;

    Cur_Pos_m[0] = Odom.pose.pose.position.x;
    Cur_Pos_m[1] = Odom.pose.pose.position.y;
    Cur_Pos_m[2] = Odom.pose.pose.position.z;

    q[0] = Odom.pose.pose.orientation.x;
    q[1] = Odom.pose.pose.orientation.y;
    q[2] = Odom.pose.pose.orientation.z;
    q[3] = Odom.pose.pose.orientation.w;
    QuaterniontoEuler(Cur_Att_rad[0], Cur_Att_rad[1], Cur_Att_rad[2]);

    quat[0] = q[0];
    quat[1] = q[1];
    quat[2] = q[2];
    quat[3] = q[3];

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(Cur_Pos_m[0], Cur_Pos_m[1], Cur_Pos_m[2]));
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "uav"));

    flag_PosAvailable = 1;
}
*/
void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg_input)
{
    Pos = *msg_input;

    Cur_Pos_m[0] = Pos.pose.position.x;
    Cur_Pos_m[1] = Pos.pose.position.y;
    Cur_Pos_m[2] = Pos.pose.position.z;

    q[0] = Pos.pose.orientation.x;
    q[1] = Pos.pose.orientation.y;
    q[2] = Pos.pose.orientation.z;
    q[3] = Pos.pose.orientation.w;
    QuaterniontoEuler(Cur_Att_rad[0], Cur_Att_rad[1], Cur_Att_rad[2]);

    quat[0] = q[0];
    quat[1] = q[1];
    quat[2] = q[2];
    quat[3] = q[3];

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(Cur_Pos_m[0], Cur_Pos_m[1], Cur_Pos_m[2]));
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "uav"));

    flag_PosAvailable = 1;
}


void callback_local_vel(const geometry_msgs::TwistStamped::ConstPtr& msg_input)
{
    Localvel = *msg_input;

    Cur_Vel_mps[0] = Localvel.twist.linear.x;
    Cur_Vel_mps[1] = Localvel.twist.linear.y;
    Cur_Vel_mps[2] = Localvel.twist.linear.z;
}

// void Mission_Update(void);
// void Auto_Takeoff(void);
// void Auto_Landing(void);
// void WP_Flight(void);
// void Hovering(void);
// void Visualization(void);

// ros::Publisher pos_cur_pub, path_uav_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minitor_offboard_node");
    ROS_INFO("INIT monitor node");
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_pub;

    // Subscribe Topic
    ros::Subscriber state_sub     = nh_sub.subscribe ("/mavros/state" , 2,                         &callback_state);
    // ros::Subscriber local_pos_sub = nh_sub.subscribe ("/mavros/global_position/local" , 2,         &callback_odom);
    // ros::Subscriber local_pos_sub = nh_sub.subscribe ("/camera/odom/sample" , 2,                   &callback_odom);
    ros::Subscriber local_pos_sub = nh_sub.subscribe ("/mavros/local_position/pose" , 2,           &callback_pose);
    ros::Subscriber local_vel_sub = nh_sub.subscribe ("/mavros/local_position/velocity_local", 2,  &callback_local_vel);
    ros::Subscriber cmd_sub       = nh_sub.subscribe ("/mavros_comm_node/tele_key/cmd_vel", 2,     &callback_cmd_vel);
    ros::Subscriber flag_sub      = nh_sub.subscribe ("/mavros_comm_node/tele_key/flag", 2,        &callback_cmd_flag);
    ros::Subscriber goal_sub      = nh_sub.subscribe ("/GoalAction", 2,                            &callback_goal);
    ros::Subscriber rc_in_sub = nh_sub.subscribe ("/mavros/rc/in", 2,                              &callback_rc_in);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    // wait for FCU connection
    while(ros::ok() && g_current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    ros::spin();

    return 0;
}

// void Mission_Update(void)
// {

//     switch (goal_service)
//     {
//     // Basic Mission
//         case 1:
//             Auto_Takeoff();
//             break;

//         case 2:
//             Auto_Landing();
//             break;

//         case 3:
//             WP_Flight();
//             break;

//         case 5:
//             Hovering();
//             break;
        
//         default:
//             cmd_x = 0.0;
//             cmd_y = 0.0;
//             cmd_z = 0.0;
//             cmd_r = 0.0;

//             velcmd.twist.linear.x = 0.0;
//             velcmd.twist.linear.y = 0.0;
//             velcmd.twist.linear.z = 0.0;
//             velcmd.twist.angular.z = 0.0;

//             takeoff_x = Cur_Pos_m[0];
//             takeoff_y = Cur_Pos_m[1];
//             init_heading = Cur_Att_rad[2];
//             hover_heading = Cur_Att_rad[2];

//             break;
//     }
// }

// void Auto_Takeoff(void) // 1
// {
//     cmd_x = satmax(Kpx*(takeoff_x - Cur_Pos_m[0]), goal_velx);
//     cmd_y = satmax(Kpx*(takeoff_y - Cur_Pos_m[1]), goal_velx);
//     cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);

//     angle_err = GetNED_angle_err(init_heading, Cur_Att_rad[2]);
//     cmd_r = -satmax(Kr*angle_err, R_MAX);
//     mission.data = 1;  // [takeoff]

//     if (fabs(goal[2] - Cur_Pos_m[2]) < 0.1)
//     {
//          goal_service = 5;
//          hover[0] = Cur_Pos_m[0];
//          hover[1] = Cur_Pos_m[1];
//          hover[2] = goal[2];
//          hover_heading = Cur_Att_rad[2];
//     }
// }

// void Auto_Landing(void) // 2
// {
//     cmd_x = satmax(Kpx*(goal[0] - Cur_Pos_m[0]),goal_velx);
//     cmd_y = satmax(Kpx*(goal[1] - Cur_Pos_m[1]),goal_velx);
//     cmd_z = goal_velz;

//     angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
//     cmd_r = -satmax(Kr*angle_err, R_MAX);
//     mission.data = 2;  // [landing]

//     if (flag_landing != 1)
//     {
//         landing_request = ros::Time::now();
//     }

//     if (Cur_Pos_m[2] < 0.2)
//     {
//         cmd_z = -0.5;
//         flag_landing = 1;
//         if (ros::Time::now() - landing_request > ros::Duration(1.0))
//         {
//             flag_takeoff = 0;
//             takeoff_x = Cur_Pos_m[0];
//             takeoff_y = Cur_Pos_m[1];
//             flag_landing = 0;
//             goal_service = 0;
//             takeoff_request = ros::Time::now();
//         }
//     }
// }

// void WP_Flight(void) // 3
// {
//     cmd_x = satmax(Kpx*(goal[0] - Cur_Pos_m[0]),goal_velx);
//     cmd_y = satmax(Kpx*(goal[1] - Cur_Pos_m[1]),goal_velx);
//     cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]),goal_velz) + Kdz*(0.0 - Cur_Vel_mps[2]);

//     angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
//     cmd_r = -satmax(Kr*angle_err, R_MAX);
//     mission.data = 3;  // [fly_to]

//     hover[0] = Cur_Pos_m[0];
//     hover[1] = Cur_Pos_m[1];
//     hover[2] = goal[2];
//     hover_heading = Cur_Att_rad[2];
// }

// void Hovering(void) // 5
// {
//     cmd_x = satmax(Kpx*(hover[0] - Cur_Pos_m[0]),goal_velx);
//     cmd_y = satmax(Kpx*(hover[1] - Cur_Pos_m[1]),goal_velx);
//     cmd_z = satmax(Kpz*(hover[2] - Cur_Pos_m[2]),goal_velz);

//     angle_err = GetNED_angle_err(hover_heading, Cur_Att_rad[2]);
//     cmd_r = -satmax(Kr*angle_err, R_MAX);
//     mission.data = 5;  // [hovering]
// }

// void Visualization(void)
// {
//     if (flag_PosAvailable == 1)
//     {
//         geometry_msgs::PoseStamped cur_pose;
//         path_cur.header.stamp = ros::Time::now();
//         path_cur.header.frame_id = "map";
//         cur_pose.pose.position.x = Cur_Pos_m[0];
//         cur_pose.pose.position.y = Cur_Pos_m[1];
//         cur_pose.pose.position.z = Cur_Pos_m[2];
//         cur_pose.pose.orientation.x = q[0];
//         cur_pose.pose.orientation.y = q[1];
//         cur_pose.pose.orientation.z = q[2];
//         cur_pose.pose.orientation.w = q[3];
//         path_cur.poses.push_back(cur_pose);
//         path_uav_pub.publish(path_cur);

//         visualization_msgs::Marker Pose;
//         Pose.type = visualization_msgs::Marker::CUBE;
//         Pose.header.frame_id = "map";
//         Pose.scale.x = Pose.scale.y = Pose.scale.z = 0.5;
//         Pose.color.r = 0.2;
//         Pose.color.b = 1.0;
//         Pose.color.g = 0.2;
//         Pose.color.a = 1.0;

//         Pose.pose.position.x = Cur_Pos_m[0];
//         Pose.pose.position.y = Cur_Pos_m[1];
//         Pose.pose.position.z = Cur_Pos_m[2];
//         Pose.pose.orientation.x = q[0];
//         Pose.pose.orientation.y = q[1];
//         Pose.pose.orientation.z = q[2];
//         Pose.pose.orientation.w = q[3];
//         Pose.ns = '1';
//         pos_cur_pub.publish(Pose);
//     }
// }

