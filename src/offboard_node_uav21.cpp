
//#include<iostream>

//#include <tf2/LinearMath/Quaternion.h>

#include <stdio.h>
#include <ros/ros.h>
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

#include "SubtUtil_uav21.h"

//#define  LOG_ON

ros::Time landing_request;
ros::Time takeoff_request;
ros::Time hovering_request;

float ucmd, vcmd, wcmd, rcmd;
float ucmd_pre, vcmd_pre, wcmd_pre, rcmd_pre;
float ucmd_LPF, vcmd_LPF, wcmd_LPF, rcmd_LPF;

mavros_msgs::State             g_current_state;
geometry_msgs::TwistStamped    velcmd;
geometry_msgs::PoseStamped     Local;
geometry_msgs::TwistStamped    Localvel;
nav_msgs::Odometry             Subt_Odom, Odom, Tar_Odom;

std_msgs::Float32MultiArray    body_vel;
std_msgs::Float32MultiArray    hover_pt;
std_msgs::UInt8                mission;
std_msgs::UInt8                MotorAct_msg;
std_msgs::UInt8                FlightMode_msg;
tf::Quaternion                 quat;

std_msgs::Float32MultiArray    tele_flag;
geometry_msgs::Twist           tele_cmd;
std_msgs::Float32MultiArray    GoalAction;
std_msgs::Float32MultiArray    Detection;

nav_msgs::Path path_cur;
visualization_msgs::MarkerArray TargetArray;

mavros_msgs::RCIn               rc_in;

float   cmd_x = 0.0;
float   cmd_y = 0.0;
float   cmd_z = 0.0;
float   cmd_r = 0.0;

float   cmd_x_raw = 0.0;
float   cmd_y_raw = 0.0;
float   cmd_z_raw = 0.0;

float   cmd_x_pre = 0.0;
float   cmd_y_pre = 0.0;
float   cmd_z_pre = 0.0;

float   cmd_RCx = 0.0;
float   cmd_RCy = 0.0;
float   cmd_RCz = 0.0;
float   cmd_RCr = 0.0;

float   cmd_ut = 0.0;
float   cmd_vt = 0.0;

float   cmd_ut_pre = 0.0;
float   cmd_vt_pre = 0.0;

float   Cur_Pos_m[3];
float   Cur_Vel_mps[3];
float   Cur_Att_rad[3];
float   Cur_Att[3];
float   cmd_rpy[3];
float   height_m = 0.0;

float   pos_x = 0.0;
float   pos_y = 0.0;
float   pos_z = 0.0;

float   cur_x = 0.0;
float   cur_y = 0.0;
float   cur_z = 0.0;
float   angle_err = 0.0;

float   takeoff_x = 0.0;
float   takeoff_y = 0.0;
float   init_heading = 0.0;

float   emer[2];
float   hover[3];
float   goal_dist = 10.0;
float   goal_heading = 0.0;
float   hover_heading = 0.0;

float   crusing_height = 0.0;
float   takeoff_height = 0.0;
float   sp_pos_cmd[3];

int     flag_takeoff = 0;
int     flag_landing = 0;
int     flag_goal = 0;
int     flag_turning = 0;
int     flag_hovering = 0;

int     turning_dir = 0;

uint8_t MotorAct = 0;
uint8_t FlightMode = 0;
int     flag_armed = 0;
int     count_ros = 0;
int     goal_service = 0;

float   goal[4];
float   goal_velx;
float   goal_velz;

float   sample_x[20];
float   sample_y[20];
float   sample_z[20];

float   temp_x[19];
float   temp_y[19];
float   temp_z[19];

float   target_odom[3];
float   prediction[3];
float   t_cur = 0.0;

float   flight_angle = 0.0;
float   del_sigma = 0.0;

int     flag_PosAvailable = 0;
char    filename[50];

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

void callback_detection(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    tar_data.flag_detect = msg->data[0];
    tar_data.size[0]     = msg->data[1];
    tar_data.size[1]     = msg->data[2];
    tar_data.impos[0]    = msg->data[3];
    tar_data.impos[1]    = msg->data[4];
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
    std::cout << "\n[USRG] state_cb(), -----------";
    std::cout << "\n          g_current_state.connected = " << ((g_current_state.connected) ? "OK!" : "Not yet!");
    std::cout << "\n          g_current_state.armed     = " << ((g_current_state.armed ) ? "OK!" : "Not yet!");
    std::cout << "\n          g_current_state.guided    = " << ((g_current_state.guided) ? "OK!" : "Not yet!");
    std::cout << "\n          g_current_state.mode      = " << g_current_state.mode;
    std::cout << "\n          Cur   X Y Z r             = " << Cur_Pos_m[0] << ", "<< Cur_Pos_m[1] << ", "<< Cur_Pos_m[2] << ", "<<  Cur_Att_rad[2]*R2D;
    std::cout << "\n          Path  X Y r               = " << path.x << ", "<< path.y << ", "<<  path.psi*R2D;
    std::cout << "\n          velocity auto             = " << cmd_x << ", "<< cmd_y << ", "<< cmd_z  << ", "<< cmd_r;
    std::cout << "\n          velocity output           = " << velcmd.twist.linear.x << ", "<<velcmd.twist.linear.y << ", "<<velcmd.twist.linear.z <<  ", "<<velcmd.twist.angular.z;
    std::cout << "\n          goal service              = " << goal_service;
    std::cout << "\n[USRG] ------------------------\n";
}

std_msgs::Float32MultiArray pathdata;
void callback_astar_path(const std_msgs::Float32MultiArray::ConstPtr& msg_input)
{
    pathdata = *msg_input;

    path.x = pathdata.data[0];
    path.y = pathdata.data[1];
    path.psi = pathdata.data[2];
}

void callback_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg_input)
{
    Local = *msg_input;

    Cur_Pos_m[0] = Local.pose.position.x;
    Cur_Pos_m[1] = Local.pose.position.y;
    Cur_Pos_m[2] = Local.pose.position.z;

    q[0] = Local.pose.orientation.x;
    q[1] = Local.pose.orientation.y;
    q[2] = Local.pose.orientation.z;
    q[3] = Local.pose.orientation.w;
    QuaterniontoEuler(Cur_Att_rad[0], Cur_Att_rad[1], Cur_Att_rad[2]);
}

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
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "uav21"));

    flag_PosAvailable = 1;
}

void callback_tar(const nav_msgs::Odometry::ConstPtr& msg_input)
{
    Tar_Odom = *msg_input;

    target_odom[0] = Tar_Odom.pose.pose.position.x;
    target_odom[1] = Tar_Odom.pose.pose.position.y;
    target_odom[2] = Tar_Odom.pose.pose.position.z;
}



void callback_local_vel(const geometry_msgs::TwistStamped::ConstPtr& msg_input)
{
    Localvel = *msg_input;

    Cur_Vel_mps[0] = Localvel.twist.linear.x;
    Cur_Vel_mps[1] = Localvel.twist.linear.y;
    Cur_Vel_mps[2] = Localvel.twist.linear.z;
}

void Local_Mission_Update(void);
void Mission_Update(void);
void Auto_Takeoff(void);
void Auto_Landing(void);
void WP_Flight(void);
void Path_Flight(void);
void Hovering(void);
void Tracking(void);
void Relative_WP_Flight(void);
void Visualization(void);

Mat polyfit(vector<cv::Point2f>& in_point, int n);

ros::Publisher pos_cur_pub, path_uav_pub, pos_pred_pub, pos_tar_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node_uav21");
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_pub;

    // Subscribe Topic
    ros::Subscriber state_sub     = nh_sub.subscribe ("/uav21/mavros/state" , 2,                         &callback_state);
    // ros::Subscriber local_pos_sub = nh_sub.subscribe ("/uav21/mavros/local_position/pose", 2,            &callback_local_pos);
    ros::Subscriber local_pos_sub = nh_sub.subscribe ("/uav21/mavros/global_position/local" , 2,         &callback_odom);
    // ros::Subscriber local_tar_sub = nh_sub.subscribe ("/uav22/odom" , 2,                                 &callback_tar);
    ros::Subscriber local_vel_sub = nh_sub.subscribe ("/uav21/mavros/local_position/velocity_local", 2,  &callback_local_vel);
    ros::Subscriber cmd_sub       = nh_sub.subscribe ("/uav21/mavros_comm_node/tele_key/cmd_vel", 2,     &callback_cmd_vel);
    ros::Subscriber flag_sub      = nh_sub.subscribe ("/uav21/mavros_comm_node/tele_key/flag", 2,        &callback_cmd_flag);

    ros::Subscriber astar_sub     = nh_sub.subscribe ("/uav21/astar_path_info", 2,                       &callback_astar_path);
    ros::Subscriber goal_sub      = nh_sub.subscribe ("/uav21/GoalAction", 2,                            &callback_goal);
    ros::Subscriber detection_sub = nh_sub.subscribe ("/uav21/detection", 2,                             &callback_detection);

    ros::Subscriber rc_in_sub = nh_sub.subscribe ("/uav21/mavros/rc/in", 2,                              &callback_rc_in);

    // Publish Topic
    ros::Publisher  local_vel_pub = nh_pub.advertise<geometry_msgs::TwistStamped>("/uav21/mavros/setpoint_velocity/cmd_vel", 2);

    path_uav_pub   = nh_pub.advertise<nav_msgs::Path>("uav_path",1);
    pos_cur_pub    = nh_pub.advertise<visualization_msgs::Marker>("uav_pos", 1);
    pos_tar_pub    = nh_pub.advertise<visualization_msgs::Marker>("uav_tar", 1);
    pos_pred_pub   = nh_pub.advertise<visualization_msgs::Marker>("uav_pred", 1);

    ros::ServiceClient  arming_client    = nh_pub.serviceClient<mavros_msgs::CommandBool> ("/uav21/mavros/cmd/arming");
    ros::ServiceClient  set_mode_client  = nh_pub.serviceClient<mavros_msgs::SetMode>     ("/uav21/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    // wait for FCU connection
    while(ros::ok() && g_current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ucmd = 0.0;
    vcmd = 0.0;
    wcmd = 0.0;
    rcmd = 0.0;

    cmd_x = 0.0;
    cmd_y = 0.0;
    cmd_z = 0.0;
    cmd_r = 0.0;

    cmd_ut_pre = Vx_track;
    cmd_vt_pre = 0.0;

    velcmd.twist.linear.x = 0.0;
    velcmd.twist.linear.y = 0.0;
    velcmd.twist.linear.z = 0.0;
    velcmd.twist.angular.z = 0.0;

    tele_flag.data.resize(5);
    body_vel.data.resize(4);
    hover_pt.data.resize(4);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_vel_pub.publish(velcmd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();
    printf("offboard start\n");

    // tele_flag.data[0] flag for arming    -- "t"
    // tele_flag.data[1] flag for auto      -- "g"

    mission.data = 6;  // [None]

    for(int ind=0; ind < 20; ind++)
    {
        sample_x[ind] = target_odom[0];
        sample_y[ind] = target_odom[1];
        sample_z[ind] = target_odom[2];
    }

    while(ros::ok())
    {
/*
        if( g_current_state.mode != "OFFBOARD" && ((ros::Time::now() - last_request) > ros::Duration(1.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (tele_flag.data[0] == 1)
            {
                if(flag_armed == 0)
                {
                    if((!g_current_state.armed) && (ros::Time::now() - last_request > ros::Duration(1.0)))
                    {
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            flag_armed = 1;
                            ROS_INFO("Vehicle armed");
                            MotorAct = 1;
                        }
                        last_request = ros::Time::now();
                    }
                }
            }
            else
            {
                if ((flag_armed == 1))
                {
                    flag_armed = 0;
                    ROS_INFO("Vehicle Disarmed");
                    arming_client.call(disarm_cmd);
                    MotorAct = 0;
                    mission.data = 0;  // [None]
                    last_request = ros::Time::now();
                    takeoff_request = ros::Time::now();
                }
            }
        }
*/
        //
        //printf("%d", subt_service.service);

        //if(flag_armed == 1)
	    if(g_current_state.mode == "OFFBOARD")
        {
            //goal_service = 6;
            Mission_Update();
            Visualization();
        }
        else
        {
            cmd_x = 0.0;
            cmd_y = 0.0;
            cmd_z = 0.0;
            cmd_r = 0.0;

            velcmd.twist.linear.x = 0.0;
            velcmd.twist.linear.y = 0.0;
            velcmd.twist.linear.z = 0.0;
            velcmd.twist.angular.z = 0.0;

            takeoff_x = Cur_Pos_m[0];
            takeoff_y = Cur_Pos_m[1];
            init_heading = Cur_Att_rad[2];
            goal_heading = Cur_Att_rad[2];

        }

#ifdef LOG_ON
        sprintf(filename,"/home/usrg/bin/data.txt");
        int train_fd = open(filename, O_WRONLY | O_APPEND , 0); // File Opening

        //                [0]  [1]  [2]  [3]
        sprintf(filename,"%.3f %.3f %.3f %.3f ", t_cur, Cur_Pos_m[0], Cur_Pos_m[1], Cur_Pos_m[2]);
        write(train_fd, filename,strlen(filename));

        //                [4]  [5]  [6]
        sprintf(filename,"%.3f %.3f %.3f ", Cur_Vel_mps[0], Cur_Vel_mps[1], Cur_Vel_mps[2]);
        write(train_fd, filename,strlen(filename));

        //                [7]  [8]  [9]  [10]
        sprintf(filename,"%.3f %.3f %.3f %.3f ", cmd_x, cmd_y, cmd_z, cmd_r);
        write(train_fd, filename,strlen(filename));

        //                [11] [12] [13]
        sprintf(filename,"%.3f %.3f %.3f ", target_odom[0], target_odom[1], target_odom[2]);
        write(train_fd, filename,strlen(filename));

        //                [14] [15] [16]
        sprintf(filename,"%.3f %.3f %.3f ", tar_data.pos[0], tar_data.pos[1], tar_data.pos[2]);
        write(train_fd, filename,strlen(filename));

        //                [17] [18] [19]
        sprintf(filename,"%.3f %.3f %.3f ", prediction[0], prediction[1], prediction[2]);
        write(train_fd, filename,strlen(filename));

        //               [20][21][22] [23] [24]
        sprintf(filename,"%d %d %.3f %.3f %.3f\n", goal_service, tar_data.flag_detect ,tar_data.a_n, del_sigma, flight_angle);
        write(train_fd, filename,strlen(filename));

        close(train_fd);
#endif

        // auto control mode
        ucmd =   cmd_x + cmd_RCx*cos(Cur_Att_rad[2]) + cmd_RCy*sin(Cur_Att_rad[2]);
        vcmd =   cmd_y + cmd_RCx*sin(Cur_Att_rad[2]) - cmd_RCy*cos(Cur_Att_rad[2]);
        wcmd =   cmd_z + tele_cmd.linear.z + cmd_RCz;
        rcmd =   cmd_r + tele_cmd.angular.z + cmd_RCr;

        //ucmd_LPF = LPF(ucmd, ucmd_pre, 4.0);
        //vcmd_LPF = LPF(vcmd, vcmd_pre, 4.0);
        //wcmd_LPF = LPF(wcmd, wcmd_pre, 4.0);
        //rcm1d_LPF = LPF(rcmd, rcmd_pre, 4.0);

        //ucmd_pre = ucmd_LPF;
        //vcmd_pre = vcmd_LPF;
        //wcmd_pre = wcmd_LPF;
        //rcmd_pre = rcmd_LPF;

        velcmd.twist.linear.x = ucmd;
        velcmd.twist.linear.y = vcmd;
        velcmd.twist.linear.z = wcmd;
        velcmd.twist.angular.z = -rcmd;

        local_vel_pub.publish(velcmd);

        count_ros = count_ros + 1;
        t_cur = count_ros/20.0;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void Mission_Update(void)
{

    switch (goal_service)
    {
    // Basic Mission
        case 1:
            Auto_Takeoff();
            break;

        case 2:
            Auto_Landing();
            break;

        case 3:
            WP_Flight();
            break;

        case 4:
            Path_Flight();
            break;

        case 5:
            Hovering();
            break;

        case 6:
            Tracking();
            break;

        case 11:
            Relative_WP_Flight();
            break;
        
        default:
            cmd_x = 0.0;
            cmd_y = 0.0;
            cmd_z = 0.0;
            cmd_r = 0.0;

            velcmd.twist.linear.x = 0.0;
            velcmd.twist.linear.y = 0.0;
            velcmd.twist.linear.z = 0.0;
            velcmd.twist.angular.z = 0.0;

            takeoff_x = Cur_Pos_m[0];
            takeoff_y = Cur_Pos_m[1];
            init_heading = Cur_Att_rad[2];
            goal_heading = Cur_Att_rad[2];

            goal_dist = 10.0;

            break;
    }
}


void Auto_Takeoff(void)
{
    cmd_x = satmax(Kpx*(takeoff_x - Cur_Pos_m[0]), goal_velx);
    cmd_y = satmax(Kpx*(takeoff_y - Cur_Pos_m[1]), goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);

    angle_err = GetNED_angle_err(init_heading, Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);
    mission.data = 1;  // [takeoff]

    if (fabs(goal[2] - Cur_Pos_m[2]) < 0.1)
    {
         goal_service = 5;
         hover[0] = Cur_Pos_m[0];
         hover[1] = Cur_Pos_m[1];
         hover[2] = goal[2];
         hover_heading = Cur_Att_rad[2];
    }
}


void Auto_Landing(void)
{
    cmd_x = satmax(Kpx*(goal[0] - Cur_Pos_m[0]),goal_velx);
    cmd_y = satmax(Kpx*(goal[1] - Cur_Pos_m[1]),goal_velx);
    cmd_z = goal_velz;

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    if (flag_landing != 1)
    {
        landing_request = ros::Time::now();
    }

    if (Cur_Pos_m[2] < 0.2)
    {
        cmd_z = -0.5;
        flag_landing = 1;
        if (ros::Time::now() - landing_request > ros::Duration(1.0))
        {
            flag_takeoff = 0;
            takeoff_x = Cur_Pos_m[0];
            takeoff_y = Cur_Pos_m[1];
            flag_landing = 0;
            goal_service = 0;
            takeoff_request = ros::Time::now();
        }
    }
}

void WP_Flight(void)
{
    cmd_x = satmax(Kpx*(goal[0] - Cur_Pos_m[0]),goal_velx);
    cmd_y = satmax(Kpx*(goal[1] - Cur_Pos_m[1]),goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]),goal_velz) + Kdz*(0.0 - Cur_Vel_mps[2]);

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);
    mission.data = 3;  // [fly_to]

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];

}

void Path_Flight(void)
{
    cmd_x = goal_velx * cos(path.psi);
    cmd_y = goal_velx * sin(path.psi);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];
}

void Hovering(void)
{
    cmd_x = satmax(Kpx*(hover[0] - Cur_Pos_m[0]),goal_velx);
    cmd_y = satmax(Kpx*(hover[1] - Cur_Pos_m[1]),goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]),goal_velz);

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);
    goal_heading = Cur_Att_rad[2];
}

void Tracking(void)
{
    //printf("Tracking\n");
    float velx_cmd = Vx_track;

    tar_data.imposfil[0] = tar_data.impos[0];
    tar_data.imposfil[1] = tar_data.impos[1];
    tar_data.sizefil[0] = tar_data.size[0];
    tar_data.sizefil[1] = tar_data.size[1];

    float diag = sqrt(tar_data.sizefil[0]*tar_data.sizefil[0] + tar_data.sizefil[1]*tar_data.sizefil[1]);
    diag = satmax(satmin(diag, 20.0),140.0);
    //tar_data.dist = 1195.5*pow(diag,-1.477);

    float dx = target_odom[0] - Cur_Pos_m[0];
    float dy = target_odom[1] - Cur_Pos_m[1];
    float dz = target_odom[2] - Cur_Pos_m[2];
    tar_data.dist = sqrt(dx*dx + dy*dy);
    //tar_data.dist = satmax(satmin(tar_data.dist, 0.5), 10.0);

    float phi   = Cur_Att_rad[0];
    float theta = Cur_Att_rad[1];
    float psi   = Cur_Att_rad[2];

    float R11 = cos(theta);
    float R12 = sin(phi)*sin(theta);
    float R13 = cos(phi)*sin(theta);

    float R21 = 0.0;
    float R22 = cos(phi);
    float R23 = -1*sin(phi);

    float R31 = -1*sin(theta);
    float R32 = sin(phi)*cos(theta);

    float R33 = cos(phi)*cos(theta);

    float u = (float)(tar_data.imposfil[0] - CAMERA_PARAM_U0)*-1.0;
    float v = (float)(tar_data.imposfil[1] - CAMERA_PARAM_V0)*-1.0;

    float rho = sqrt(u*u + v*v);
    float F = a0 + a1*rho + a2*pow(rho,2) + a3*pow(rho,3) + a4*pow(rho,4);

    float s = (R11*F + R12*u + R13*v)/tar_data.dist;
    float y_tar = -1*(R21*F + R22*u + R23*v)/s;
    float z_tar = -1*(R31*F + R32*u + R33*v)/s;

    tar_data.pos[0] = Cur_Pos_m[0] + tar_data.dist * cos(Cur_Att_rad[2]) - y_tar * sin(Cur_Att_rad[2]);
    tar_data.pos[1] = Cur_Pos_m[1] + tar_data.dist * sin(Cur_Att_rad[2]) + y_tar * cos(Cur_Att_rad[2]);
    tar_data.pos[2] = Cur_Pos_m[2] + z_tar;

    visualization_msgs::Marker Target;
    Target.type = visualization_msgs::Marker::CUBE;
    Target.header.frame_id = "map";
    Target.scale.x = Target.scale.y = Target.scale.z = 0.5;
    Target.color.r = 1.0;
    Target.color.b = 0.2;
    Target.color.g = 0.2;
    Target.color.a = 1.0;

    Target.pose.position.x = tar_data.pos[0];
    Target.pose.position.y = tar_data.pos[1];
    Target.pose.position.z = tar_data.pos[2];
    Target.pose.orientation.w = 1.0;
    Target.ns = '2';
    pos_tar_pub.publish(Target);

    //TargetArray.markers.push_back(Target);

    printf("[tar] %.3f %.3f %.3f [NED] %.3f %.3f %.3f\n", tar_data.dist, y_tar, z_tar, tar_data.pos[0], tar_data.pos[1], tar_data.pos[2]);

    sample_x[0] = tar_data.pos[0];
    sample_y[0] = tar_data.pos[1];
    sample_z[0] = tar_data.pos[2];

    /*
    for(int ind=0; ind < 10; ind++)
    {
        printf("%.3f, %.3f, %.3f\n",sample_x[ind],sample_y[ind],sample_z[ind]);
    }
    */

    vector<cv::Point2f> point_x, point_y, point_z;
    cv::Point2f p1, p2, p3;

    p1.x = -1.0;
    p1.y = sample_x[19];
    point_x.push_back(p1);

    p2.x = -0.5;
    p2.y = sample_x[10];
    point_x.push_back(p2);

    p3.x = -0.0;
    p3.y = sample_x[0];
    point_x.push_back(p3);

    Mat mat_x = polyfit(point_x, 2);

    p1.x = -1.0;
    p1.y = sample_y[19];
    point_y.push_back(p1);

    p2.x = -0.5;
    p2.y = sample_y[10];
    point_y.push_back(p2);

    p3.x = -0.0;
    p3.y = sample_y[0];
    point_y.push_back(p3);

    Mat mat_y = polyfit(point_y, 2);

    p1.x = -1.0;
    p1.y = sample_z[19];
    point_z.push_back(p1);

    p2.x = -0.5;
    p2.y = sample_z[10];
    point_z.push_back(p2);

    p3.x = -0.0;
    p3.y = sample_z[0];
    point_z.push_back(p3);

    Mat mat_z = polyfit(point_z, 2);

    float tp = 0.5;
    prediction[0] = mat_x.at<double>(2, 0)*tp*tp + mat_x.at<double>(1, 0)*tp + mat_x.at<double>(0, 0);
    prediction[1] = mat_y.at<double>(2, 0)*tp*tp + mat_y.at<double>(1, 0)*tp + mat_y.at<double>(0, 0);
    prediction[2] = mat_z.at<double>(2, 0)*tp*tp + mat_z.at<double>(1, 0)*tp + mat_z.at<double>(0, 0);

    visualization_msgs::Marker Pred;
    Pred.type = visualization_msgs::Marker::CUBE;
    Pred.header.frame_id = "map";
    Pred.scale.x = Pred.scale.y = Pred.scale.z = 0.5;
    Pred.color.r = 0.2;
    Pred.color.b = 0.1;
    Pred.color.g = 1.0;
    Pred.color.a = 1.0;

    Pred.pose.position.x = prediction[0];
    Pred.pose.position.y = prediction[1];
    Pred.pose.position.z = prediction[2];
    Pred.pose.orientation.w = 1.0;
    Pred.ns = '3';
    pos_pred_pub.publish(Pred);


    for(int ind=0; ind < 20; ind++)
    {
        temp_x[ind] = sample_x[ind];
        temp_y[ind] = sample_y[ind];
        temp_z[ind] = sample_z[ind];
    }

    for(int ind=0; ind < 19; ind++)
    {
        sample_x[ind+1] = temp_x[ind];
        sample_y[ind+1] = temp_y[ind];
        sample_z[ind+1] = temp_z[ind];
    }

    tar_data.psi_ref = atan((tar_data.imposfil[0] - CAMERA_PARAM_U0)/CAMERA_PARAM_a);
    float tar_vel = sqrt(tar_data.vel[0]*tar_data.vel[0] + tar_data.vel[1]*tar_data.vel[1]);

    //velx_cmd = function_dist(tar_data.dist)*velx_cmd;
    float delT = 0.05;

    float del_rx = tar_data.pos[0] - Cur_Pos_m[0];
    float del_ry = tar_data.pos[1] - Cur_Pos_m[1];
    //float del_rx = prediction[0] - Cur_Pos_m[0];
    //float del_ry = prediction[1] - Cur_Pos_m[1];

    float del_vx = tar_data.vel[0] - Cur_Vel_mps[0];
    float del_vy = tar_data.vel[1] - Cur_Vel_mps[1];

    float del_r = sqrt(del_rx * del_rx + del_ry * del_ry);
    del_sigma = atan(del_ry / (del_rx + eps));


    float cur_vel = sqrt(Cur_Vel_mps[0]*Cur_Vel_mps[0] + Cur_Vel_mps[1]*Cur_Vel_mps[1]);

    if (cur_vel < 0.5)
    {
        flight_angle = Cur_Att_rad[2];
    }
    else
    {
        flight_angle = atan2(Cur_Vel_mps[1], Cur_Vel_mps[0]+eps);
    }

    tar_data.a_n = -1.0*Vx_track*Vx_track / (del_r + eps) * (-6*del_sigma + 4*flight_angle + 2*Cur_Att_rad[2]);

    /// pursuit guidance
    //tar_data.a_n = 3*(del_sigma-flight_angle)*velx_cmd;

    cmd_ut = cmd_ut_pre - tar_data.a_n*sin(tar_data.beta)*delT;
    cmd_vt = cmd_vt_pre + tar_data.a_n*cos(tar_data.beta)*delT;

    tar_data.beta = atan2(cmd_vt,cmd_ut+eps);

    float cmd_vel = sqrt(cmd_ut*cmd_ut + cmd_vt*cmd_vt);

    cmd_ut_pre = Vx_track/(cmd_vel+eps)*cmd_ut;
    cmd_vt_pre = Vx_track/(cmd_vel+eps)*cmd_vt;

    angle_err = 2.0*wrap(tar_data.psi_ref);

    float velz_cmd  = Kpz * (2.5 - Cur_Pos_m[2]);
    //float velz_cmd  = 0.0

    cmd_x = velx_cmd*cos(Cur_Att_rad[2]);
    cmd_y = velx_cmd*sin(Cur_Att_rad[2]);

    //cmd_x = cmd_ut;
    //cmd_y = cmd_vt;
    cmd_z = satmax(velz_cmd, VZ_MAX);
    cmd_r = satmax(Kr*angle_err, R_MAX);

    if (tar_data.dist < 1.0)
    {
        goal_service = 5;
    }
}

void Relative_WP_Flight(void)
{
    cmd_x_raw = satmax(Kpx_rel*(goal[0]*cos(Cur_Att_rad[2]) - goal[1]*sin(Cur_Att_rad[2])),goal_velx);
    cmd_y_raw = satmax(Kpx_rel*(goal[0]*sin(Cur_Att_rad[2]) + goal[1]*cos(Cur_Att_rad[2])),goal_velx);
    cmd_z_raw = satmax(Kpz_rel*goal[2],goal_velz) + Kdz_rel*(0.0 - Cur_Vel_mps[2]);

    cmd_x = LPF(cmd_x_raw, cmd_x_pre, 10.0);
    cmd_y = LPF(cmd_y_raw, cmd_y_pre, 10.0);
    cmd_z = LPF(cmd_z_raw, cmd_z_pre, 10.0);

    cmd_x_pre = cmd_x;
    cmd_y_pre = cmd_y;
    cmd_z_pre = cmd_z;

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr_rel*angle_err, R_MAX);
    // mission.data = 11;  // [fly_to]

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = Cur_Pos_m[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];
}

void Visualization(void)
{
    if (flag_PosAvailable == 1)
    {
        geometry_msgs::PoseStamped cur_pose;
        path_cur.header.stamp = ros::Time::now();
        path_cur.header.frame_id = "map";
        cur_pose.pose.position.x = Cur_Pos_m[0];
        cur_pose.pose.position.y = Cur_Pos_m[1];
        cur_pose.pose.position.z = Cur_Pos_m[2];
        cur_pose.pose.orientation.x = q[0];
        cur_pose.pose.orientation.y = q[1];
        cur_pose.pose.orientation.z = q[2];
        cur_pose.pose.orientation.w = q[3];
        path_cur.poses.push_back(cur_pose);
        path_uav_pub.publish(path_cur);

        visualization_msgs::Marker Pose;
        Pose.type = visualization_msgs::Marker::CUBE;
        Pose.header.frame_id = "map";
        Pose.scale.x = Pose.scale.y = Pose.scale.z = 0.5;
        Pose.color.r = 0.2;
        Pose.color.b = 1.0;
        Pose.color.g = 0.2;
        Pose.color.a = 1.0;

        Pose.pose.position.x = Cur_Pos_m[0];
        Pose.pose.position.y = Cur_Pos_m[1];
        Pose.pose.position.z = Cur_Pos_m[2];
        Pose.pose.orientation.x = q[0];
        Pose.pose.orientation.y = q[1];
        Pose.pose.orientation.z = q[2];
        Pose.pose.orientation.w = q[3];
        Pose.ns = '1';
        pos_cur_pub.publish(Pose);
    }

}



Mat polyfit(vector<cv::Point2f>& in_point, int n)
{
    int size = in_point.size();

    int x_num = n + 1;

    Mat mat_u(size, x_num, CV_64F);
    Mat mat_y(size, 1, CV_64F);

    for (int i = 0; i < mat_u.rows; ++i)
            for (int j = 0; j < mat_u.cols; ++j)
            {
                    mat_u.at<double>(i, j) = pow(in_point[i].x, j);
            }

    for (int i = 0; i < mat_y.rows; ++i)
    {
            mat_y.at<double>(i, 0) = in_point[i].y;
    }

    Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
    //cout << mat_k << endl;
    return mat_k;
}
