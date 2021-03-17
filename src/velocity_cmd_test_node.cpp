
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <unistd.h>
#include <math.h>

#include "SubtUtil.h"

mavros_msgs::State             g_current_state;
geometry_msgs::PoseStamped     Pos;
geometry_msgs::TwistStamped    Localvel;

std_msgs::Float32MultiArray    goal_pub_data;
tf::Quaternion                 quat;

float   cmd_x = 0.0;
float   cmd_y = 0.0;
float   cmd_z = 0.0;
float   cmd_r = 0.0;

float   Cur_Pos_m[3];
float   Cur_Vel_mps[3];
float   Cur_Att_rad[3];
float   Cur_Att[3];

int     count_ros = 0;
int     goal_service = 0;

float   goal[4];

int     count_published = 0;
float   vel_cmd_set[32][3] = {
/// 2hz, cmd change 2.5s
/*
        {   0.5,   0.0,  3.0 },
        {   0.5,   0.0,  3.0 },
        {   0.5,   0.0,  3.0 },
        {   0.5,   0.0,  3.0 },

        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {  -0.5,   0.0,  3.0 },
        {  -0.5,   0.0,  3.0 },
        {  -0.5,   0.0,  3.0 },
        {  -0.5,   0.0,  3.0 },

        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.0,   0.5,  3.0 },
        {   0.0,   0.5,  3.0 },
        {   0.0,   0.5,  3.0 },
        {   0.0,   0.5,  3.0 },

        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.0,  -0.5,  3.0 },
        {   0.0,  -0.5,  3.0 },
        {   0.0,  -0.5,  3.0 },
        {   0.0,  -0.5,  3.0 },

        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
*/

/// 2hz, cmd change 0.5s
        {   0.5,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.5,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.5,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {  -0.5,   0.0,  3.0 },
        {  -0.0,   0.0,  3.0 },
        {  -0.5,   0.0,  3.0 },
        {  -0.0,   0.0,  3.0 },

        {  -0.5,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.0,   0.5,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.5,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.0,   0.5,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.0,  -0.5,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,  -0.5,  3.0 },
        {   0.0,   0.0,  3.0 },

        {   0.0,  -0.5,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
        {   0.0,   0.0,  3.0 },
    };

void callback_state(const mavros_msgs::State::ConstPtr& msg)
{
    g_current_state = *msg;

    system("clear");
    std::cout << "\n[KAIST - Velocity Test] state_cb(), -----------";
    std::cout << "\n         g_current_state.connected  = " << ((g_current_state.connected) ? "OK!" : "Not yet!");
    std::cout << "\n         g_current_state.armed      = " << ((g_current_state.armed ) ? "OK!" : "Not yet!");
    std::cout << "\n         g_current_state.guided     = " << ((g_current_state.guided) ? "OK!" : "Not yet!");
    std::cout << "\n         g_current_state.mode       = " << g_current_state.mode;
    std::cout << "\n         Cur   X Y Z r              = " << Cur_Pos_m[0] << ", "<< Cur_Pos_m[1] << ", "<< Cur_Pos_m[2] << ", "<<  Cur_Att_rad[2]*R2D;
    std::cout << "\n         velocity auto command      = " << cmd_x << ", "<< cmd_y << ", "<< cmd_z  << ", "<< cmd_r;
    std::cout << "\n         velocity drone             = " << Cur_Vel_mps[0] << ", "<< Cur_Vel_mps[1] << ", "<< Cur_Vel_mps[2];
    std::cout << "\n         count published            = " << count_published;
    std::cout << "\n         goal service               = " << goal_service;
    std::cout << "\n[KAIST - Velocity Test] ------------------------\n";
}

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
}

void callback_local_vel(const geometry_msgs::TwistStamped::ConstPtr& msg_input)
{
    Localvel = *msg_input;

    Cur_Vel_mps[0] = Localvel.twist.linear.x;
    Cur_Vel_mps[1] = Localvel.twist.linear.y;
    Cur_Vel_mps[2] = Localvel.twist.linear.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_mission_node");
    ROS_INFO("INIT velocity mission node");
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_pub;

    // Subscribe Topic
    ros::Subscriber state_sub     = nh_sub.subscribe ("/mavros/state" , 2,                         &callback_state);
    ros::Subscriber local_pos_sub = nh_sub.subscribe ("/mavros/local_position/pose" , 2,           &callback_pose);
    ros::Subscriber local_vel_sub = nh_sub.subscribe ("/mavros/local_position/velocity_local", 2,  &callback_local_vel);

    // Publish Topic
    ros::Publisher  goal_pub = nh_pub.advertise<std_msgs::Float32MultiArray>("/GoalAction", 2);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    int count_ros = 0;
    int index_cmd = 0;
    while(ros::ok()){
        // std::cout << "index: " << index_cmd << ", count: " << count_ros << endl; 
	    // if(!g_current_state.connected && g_current_state.mode == "OFFBOARD" && count_ros%10 == 0)
	    if(count_ros%10 == 0 && g_current_state.mode == "OFFBOARD")
        {
            // std::cout << "index: " << index_cmd << ", count: " << count_ros << endl;
            goal_service = 11;
            cmd_x = vel_cmd_set[index_cmd][0];
            cmd_y = vel_cmd_set[index_cmd][1];
            cmd_z = vel_cmd_set[index_cmd][2];
            cmd_r = 0;

            goal_pub_data.data.resize(7);
            goal_pub_data.data[0] = goal_service;
            goal_pub_data.data[1] = cmd_x;
            goal_pub_data.data[2] = cmd_y;
            goal_pub_data.data[3] = cmd_z;
            goal_pub_data.data[4] = cmd_r;
            goal_pub_data.data[5] = 1.0;
            goal_pub_data.data[6] = 1.0;
            goal_pub.publish(goal_pub_data);
            // std::cout << "\ngoal_data published\n";
            count_published++;

            index_cmd++;
            if (index_cmd >= 32)
                index_cmd = 0;
        }
        count_ros++;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

