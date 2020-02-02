#ifndef DefineList_H
#define DefineList_H

nav_msgs::Odometry             pose;
tf2::Quaternion                quaternion;

std_msgs::Float32MultiArray    tele_flag;
geometry_msgs::Twist           tele_cmd;

std_msgs::Float64              MAV_Compass;
mavros_msgs::State             MAV_Current_State;
geometry_msgs::TwistStamped    MAV_Cmd_Vel;

geometry_msgs::PoseStamped     MAV_Local_Pose;
geometry_msgs::TwistStamped    MAV_Local_Vel;
geometry_msgs::PoseStamped     MAV_Tar_Pose;
geometry_msgs::TwistStamped    MAV_Tar_Vel;

std_msgs::UInt8                RL_Mission;

ros::Subscriber  sub_mav_state;
ros::Subscriber  sub_mav_comp;
ros::Subscriber  sub_cmd;
ros::Subscriber  sub_flag;
ros::Subscriber  sub_local_pos;
ros::Subscriber  sub_local_vel;
ros::Subscriber  sub_tar_pos;
ros::Subscriber  sub_tar_vel;
ros::Subscriber  sub_mission;

ros::Publisher   pub_local_vel;

ros::ServiceClient  arming_client;
ros::ServiceClient  set_mode_client;

float angle_err = 0.0;
float takeoff_height = 0.0;

float a_n = 0.0;
float delPos[3];

struct struct_offboard
{
    float Cmd_Output[4];    // [0]:ucmd [1]:vcmd [2]:wcmd [3]:rcmd
    float Cmd_Auto[4];

    float Cur_Att_rad[3];
    float Cur_pqr_rps[3];
    float Cur_Pos_m[3];
    float Cur_VelNED_mps[3];
};

struct struct_target
{
    float Cur_Att_rad[3];
    float Cur_pqr_rps[3];
    float Cur_Pos_m[3];
    float Cur_VelNED_mps[3];
};

struct struct_mission
{
    float takeoff[4];
    float landing[4];
    float hover[4];
    float tracking[3];

    float beta = 0.0;
    float cmd_x = 0.0;
    float cmd_y = 0.0;

    int   goal_service;
};

struct struct_flag
{
    bool armed = 0;
};


struct_offboard  offstate;
struct_target    tarstate;
struct_mission   mission;
struct_flag      flag;

#endif
