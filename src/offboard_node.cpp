
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

constexpr double const_pi() { return atan(1)*4; }
constexpr double const_R2D() { return 180.0 / const_pi(); }
constexpr double const_D2R() { return const_pi() / 180.0; }

class Px4Drone {
private:
    ros::Subscriber m_state_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_goal_action_sub;
    ros::Publisher m_cmd_vel_pub;
    ros::Publisher m_cmd_pos_pub;
    ros::Publisher m_uav_path_pub;
    ros::Publisher m_uav_pos_pub;
    ros::ServiceClient m_arming_client;
    ros::ServiceClient m_set_mode_client;
    ros::ServiceClient m_param_set_client;
    ros::ServiceClient m_param_get_client;

    mavros_msgs::State m_state;
    nav_msgs::Odometry m_odom;
    mavros_msgs::CommandBool m_arm_cmd;
    mavros_msgs::SetMode m_set_mode;
    mavros_msgs::ParamSet m_param_set;
    mavros_msgs::ParamGet m_param_get;

    tf::Vector3 m_cur_pos_m;
    tf::Vector3 m_cur_vel_mps;
    tf::Quaternion m_cur_q;
    tf::Vector3 m_cur_rpy_rad;
    tf::Vector3 m_cur_rpy_deg;
    tf::Vector3 m_cur_rpy_rate_dps;

    float m_takeoff_alt_m;
    float m_takeoff_spd_mps;
    bool m_auto_override;
    bool m_offboard_override;

    std_msgs::Float32MultiArray m_goal_action;
    int m_goal_service;
    double m_goal_x;
    double m_goal_y;
    double m_goal_z;
    double m_goal_r;
    float m_ros_rate;

public:
    Px4Drone(ros::NodeHandle *nh_);
    ~Px4Drone();
    void cbState(const mavros_msgs::State::ConstPtr& msg_);
    void cbOdom(const nav_msgs::Odometry::ConstPtr& msg_);
    void cbGoalAction(const std_msgs::Float32MultiArray::ConstPtr& msg_);

    void showState(void);
    void showOdom(void);

    mavros_msgs::State getState(void);
    nav_msgs::Odometry getOdom();
    float getTakeOffAlt(void);
    float getTakeOffSpd(void);
    float getAutoOverride(void);
    float getOffboardOverride(void);
    float getGoalService(void);
    float getGoalX(void);
    float getGoalY(void);
    float getGoalZ(void);
    float getGoalR(void);
    float getRosRate(void);

    bool setComRCOverride(bool auto_override_, bool offboard_override_);
    bool setArm(void);
    bool setDisarm(void);
    bool setAcro(void);
    bool setStabilized(void);
    bool setAltctl(void);
    bool setPosctl(void);
    bool setOffboard(void);

    bool goTakeOff(float takeoff_alt_m_, float takeoff_spd_mps_);
    bool goLanding(void);
    void goForce(void);
    void goAccel(void);
    void goAngularVelocity(void);
    void goAttitude(void);
    void goVelocity(double x_east_mps_, double y_north_mps_, double z_up_mps_, double heading_ccw_deg_);
    void goVelocityBody(double x_forward_mps_, double y_left_mps_, double z_up_mps_, double heading_ccw_deg_);
    void goPosition(double x_east_m_, double y_north_m_, double z_up_m_, double heading_ccw_deg_);

    void doMission(int goal_service_, double goal_x_east_, double goal_y_north_, double goal_z_up_, double goal_heading_CCW_, ros::Rate rate_);

    void pubRvizTopics(void);
};

Px4Drone::Px4Drone(ros::NodeHandle *nh_)
    : m_cur_pos_m(0,0,0),
        m_cur_vel_mps(0,0,0),
        m_cur_q(0,0,0,0),
        m_cur_rpy_rad(0,0,0),
        m_cur_rpy_deg(0,0,0),
        m_cur_rpy_rate_dps(0,0,0),
        m_takeoff_alt_m(5.0), // default: 2.5 [m]
        m_takeoff_spd_mps(1.0), // default: 1.5 [m/s]
        m_auto_override(true), // default: true
        m_offboard_override(true), // default: false
        m_goal_service(-1),
        m_ros_rate(50.0)
{
    m_state_sub = nh_->subscribe ("/mavros/state", 10, &Px4Drone::cbState, this);
    m_odom_sub = nh_->subscribe ("/mavros/local_position/odom", 10, &Px4Drone::cbOdom, this);
    m_goal_action_sub = nh_->subscribe ("/GoalAction", 10, &Px4Drone::cbGoalAction, this);
    m_cmd_vel_pub = nh_->advertise <geometry_msgs::TwistStamped> ("/mavros/setpoint_velocity/cmd_vel", 10);
    m_cmd_pos_pub = nh_->advertise <geometry_msgs::PoseStamped> ("/mavros/setpoint_position/local", 10);
    m_uav_path_pub = nh_->advertise <nav_msgs::Path> ("/uav_path", 10);
    m_uav_pos_pub = nh_->advertise <visualization_msgs::Marker> ("/uav_pos", 10);
    m_arming_client = nh_->serviceClient <mavros_msgs::CommandBool> ("/mavros/cmd/arming");
    m_set_mode_client = nh_->serviceClient <mavros_msgs::SetMode> ("/mavros/set_mode");
    m_param_set_client = nh_->serviceClient <mavros_msgs::ParamSet> ("/mavros/param/set");
    m_param_get_client = nh_->serviceClient <mavros_msgs::ParamGet> ("/mavros/param/get");
}

Px4Drone::~Px4Drone() {

}

void Px4Drone::cbState(const mavros_msgs::State::ConstPtr& msg_) {
    m_state = *msg_;
    return;
}

void Px4Drone::cbOdom(const nav_msgs::Odometry::ConstPtr& msg_) {
    m_odom = *msg_;

    m_cur_pos_m.setValue(m_odom.pose.pose.position.x, 
        m_odom.pose.pose.position.y, 
        m_odom.pose.pose.position.z);

    m_cur_q.setValue(m_odom.pose.pose.orientation.x, 
        m_odom.pose.pose.orientation.y, 
        m_odom.pose.pose.orientation.z, 
        m_odom.pose.pose.orientation.w);

    m_cur_vel_mps.setValue(m_odom.twist.twist.linear.x,
        m_odom.twist.twist.linear.y,
        m_odom.twist.twist.linear.z);

    m_cur_rpy_rate_dps.setValue(m_odom.twist.twist.angular.x,
        m_odom.twist.twist.angular.y,
        m_odom.twist.twist.angular.z);

    tf::Matrix3x3 m_cur_mat(m_cur_q);
    static double yaw, pitch, roll;
    m_cur_mat.getEulerYPR(yaw, pitch, roll);
    m_cur_rpy_rad.setValue(roll, pitch, yaw);
    m_cur_rpy_deg = m_cur_rpy_rad *const_R2D();
    
    static tf::TransformBroadcaster br;
    static tf::Transform transform;
    transform.setOrigin(m_cur_pos_m);
    transform.setRotation(m_cur_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "uav"));
    return;
}

void Px4Drone::cbGoalAction(const std_msgs::Float32MultiArray::ConstPtr& msg_) {
    m_goal_action = *msg_;
    m_goal_service = (int)m_goal_action.data[0];
    m_goal_x = m_goal_action.data[1];
    m_goal_y = m_goal_action.data[2];
    m_goal_z = m_goal_action.data[3];
    m_goal_r = m_goal_action.data[4];
    return;
}

void Px4Drone::showState(void) {
    static const string enum_system_status[9] = \
        {"UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE", 
        "CRITICAL", "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION"};

    cout << endl;
    cout << "\t[Px4Drone] State\t(/mavros/state)" << endl;
    cout << "m_state.connected     = " << (m_state.connected ? "OK!" : "Not yet!") << endl;
    cout << "m_state.armed         = " << (m_state.armed ? "OK!" : "Not yet!") << endl;
    cout << "m_state.guided        = " << (m_state.guided ? "OK!" : "Not yet!") << endl;
    cout << "m_state.manual_input  = " << (m_state.manual_input ? "OK!" : "Not yet!") << endl;
    cout << "m_state.mode          = " << m_state.mode << endl;
    cout << "m_state.system_status = " << "MAV_STATE_" << enum_system_status[m_state.system_status+1] << endl;
    cout << endl;
    return;
}

void Px4Drone::showOdom(void) {
    cout << endl;
    cout << "\t[Px4Drone] Odometry\t(/mavros/local_position/odom)" << endl;
    cout << "m_odom.position_m [x,y,z]        = [" << m_cur_pos_m.getX() << \
        ",\t" << m_cur_pos_m.getY() << ",\t" << m_cur_pos_m.getZ() << "]" << endl;
    cout << "m_odom.velocity_mps [x,y,z]      = [" << m_cur_vel_mps.getX() << \
        ",\t" << m_cur_vel_mps.getY() << ",\t" << m_cur_vel_mps.getZ() << "]" << endl;
    cout << "m_odom.orientation_rad [x,y,z,w] = [" << m_cur_q.getX() << \
        ",\t" << m_cur_q.getY() << ",\t" << m_cur_q.getZ() << ",\t" << m_cur_q.getW() << "]" << endl;
    cout << "m_odom.rpy_deg [r,p,y]           = [" << m_cur_rpy_deg.getX() << \
        ",\t" << m_cur_rpy_deg.getY() << ",\t" << m_cur_rpy_deg.getZ() << "]" << endl;
    cout << "m_odom.rpy_rate_deg [r,p,y]      = [" << m_cur_rpy_rate_dps.getX() << \
        ",\t" << m_cur_rpy_rate_dps.getY() << ",\t" << m_cur_rpy_rate_dps.getZ() << "]" << endl;
    cout << endl;
    return;
}

mavros_msgs::State Px4Drone::getState(void) {
    return m_state;
}

nav_msgs::Odometry Px4Drone::getOdom(void) {
    return m_odom;
}

float Px4Drone::getTakeOffAlt(void) {
    return m_takeoff_alt_m;
}

float Px4Drone::getTakeOffSpd(void) {
    return m_takeoff_spd_mps;
}

float Px4Drone::getAutoOverride(void) {
    return m_auto_override;
}

float Px4Drone::getOffboardOverride(void) {
    return m_offboard_override;
}

float Px4Drone::getGoalService(void) {
    return m_goal_service;
}

float Px4Drone::getGoalX(void) {
    return m_goal_x;
}

float Px4Drone::getGoalY(void) {
    return m_goal_y;
}

float Px4Drone::getGoalZ(void) {
    return m_goal_z;
}

float Px4Drone::getGoalR(void) {
    return m_goal_r;
}

float Px4Drone::getRosRate(void) {
    return m_ros_rate;
}

bool Px4Drone::setComRCOverride(bool auto_override_, bool offboard_override_) {
    m_param_set.request.param_id = "COM_RC_OVERRIDE";
    m_param_set.request.value.integer = (auto_override_ ? 1:0) | (offboard_override_? 2:0);
    m_param_set_client.call(m_param_set);
    return m_param_set.response.success;
}

bool Px4Drone::setArm(void) {
    m_arm_cmd.request.value = true;
    m_arming_client.call(m_arm_cmd);
    return m_arm_cmd.response.success;
}

bool Px4Drone::setDisarm(void) {
    m_arm_cmd.request.value = false;
    m_arming_client.call(m_arm_cmd);
    return m_arm_cmd.response.success;
}

bool Px4Drone::setAcro(void) {
    m_set_mode.request.custom_mode = "ACRO";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

bool Px4Drone::setStabilized(void) {
    m_set_mode.request.custom_mode = "STABILIZED";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

bool Px4Drone::setAltctl(void) {
    m_set_mode.request.custom_mode = "ALTCTL";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

bool Px4Drone::setPosctl(void) {
    m_set_mode.request.custom_mode = "POSCTL";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

bool Px4Drone::setOffboard(void) {
    m_set_mode.request.custom_mode = "OFFBOARD";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

bool Px4Drone::goTakeOff(float takeoff_alt_m_, float takeoff_spd_mps_) {
    bool ack_takeoff_alt, ack_takeoff_spd;
    m_param_set.request.param_id = "MIS_TAKEOFF_ALT";
    m_param_set.request.value.real = takeoff_alt_m_;
    m_param_set_client.call(m_param_set);
    m_param_set.request.param_id = "MPC_TKO_SPEED";
    m_param_set.request.value.real = takeoff_spd_mps_;
    m_param_set_client.call(m_param_set);

    m_param_get.request.param_id = "MIS_TAKEOFF_ALT";
    m_param_get_client.call(m_param_get);
    ack_takeoff_alt = m_param_get.response.success;
    m_param_get.request.param_id = "MPC_TKO_SPEED";
    m_param_get_client.call(m_param_get);
    ack_takeoff_spd = m_param_get.response.success;
    
    if (ack_takeoff_alt && ack_takeoff_spd) {
        m_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        m_set_mode_client.call(m_set_mode);
        return m_set_mode.response.mode_sent;
    }
    else {
        return false;
    }
}

bool Px4Drone::goLanding(void) {
    m_set_mode.request.custom_mode = "AUTO.LAND";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

void Px4Drone::goVelocity(double x_east_mps_, double y_north_mps_, double z_up_mps_, double heading_CCW_dps_) {
    geometry_msgs::TwistStamped targetVelocity;
    targetVelocity.twist.linear.x = x_east_mps_;
    targetVelocity.twist.linear.y = y_north_mps_;
    targetVelocity.twist.linear.z = z_up_mps_;
    targetVelocity.twist.angular.z = heading_CCW_dps_;
    m_cmd_vel_pub.publish(targetVelocity);
    return;
}

void Px4Drone::goVelocityBody(double x_forward_mps_, double y_left_mps_, double z_up_mps_, double heading_CCW_dps_) {
    geometry_msgs::TwistStamped targetVelocityBody;
    targetVelocityBody.twist.linear.x = x_forward_mps_ *cos(m_cur_rpy_rad.getZ()) - y_left_mps_ *sin(m_cur_rpy_rad.getZ());
    targetVelocityBody.twist.linear.y = x_forward_mps_ *sin(m_cur_rpy_rad.getZ()) + y_left_mps_ *cos(m_cur_rpy_rad.getZ());
    targetVelocityBody.twist.linear.z = z_up_mps_;
    targetVelocityBody.twist.angular.z = heading_CCW_dps_;
    m_cmd_vel_pub.publish(targetVelocityBody);
    return;
}

void Px4Drone::goPosition(double x_east_m_, double y_north_m_, double z_up_m_, double heading_CCW_deg_) {
    geometry_msgs::PoseStamped targetPosition;
    targetPosition.pose.position.x = x_east_m_;
    targetPosition.pose.position.y = y_north_m_;
    targetPosition.pose.position.z = z_up_m_;
    quaternionTFToMsg(tf::Quaternion(0, 0, heading_CCW_deg_ *const_D2R()), targetPosition.pose.orientation);
    m_cmd_pos_pub.publish(targetPosition);
    return;
}

void Px4Drone::doMission(int goal_service_, double goal_x_, double goal_y_, double goal_z_, double goal_r_, ros::Rate rate_) {
    switch(goal_service_) {
        case -1: // Initial value
            break;

        case 0: // Auto arm(POSCTL) - takeoff - offboard
            // wait for POSCTL Mode
            this->setPosctl();
            while(ros::ok() && !(m_state.mode == "POSCTL")){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "POSCTL Mode" << endl;
            
            // wait for Arming
            this->setArm();
            while(ros::ok() && m_state.armed){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "Armed" << endl;

            // wait for TakeOff
            this->goTakeOff(m_takeoff_alt_m, m_takeoff_spd_mps);
            while(ros::ok() && !(m_state.mode == "AUTO.LOITER")){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "TakeOff" << endl;

            // wait for OFFBOARD Mode
            while(ros::ok() && !this->setOffboard()){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "OFFBOARD Mode" << endl;
            break;

        case 1: // Auto arm(POSCTL) - takeoff
            // wait for POSCTL Mode
            this->setPosctl();
            while(ros::ok() && !(m_state.mode == "POSCTL")){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "POSCTL Mode" << endl;

            // wait for Arming
            this->setArm();
            while(ros::ok() && !m_state.armed){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "Armed" << endl;

            // wait for TakeOff
            this->goTakeOff(m_takeoff_alt_m, m_takeoff_spd_mps);
            while(ros::ok() && !(m_state.mode == "AUTO.LOITER")){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "TakeOff" << endl;
            break;

        case 2: // Auto landing - disarm
            // wait for landing
            this->goLanding();
            while(ros::ok() && !(m_state.mode == "LAND")){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "Landing" << endl;

            // wait for Disarming
            this->setDisarm();
            while(ros::ok() && m_state.armed){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "Disarmed" << endl;
            break;

        case 11: // Force control
            break;

        case 12: // Acceleration control
            break;

        case 13: // Angular velocity control
            break;

        case 14: // Attitude control
            break;

        case 21: // Velocity control
            // x_east_mps, y_north_mps, z_up_mps, heading_CCW_dps
            this->goVelocity(goal_x_, goal_y_, goal_z_, goal_r_);
            break;

        case 22: // Velocity Body control
            // x_forward_mps, y_left_mps, z_up_mps, heading_CCW_dps
            this->goVelocity(goal_x_, goal_y_, goal_z_, goal_r_);
            break;

        case 23: // Position control
            // x_east_m, y_north_m, z_up_m, heading_CCW_deg
            this->goPosition(goal_x_, goal_y_, goal_z_, goal_r_);
            break;

        default: // Emergency - POSCTL Mode
            // wait for POSCTL Mode
            this->setPosctl();
            while(ros::ok() && !(m_state.mode == "POSCTL")){
                ros::spinOnce();
                rate_.sleep();
            }
            cout << "POSCTL Mode" << endl;
            break;
    }
}

void Px4Drone::pubRvizTopics(void) {
    static nav_msgs::Path uav_path;
    geometry_msgs::PoseStamped cur_pos;
    cur_pos.pose = m_odom.pose.pose;
    uav_path.header.frame_id = "map";
    uav_path.poses.push_back(cur_pos);
    m_uav_path_pub.publish(uav_path);

    visualization_msgs::Marker uav_pos;
    uav_pos.type = visualization_msgs::Marker::CUBE;
    uav_pos.header.frame_id = "map";
    uav_pos.scale.x = uav_pos.scale.y = uav_pos.scale.z = 0.3;
    uav_pos.color.r = 0.2;
    uav_pos.color.g = 1.0;
    uav_pos.color.b = 0.2;
    uav_pos.color.a = 1.0;
    uav_pos.pose = m_odom.pose.pose;
    uav_pos.ns = "uav";
    m_uav_pos_pub.publish(uav_pos);
    return;
}

int main(int argc, char **argv)
{
    cout << fixed << setprecision(2);

    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    Px4Drone px4drone = Px4Drone(&nh);
    ros::Rate rate(px4drone.getRosRate());

    // wait for FCU connection
    while(ros::ok() && !px4drone.getState().connected){
        ros::spinOnce();
        rate.sleep();
    }
    cout << "FCU Connected" << endl;

    // set offboard override
    //TODO: move to the new initialization function with other parameters
    while(ros::ok() && !px4drone.setComRCOverride(px4drone.getAutoOverride(), px4drone.getOffboardOverride())){
        ros::spinOnce();
        rate.sleep();
    }
    cout << "Auto & Offboard RC Override" << endl;

    px4drone.doMission(0, -1, -1, -1, -1, rate);

    cout << "main loop start" << endl;
    while(ros::ok())
    {
        // system("clear");
        px4drone.showState();
        px4drone.showOdom();

        px4drone.doMission(23, 1, 2, 3, 45, rate);
        // px4drone.doMission(px4drone.getGoalService(), px4drone.getGoalX(), px4drone.getGoalY(), px4drone.getGoalZ(), px4drone.getGoalR(), rate);

        px4drone.pubRvizTopics();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
