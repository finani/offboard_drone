
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace ros;

constexpr double const_pi() { return atan(1)*4; }
constexpr double const_R2D() { return 180.0 / const_pi(); }
constexpr double const_D2R() { return const_pi() / 180.0; }

class Px4Drone {
private:
    Subscriber m_state_sub;
    Subscriber m_odom_sub;
    Publisher m_cmd_vel_pub;
    Publisher m_cmd_pos_pub;
    ServiceClient m_arming_client;
    ServiceClient m_set_mode_client;
    ServiceClient m_param_set_client;

    mavros_msgs::State m_state;
    nav_msgs::Odometry m_odom;
    mavros_msgs::CommandBool m_arm_cmd;
    mavros_msgs::SetMode m_set_mode;
    mavros_msgs::ParamSet m_param_set;

    tf::Vector3 m_cur_pos_m;
    tf::Vector3 m_cur_vel_mps;
    tf::Quaternion m_cur_q;
    tf::Vector3 m_cur_rpy_rad;
    tf::Vector3 m_cur_rpy_deg;
    tf::Vector3 m_cur_rpy_rate_dps;
    float m_takeoff_alt_m;
    float m_ros_rate;

public:
    Px4Drone(NodeHandle *nh_);
    ~Px4Drone();
    void cbState(const mavros_msgs::State::ConstPtr& msg_);
    void cbOdom(const nav_msgs::Odometry::ConstPtr& msg_);
    void showState(void);
    void showOdom(void);

    float getRosRate(void);
    mavros_msgs::State getState(void);
    nav_msgs::Odometry getOdom();

    bool setArm(void);
    bool setDisarm(void);
    bool setAcro(void);
    bool setStabilized(void);
    bool setAltctl(void);
    bool setPosctl(void);
    bool setOffboard(void);

    bool goTakeOff(float takeoff_alt_m_);
    bool goLanding(void);
    void goVelocity(double x_east_mps_, double y_north_mps_, double z_up_mps_, double heading_ccw_deg_);
    void goVelocityBody(double x_forward_mps_, double y_left_mps_, double z_up_mps_, double heading_ccw_deg_);
    void goPosition(double x_east_m_, double y_north_m_, double z_up_m_, double heading_ccw_deg_);
};

Px4Drone::Px4Drone(NodeHandle *nh_)
    : m_cur_pos_m(0,0,0),
        m_cur_vel_mps(0,0,0),
        m_cur_q(0,0,0,0),
        m_cur_rpy_rad(0,0,0),
        m_cur_rpy_deg(0,0,0),
        m_cur_rpy_rate_dps(0,0,0),
        m_takeoff_alt_m(3.0),
        m_ros_rate(20.0)
{
    m_state_sub = nh_->subscribe("/mavros/state", 10, &Px4Drone::cbState, this);
    m_odom_sub = nh_->subscribe("/mavros/local_position/odom", 10, &Px4Drone::cbOdom, this);
    m_cmd_vel_pub = nh_->advertise <geometry_msgs::TwistStamped> ("/mavros/setpoint_velocity/cmd_vel", 10);
    m_cmd_pos_pub = nh_->advertise <geometry_msgs::PoseStamped> ("/mavros/setpoint_position/local", 10);
    m_arming_client = nh_->serviceClient <mavros_msgs::CommandBool> ("/mavros/cmd/arming");
    m_set_mode_client = nh_->serviceClient <mavros_msgs::SetMode> ("/mavros/set_mode");
    m_param_set_client = nh_->serviceClient <mavros_msgs::ParamSet> ("/mavros/param/set");
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

float Px4Drone::getRosRate(void) {
    return m_ros_rate;
}

mavros_msgs::State Px4Drone::getState(void) {
    return m_state;
}

nav_msgs::Odometry Px4Drone::getOdom(void) {
    return m_odom;
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

bool Px4Drone::goTakeOff(float takeoff_alt_m_) {
    // m_param_set.request.param_id = "MIS_TAKEOFF_ALT";
    // m_param_set.request.value.real = takeoff_alt_m_;
    // m_param_set_client.call(m_param_set);
    // if (m_param_set.response.success) {
        m_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        m_set_mode_client.call(m_set_mode);
        return m_set_mode.response.mode_sent;
    // }
    // else {
    //     return false;
    // }
}

bool Px4Drone::goLanding(void) {
    m_set_mode.request.custom_mode = "AUTO.LAND";
    m_set_mode_client.call(m_set_mode);
    return m_set_mode.response.mode_sent;
}

void Px4Drone::goVelocity(double x_east_mps_, double y_north_mps_, double z_up_mps_, double heading_ccw_dps_) {
    geometry_msgs::TwistStamped targetVelocity;
    targetVelocity.twist.linear.x = x_east_mps_;
    targetVelocity.twist.linear.y = y_north_mps_;
    targetVelocity.twist.linear.z = z_up_mps_;
    targetVelocity.twist.angular.z = heading_ccw_dps_;
    m_cmd_vel_pub.publish(targetVelocity);
    return;
}

void Px4Drone::goVelocityBody(double x_forward_mps_, double y_left_mps_, double z_up_mps_, double heading_ccw_dps_) {
    geometry_msgs::TwistStamped targetVelocityBody;
    targetVelocityBody.twist.linear.x = x_forward_mps_ *cos(m_cur_rpy_rad.getZ()) - y_left_mps_ *sin(m_cur_rpy_rad.getZ());
    targetVelocityBody.twist.linear.y = x_forward_mps_ *sin(m_cur_rpy_rad.getZ()) + y_left_mps_ *cos(m_cur_rpy_rad.getZ());
    targetVelocityBody.twist.linear.z = z_up_mps_;
    targetVelocityBody.twist.angular.z = heading_ccw_dps_;
    m_cmd_vel_pub.publish(targetVelocityBody);
    return;
}

void Px4Drone::goPosition(double x_east_m_, double y_north_m_, double z_up_m_, double heading_ccw_deg_) {
    geometry_msgs::PoseStamped targetPosition;
    targetPosition.pose.position.x = x_east_m_;
    targetPosition.pose.position.y = y_north_m_;
    targetPosition.pose.position.z = z_up_m_;
    quaternionTFToMsg(tf::Quaternion(0, 0, heading_ccw_deg_ *const_D2R()), targetPosition.pose.orientation);
    m_cmd_pos_pub.publish(targetPosition);
    return;
}

int main(int argc, char **argv)
{
    cout << fixed << setprecision(2);

    init(argc, argv, "offboard_node");
    NodeHandle nh;
    Px4Drone px4drone = Px4Drone(&nh);
    Rate rate(px4drone.getRosRate());

    // wait for FCU connection
    while(ros::ok() && !px4drone.getState().connected){
        ros::spinOnce();
        rate.sleep();
    }
    cout << "FCU Connected" << endl;

    // wait for Arming
    px4drone.setArm();
    while(ros::ok() && !px4drone.getState().armed){
        ros::spinOnce();
        rate.sleep();
    }
    cout << "Armed" << endl;

    // wait for TakeOff
    px4drone.goTakeOff(3.0);
    while(ros::ok() && !(px4drone.getState().mode == "AUTO.LOITER")){
        ros::spinOnce();
        rate.sleep();
    }
    cout << "TakeOff" << endl;

    cout << "main loop start" << endl;
    while(ros::ok())
    {
        // system("clear");
        px4drone.showState();
        px4drone.showOdom();

        if (px4drone.setOffboard()) {
            // px4drone.goVelocity(1, 0, 0, 5);
            px4drone.goVelocityBody(1, 0, 0, 1);
            // px4drone.goPosition(1, 2, 3, 45);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
