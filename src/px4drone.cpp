#include "px4drone/px4drone.hpp"

using namespace std;

Px4Drone::Px4Drone(ros::NodeHandle *nh_)
  : mCurPos_m(0,0,0),
    mCurVel_mps(0,0,0),
    mCurQ(0,0,0,0),
    mCurRpy_rad(0,0,0),
    mCurRpy_deg(0,0,0),
    mCurRpyRate_dps(0,0,0),
    mAutoOverride(true), // default: true
    mOffboardOverride(true), // default: false
    mGoalService(-1),
    mGoalX(0),
    mGoalY(0),
    mGoalZ(0),
    mGoalR(0),
    mGoalVXY(0),
    mGoalVZUp(0),
    mGoalVZDown(0),
    mRosRate(50.0)
{
  mState_sub = nh_->subscribe ("/mavros/state", 10, &Px4Drone::cbState, this);
  mOdom_sub = nh_->subscribe ("/mavros/local_position/odom", 10, &Px4Drone::cbOdom, this);
  mGoalAction_sub = nh_->subscribe ("/GoalAction", 10, &Px4Drone::cbGoalAction, this);
  mCmdVel_pub = nh_->advertise <geometry_msgs::TwistStamped> ("/mavros/setpoint_velocity/cmd_vel", 10);
  mCmdPos_pub = nh_->advertise <geometry_msgs::PoseStamped> ("/mavros/setpoint_position/local", 10);
  mUavPath_pub = nh_->advertise <nav_msgs::Path> ("/uav_path", 10);
  mUavPos_pub = nh_->advertise <visualization_msgs::Marker> ("/uav_pos", 10);
  mArming_client = nh_->serviceClient <mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  mSetMode_client = nh_->serviceClient <mavros_msgs::SetMode> ("/mavros/set_mode");
  mParamSet_client = nh_->serviceClient <mavros_msgs::ParamSet> ("/mavros/param/set");
  mParamGet_client = nh_->serviceClient <mavros_msgs::ParamGet> ("/mavros/param/get");
}

Px4Drone::~Px4Drone() {

}

void Px4Drone::cbState(const mavros_msgs::State::ConstPtr& msg_) {
  mState = *msg_;
  return;
}

void Px4Drone::cbOdom(const nav_msgs::Odometry::ConstPtr& msg_) {
  mOdom = *msg_;

  mCurPos_m.setValue(mOdom.pose.pose.position.x, 
    mOdom.pose.pose.position.y, 
    mOdom.pose.pose.position.z);

  mCurQ.setValue(mOdom.pose.pose.orientation.x, 
    mOdom.pose.pose.orientation.y, 
    mOdom.pose.pose.orientation.z, 
    mOdom.pose.pose.orientation.w);

  mCurVel_mps.setValue(mOdom.twist.twist.linear.x,
    mOdom.twist.twist.linear.y,
    mOdom.twist.twist.linear.z);

  mCurRpyRate_dps.setValue(mOdom.twist.twist.angular.x,
    mOdom.twist.twist.angular.y,
    mOdom.twist.twist.angular.z);

  tf::Matrix3x3 curMat(mCurQ);
  static double yaw, pitch, roll;
  curMat.getEulerYPR(yaw, pitch, roll);
  mCurRpy_rad.setValue(roll, pitch, yaw);
  mCurRpy_deg = mCurRpy_rad *const_R2D();
  
  static tf::TransformBroadcaster br;
  static tf::Transform transform;
  transform.setOrigin(mCurPos_m);
  transform.setRotation(mCurQ);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "uav"));
  return;
}

void Px4Drone::cbGoalAction(const std_msgs::Float32MultiArray::ConstPtr& msg_) {
  mGoalAction = *msg_;
  mGoalService = (int)mGoalAction.data[0];
  mGoalX = mGoalAction.data[1];
  mGoalY = mGoalAction.data[2];
  mGoalZ = mGoalAction.data[3];
  mGoalR = mGoalAction.data[4];
  mGoalVXY = mGoalAction.data[5];
  mGoalVZUp = mGoalAction.data[6];
  mGoalVZDown = mGoalAction.data[7];
  return;
}

void Px4Drone::showState(void) {
  static const string enum_system_status[9] = \
    {"UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE", 
    "CRITICAL", "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION"};

  cout << endl;
  cout << "\t[Px4Drone] State\t(/mavros/state)" << endl;
  cout << "m_state.connected     = " << (mState.connected ? "OK!" : "Not yet!") << endl;
  cout << "m_state.armed         = " << (mState.armed ? "OK!" : "Not yet!") << endl;
  cout << "m_state.guided        = " << (mState.guided ? "OK!" : "Not yet!") << endl;
  cout << "m_state.manual_input  = " << (mState.manual_input ? "OK!" : "Not yet!") << endl;
  cout << "m_state.mode          = " << mState.mode << endl;
  cout << "m_state.system_status = " << "MAV_STATE_" << enum_system_status[mState.system_status+1] << endl;
  cout << endl;
  return;
}

void Px4Drone::showOdom(void) {
  cout << endl;
  cout << "\t[Px4Drone] Odometry\t(/mavros/local_position/odom)" << endl;
  cout << "m_odom.position_m [x,y,z]        = [" << mCurPos_m.getX() << \
    ",\t" << mCurPos_m.getY() << ",\t" << mCurPos_m.getZ() << "]" << endl;
  cout << "m_odom.velocity_mps [x,y,z]      = [" << mCurVel_mps.getX() << \
    ",\t" << mCurVel_mps.getY() << ",\t" << mCurVel_mps.getZ() << "]" << endl;
  cout << "m_odom.orientation_rad [x,y,z,w] = [" << mCurQ.getX() << \
    ",\t" << mCurQ.getY() << ",\t" << mCurQ.getZ() << ",\t" << mCurQ.getW() << "]" << endl;
  cout << "m_odom.rpy_deg [r,p,y]           = [" << mCurRpy_deg.getX() << \
    ",\t" << mCurRpy_deg.getY() << ",\t" << mCurRpy_deg.getZ() << "]" << endl;
  cout << "m_odom.rpy_rate_deg [r,p,y]      = [" << mCurRpyRate_dps.getX() << \
    ",\t" << mCurRpyRate_dps.getY() << ",\t" << mCurRpyRate_dps.getZ() << "]" << endl;
  cout << endl;
  return;
}

void Px4Drone::showGoalAction(void) {
  cout << endl;
  cout << "\t[Px4Drone] GoalAction\t(/GoalAction)" << endl;
  cout << "goal_service           = [" << mGoalService << endl;
  cout << "goal [x,y,z]           = [" << mGoalX << ",\t" << mGoalY << ",\t" << mGoalZ << "]" << endl;
  cout << "goalV [xy,z_up,z_down] = [" << mGoalVXY << ",\t" << mGoalVZUp << ",\t" << mGoalVZDown << "]" << endl;
  cout << endl;
  return;
}

mavros_msgs::State Px4Drone::getState(void) {
  return mState;
}

nav_msgs::Odometry Px4Drone::getOdom(void) {
  return mOdom;
}

float Px4Drone::getGoalService(void) {
  return mGoalService;
}

float Px4Drone::getGoalX(void) {
  return mGoalX;
}

float Px4Drone::getGoalY(void) {
  return mGoalY;
}

float Px4Drone::getGoalZ(void) {
  return mGoalZ;
}

float Px4Drone::getGoalR(void) {
  return mGoalR;
}

float Px4Drone::getGoalVXY(void) {
  return mGoalVXY;
}

float Px4Drone::getGoalVZUp(void) {
  return mGoalVZUp;
}

float Px4Drone::getGoalVZDown(void) {
  return mGoalVZDown;
}

float Px4Drone::getRosRate(void) {
  return mRosRate;
}

bool Px4Drone::setParamWithAck(string param_id_, int value_) {
  mavros_msgs::ParamSet paramSet;
  mavros_msgs::ParamGet paramGet;

  paramSet.request.param_id = param_id_;
  paramSet.request.value.integer = value_;
  mParamSet_client.call(paramSet);
  paramGet.request.param_id = param_id_;
  mParamGet_client.call(paramGet);

  return paramSet.response.success && paramGet.response.success && (value_ == paramGet.response.value.integer);
}

bool Px4Drone::setParamWithAck(string param_id_, float value_) {
  mavros_msgs::ParamSet paramSet;
  mavros_msgs::ParamGet paramGet;

  paramSet.request.param_id = param_id_;
  paramSet.request.value.real = value_;
  mParamSet_client.call(paramSet);
  paramGet.request.param_id = param_id_;
  mParamGet_client.call(paramGet);

  return paramSet.response.success && paramGet.response.success && (value_ == paramGet.response.value.real);
}

bool Px4Drone::setArm(void) {
  mArmCmd.request.value = true;
  mArming_client.call(mArmCmd);
  return mArmCmd.response.success;
}

bool Px4Drone::setDisarm(void) {
  mArmCmd.request.value = false;
  mArming_client.call(mArmCmd);
  return mArmCmd.response.success;
}

bool Px4Drone::setAcro(void) {
  mSetMode.request.custom_mode = "ACRO";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::setStabilized(void) {
  mSetMode.request.custom_mode = "STABILIZED";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::setAltctl(void) {
  mSetMode.request.custom_mode = "ALTCTL";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::setPosctl(void) {
  mSetMode.request.custom_mode = "POSCTL";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::setOffboard(void) {
  mSetMode.request.custom_mode = "OFFBOARD";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::goTakeOff(float takeoffAlt_m_, float takeoffSpd_mps_) {
  bool ackTakeoffAlt, ackTakeoffSpd;
  ackTakeoffAlt = setParamWithAck("MIS_TAKEOFF_ALT", takeoffAlt_m_);
  ackTakeoffSpd = setParamWithAck("MPC_TKO_SPEED", takeoffSpd_mps_);

  if (ackTakeoffAlt && ackTakeoffSpd) {
    mSetMode.request.custom_mode = "AUTO.TAKEOFF";
    mSetMode_client.call(mSetMode);
    return mSetMode.response.mode_sent;
  }
  else {
    return false;
  }
}

bool Px4Drone::goLanding(float landSpd_mps_) {
  bool ackAndSpd_mps_;
  ackAndSpd_mps_ = setParamWithAck("MPC_LAND_SPEED", landSpd_mps_);

  if (ackAndSpd_mps_) {
    mSetMode.request.custom_mode = "AUTO.LAND";
    mSetMode_client.call(mSetMode);
    return mSetMode.response.mode_sent;
  }
  else {
    return false;
  }
}

void Px4Drone::goVelocity(double xEast_mps_, double yNorth_mps_, double zUp_mps_, double headingCCW_deg_) { //TODO: , double VXY_mps_, double VZUp_mps_, double VZDown_mps_
//TODO: param set MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN
  geometry_msgs::TwistStamped targetVelocity;
  targetVelocity.twist.linear.x = xEast_mps_;
  targetVelocity.twist.linear.y = yNorth_mps_;
  targetVelocity.twist.linear.z = zUp_mps_;
  targetVelocity.twist.angular.z = headingCCW_deg_;
  mCmdVel_pub.publish(targetVelocity);
  return;
}

void Px4Drone::goVelocityBody(double xForward_mps_, double yLeft_mps_, double zUp_mps_, double headingCCW_deg_) { //TODO: , double VXY_mps_, double VZUp_mps_, double VZDown_mps_
//TODO: param set MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN
  geometry_msgs::TwistStamped targetVelocityBody;
  targetVelocityBody.twist.linear.x = xForward_mps_ *cos(mCurRpy_rad.getZ()) - yLeft_mps_ *sin(mCurRpy_rad.getZ());
  targetVelocityBody.twist.linear.y = xForward_mps_ *sin(mCurRpy_rad.getZ()) + yLeft_mps_ *cos(mCurRpy_rad.getZ());
  targetVelocityBody.twist.linear.z = zUp_mps_;
  targetVelocityBody.twist.angular.z = headingCCW_deg_;
  mCmdVel_pub.publish(targetVelocityBody);
  return;
}

void Px4Drone::goPosition(double xEast_m_, double yNorth_m_, double zUp_m_, double headingCCW_deg_, double VXY_mps_, double VZUp_mps_, double VZDown_mps_) {
//TODO: MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN
//TODO: make setAction seperately
  geometry_msgs::PoseStamped targetPosition;
  targetPosition.pose.position.x = xEast_m_;
  targetPosition.pose.position.y = yNorth_m_;
  targetPosition.pose.position.z = zUp_m_;
  quaternionTFToMsg(tf::Quaternion(0, 0, headingCCW_deg_ *const_D2R()), targetPosition.pose.orientation);
  mCmdPos_pub.publish(targetPosition);
  return;
}

bool Px4Drone::doInitialization(ros::Rate rate_) {
  bool ackConnected = false;
  bool ackComRCOverride = false;

  // wait for FCU connection
  while(ros::ok() && !(ackConnected == true)) {
    ackConnected = mState.connected;
    ros::spinOnce();
    rate_.sleep();
  }
  cout << "FCU Connected" << endl;

  // set offboard override
  ackComRCOverride = this->setParamWithAck("COM_RC_OVERRIDE", (mAutoOverride ? 1:0) | (mOffboardOverride ? 2:0));

  if (ackComRCOverride == true) {
    cout << "Auto & Offboard RC Override" << endl;
  }
  
  return ackConnected && ackComRCOverride;
}

void Px4Drone::doMission(int goalService_, double goalX_, double goalY_, double goalZ_, double goalR_, double goalVXY_, double goalVZUp_, double goalVZDown_, ros::Rate rate_) {
//TODO: check if states have changed
//TODO: fix takeoff while takeoff
  switch (goalService_) {
    case -1: // Initial value
      break;

    case 0: // Auto arm(POSCTL) - Takeoff - OFFBOARD
      // wait for POSCTL Mode
      this->setPosctl();
      while(ros::ok() && !(mState.mode == "POSCTL")) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "POSCTL Mode" << endl;
      
      // wait for Arming
      this->setArm();
      while(ros::ok() && mState.armed) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "Armed" << endl;

      // wait for TakeOff
      this->goTakeOff(goalZ_, goalVZUp_);
      while(ros::ok() && !(mState.mode == "AUTO.LOITER")) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "TakeOff" << endl;

      // wait for OFFBOARD Mode
      while(ros::ok() && !this->setOffboard()) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "OFFBOARD Mode" << endl;
      break;

    case 1: // Auto arm(POSCTL) - takeoff
      // wait for POSCTL Mode
      this->setPosctl();
      while(ros::ok() && !(mState.mode == "POSCTL")) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "POSCTL Mode" << endl;

      // wait for Arming
      this->setArm();
      while(ros::ok() && !mState.armed) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "Armed" << endl;

      // wait for TakeOff
      this->goTakeOff(goalZ_, goalVZUp_);
      while(ros::ok() && !(mState.mode == "AUTO.LOITER")) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "TakeOff" << endl;
      break;

    case 2: // Auto landing - disarm
      // wait for Landing
      this->goLanding(goalVZDown_);
      while(ros::ok() && !(mState.mode == "AUTO.LAND")) {
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "Landing" << endl;

      // Auto disarming 2seconds after landing
        // wait for Disarming
        // this->setDisarm();
        // while(ros::ok() && mState.armed) {
        //   ros::spinOnce();
        //   rate_.sleep();
        // }
        // cout << "Disarmed" << endl;
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
      this->goVelocity(goalX_, goalY_, goalZ_, goalR_);
      break;

    case 22: // Velocity Body control
      // x_forward_mps, y_left_mps, z_up_mps, heading_CCW_dps
      this->goVelocity(goalX_, goalY_, goalZ_, goalR_);
      break;

    case 23: // Position control
      // x_east_m, y_north_m, z_up_m, heading_CCW_deg
      this->goPosition(goalX_, goalY_, goalZ_, goalR_, goalVXY_, goalVZUp_, goalVZDown_);
      break;

    default: // Emergency - POSCTL Mode
      // wait for POSCTL Mode
      this->setPosctl();
      while(ros::ok() && !(mState.mode == "POSCTL")){
        ros::spinOnce();
        rate_.sleep();
      }
      cout << "POSCTL Mode" << endl;
      break;
  }
  return;
}

void Px4Drone::pubRvizTopics(void) {
  static nav_msgs::Path uavPath;
  geometry_msgs::PoseStamped curPos;
  curPos.pose = mOdom.pose.pose;
  uavPath.header.frame_id = "map";
  uavPath.poses.push_back(curPos);
  mUavPath_pub.publish(uavPath);

  visualization_msgs::Marker uavPos;
  uavPos.type = visualization_msgs::Marker::CUBE;
  uavPos.header.frame_id = "map";
  uavPos.scale.x = uavPos.scale.y = uavPos.scale.z = 0.3;
  uavPos.color.r = 0.2;
  uavPos.color.g = 1.0;
  uavPos.color.b = 0.2;
  uavPos.color.a = 1.0;
  uavPos.pose = mOdom.pose.pose;
  uavPos.ns = "uav";
  mUavPos_pub.publish(uavPos);
  return;
}
