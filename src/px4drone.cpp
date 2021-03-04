#include "px4drone/px4drone.hpp"

Px4Drone::Px4Drone(ros::NodeHandle *nh_)
  : mSystemStatusVector({"UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE", \
      "CRITICAL", "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION"}),
    mCurPos_m(0,0,0),
    mCurVel_mps(0,0,0),
    mCurQ(0,0,0,0),
    mCurRpy_rad(0,0,0),
    mCurRpy_deg(0,0,0),
    mCurRpyRate_rps(0,0,0),
    mAutoOverride(true),
    mOffboardOverride(false),
    mGoalService(-1),
    mGoalX(0),
    mGoalY(0),
    mGoalZ(0),
    mGoalR(0),
    mRosRate(20.0),
    mShowState(false),
    mShowOdom(false),
    mShowGoalAction(false),
    mPubRvizTopics(false),
    mEnableAutoTakeoff(false)
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

  nh_->getParam("/Px4_Drone/ROS/ROS_RATE", mRosRate);
  nh_->getParam("/Px4_Drone/USER/SHOW_STATE", mShowState);
  nh_->getParam("/Px4_Drone/USER/SHOW_ODOM", mShowOdom);
  nh_->getParam("/Px4_Drone/USER/SHOW_GOAL_ACTION", mShowGoalAction);
  nh_->getParam("/Px4_Drone/USER/PUB_RVIZ_TOPICS", mPubRvizTopics);
  nh_->getParam("/Px4_Drone/USER/ENABLE_AUTO_TAKEOFF", mEnableAutoTakeoff);
  nh_->getParam("/Px4_Drone/MAVROS/REVERSE_THRUST", mReverseThrust);
  nh_->getParam("/Px4_Drone/Commander/COM_ARM_IMU_ACC", mArmAccErr_mpss);
  nh_->getParam("/Px4_Drone/Commander/COM_ARM_IMU_GYR", mArmGyroErr_rps);
  nh_->getParam("/Px4_Drone/Commander/COM_ARM_MAG_ANG", mArmMagErr_deg);
  nh_->getParam("/Px4_Drone/Commander/COM_RC_OVERRIDE_AUTO", mAutoOverride);
  nh_->getParam("/Px4_Drone/Commander/COM_RC_OVERRIDE_OFFBOARD", mOffboardOverride);
  nh_->getParam("/Px4_Drone/Mission/MIS_TAKEOFF_ALT", mTakeoffAlt_m);
  nh_->getParam("/Px4_Drone/Geofence/GF_ACTION", mGeofenceAction);
  nh_->getParam("/Px4_Drone/Geofence/GF_MAX_HOR_DIST", mGeofenceXY_m);
  nh_->getParam("/Px4_Drone/Geofence/GF_MAX_VER_DIST", mGeofenceZ_m);
  nh_->getParam("/Px4_Drone/Multicopter_Position_Control/MPC_TKO_SPEED", mTakeoffSpd_mps);
  nh_->getParam("/Px4_Drone/Multicopter_Position_Control/MPC_LAND_SPEED", mLandSpd_mps);
  nh_->getParam("/Px4_Drone/Multicopter_Position_Control/MPC_XY_VEL_MAX", mXYMaxSpd_mps);
  nh_->getParam("/Px4_Drone/Multicopter_Position_Control/MPC_Z_VEL_MAX_UP", mZUpMaxSpd_mps);
  nh_->getParam("/Px4_Drone/Multicopter_Position_Control/MPC_Z_VEL_MAX_DN", mZDownMaxSpd_mps);

  //TODO: check if it is working (reverse_throttle)
  nh_->setParam("/mavros/setpoint_attitude/reverse_thrust", mReverseThrust);
}

void Px4Drone::cbState(const mavros_msgs::State::ConstPtr& msg_) {
  ROS_INFO_ONCE_NAMED("px4drone", "[cbState] cbState start");

  mState = *msg_;
  return;
}

void Px4Drone::cbOdom(const nav_msgs::Odometry::ConstPtr& msg_) {
  ROS_INFO_ONCE_NAMED("px4drone", "[cbOdom] cbOdom start");

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

  mCurRpyRate_rps.setValue(mOdom.twist.twist.angular.x,
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
  ROS_INFO_ONCE_NAMED("px4drone", "[cbGoalAction] cbGoalAction start");

  mGoalAction = *msg_;
  mGoalService = static_cast<int>(mGoalAction.data[0]);
  mGoalX = mGoalAction.data[1];
  mGoalY = mGoalAction.data[2];
  mGoalZ = mGoalAction.data[3];
  mGoalR = mGoalAction.data[4];
  return;
}

void Px4Drone::showState(void) const {
  if (mShowState == true) {
    ROS_INFO_ONCE_NAMED("px4drone", "[showState] showState Turn On");

    std::cout << std::endl;
    std::cout << "\t[Px4Drone] State\t(/mavros/state)" << std::endl;
    std::cout << "m_state.connected     = " << (mState.connected ? "OK!" : "Not yet!") << std::endl;
    std::cout << "m_state.armed         = " << (mState.armed ? "OK!" : "Not yet!") << std::endl;
    std::cout << "m_state.guided        = " << (mState.guided ? "OK!" : "Not yet!") << std::endl;
    std::cout << "m_state.manual_input  = " << (mState.manual_input ? "OK!" : "Not yet!") << std::endl;
    std::cout << "m_state.mode          = " << mState.mode << std::endl;
    std::cout << "m_state.system_status = " << "MAV_STATE_" << mSystemStatusVector[static_cast<int>(mState.system_status)] << std::endl;
    std::cout << std::endl;
  }
  else {
    ROS_INFO_ONCE_NAMED("px4drone", "[showState] showState Turn Off");
  }
  return;
}

void Px4Drone::showOdom(void) const {
  if (mShowOdom == true) {
    ROS_INFO_ONCE_NAMED("px4drone", "[showOdom] showOdom Turn On");

    std::cout << std::endl;
    std::cout << "\t[Px4Drone] Odometry\t(/mavros/local_position/odom)" << std::endl;
    std::cout << "m_odom.position_m [x,y,z]      = [" << mCurPos_m.getX() << \
      ", \t" << mCurPos_m.getY() << ", \t" << mCurPos_m.getZ() << "]" << std::endl;
    std::cout << "m_odom.velocity_mps [x,y,z]    = [" << mCurVel_mps.getX() << \
      ", \t" << mCurVel_mps.getY() << ", \t" << mCurVel_mps.getZ() << "]" << std::endl;
    std::cout << "m_odom.orientation_q [x,y,z,w] = [" << mCurQ.getX() << \
      ", \t" << mCurQ.getY() << ", \t" << mCurQ.getZ() << ", \t" << mCurQ.getW() << "]" << std::endl;
    std::cout << "m_odom.rpy_deg [r,p,y]         = [" << mCurRpy_deg.getX() << \
      ", \t" << mCurRpy_deg.getY() << ", \t" << mCurRpy_deg.getZ() << "]" << std::endl;
    std::cout << "m_odom.rpy_rate_dps [r,p,y]    = [" << mCurRpyRate_rps.getX() *const_R2D() << \
      ", \t" << mCurRpyRate_rps.getY() *const_R2D() << ", \t" << mCurRpyRate_rps.getZ() *const_R2D() << "]" << std::endl;
    std::cout << std::endl;
  }
  else {
    ROS_INFO_ONCE_NAMED("px4drone", "[showOdom] showOdom Turn Off");
  }
  return;
}

void Px4Drone::showGoalAction(void) const {
  if (mShowGoalAction == true) {
    ROS_INFO_ONCE_NAMED("px4drone", "[showGoalAction] showGoalAction Turn On");

    std::cout << std::endl;
    std::cout << "\t[Px4Drone] GoalAction\t(/GoalAction)" << std::endl;
    std::cout << "goal_service   = [" << mGoalService << std::endl;
    std::cout << "goal [x,y,z,r] = [" << mGoalX << ", \t" << mGoalY << ", \t" << \
      mGoalZ << ", \t" << mGoalR << "]" << std::endl;
    std::cout << std::endl;
  }
  else {
    ROS_INFO_ONCE_NAMED("px4drone", "[showGoalAction] showGoalAction Turn Off");
  }
  return;
}

mavros_msgs::State Px4Drone::getState(void) const {
  return mState;
}

nav_msgs::Odometry Px4Drone::getOdom(void) const {
  return mOdom;
}

float Px4Drone::getGoalService(void) const {
  return mGoalService;
}

float Px4Drone::getGoalX(void) const {
  return mGoalX;
}

float Px4Drone::getGoalY(void) const {
  return mGoalY;
}

float Px4Drone::getGoalZ(void) const {
  return mGoalZ;
}

float Px4Drone::getGoalR(void) const {
  return mGoalR;
}

float Px4Drone::getRosRate(void) const {
  return mRosRate;
}

bool Px4Drone::setParamWithAck(std::string param_id_, int value_) {
  bool ackSetParamWithAck = false;
  mavros_msgs::ParamSet paramSet;
  mavros_msgs::ParamGet paramGet;

  paramSet.request.param_id = param_id_;
  paramSet.request.value.integer = value_;
  mParamSet_client.call(paramSet);
  paramGet.request.param_id = param_id_;
  mParamGet_client.call(paramGet);

  ackSetParamWithAck = (paramSet.response.success == true) && \
    (paramGet.response.success == true) && \
    (value_ == paramGet.response.value.integer);

  ROS_ASSERT_CMD(ackSetParamWithAck == true, \
    ROS_FATAL_STREAM_NAMED("px4drone", "[setParamWithAck] setParam " << param_id_ << " Fail"); \
    ROS_BREAK());
  ROS_INFO_STREAM_NAMED("px4drone", "[setParamWithAck] setParam " << param_id_ << " Sent");

  return ackSetParamWithAck;
}

bool Px4Drone::setParamWithAck(std::string param_id_, float value_) {
  bool ackSetParamWithAck = false;
  mavros_msgs::ParamSet paramSet;
  mavros_msgs::ParamGet paramGet;

  paramSet.request.param_id = param_id_;
  paramSet.request.value.real = value_;
  mParamSet_client.call(paramSet);
  paramGet.request.param_id = param_id_;
  mParamGet_client.call(paramGet);

  ackSetParamWithAck = (paramSet.response.success == true) && \
    (paramGet.response.success == true) && \
    (value_ == paramGet.response.value.real);

  ROS_ASSERT_CMD(ackSetParamWithAck == true, \
    ROS_FATAL_STREAM_NAMED("px4drone", "[setParamWithAck] setParam " << param_id_ << " Fail"); \
    ROS_BREAK());
  ROS_INFO_STREAM_NAMED("px4drone", "[setParamWithAck] setParam " << param_id_ << " Sent");

  return ackSetParamWithAck;
}

bool Px4Drone::setArm(void) {
  bool ackSetArm = false;

  mArmCmd.request.value = true;
  mArming_client.call(mArmCmd);
  ackSetArm = mArmCmd.response.success;

  ROS_ASSERT_CMD(ackSetArm == true, \
    this->setDisarm(); \
    ROS_FATAL_NAMED("px4drone", "[setArm] Arm Failed"); \
    ROS_INFO_NAMED("px4drone", "[setArm] setDisarm start"); \
    ROS_BREAK());
  ROS_INFO_THROTTLE_NAMED(1.0, "px4drone", "[setArm] Arm Sent");

  return ackSetArm;
}

bool Px4Drone::setDisarm(void) {
  bool ackSetDisarm = false;

  mArmCmd.request.value = false;
  mArming_client.call(mArmCmd);
  ackSetDisarm = mArmCmd.response.success;

  ROS_ASSERT_CMD(ackSetDisarm == true, \
    ROS_FATAL_NAMED("px4drone", "[setDisarm] Disarm Failed"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[setDisarm] Disarm Sent");

  return ackSetDisarm;
}

bool Px4Drone::setAcro(void) {
  bool ackSetAcro = false;

  mSetMode.request.custom_mode = "ACRO";
  mSetMode_client.call(mSetMode);
  ackSetAcro = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackSetAcro == true, \
    ROS_FATAL_NAMED("px4drone", "[setAcro] ACRO Mode Set Failed"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[setAcro] ACRO Mode Sent");

  return ackSetAcro;
}

bool Px4Drone::setStabilized(void) {
  bool ackSetStabilized = false;

  mSetMode.request.custom_mode = "STABILIZED";
  mSetMode_client.call(mSetMode);
  ackSetStabilized = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackSetStabilized == true, \
    ROS_FATAL_NAMED("px4drone", "[setStabilized] STABILIZED Mode Set Failed"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[setStabilized] STABILIZED Mode Sent");

  return ackSetStabilized;
}

bool Px4Drone::setAltctl(void) {
  bool ackSetAltctl = false;

  mSetMode.request.custom_mode = "ALTCTL";
  mSetMode_client.call(mSetMode);
  ackSetAltctl = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackSetAltctl == true, \
    this->setStabilized(); \
    ROS_FATAL_NAMED("px4drone", "[setAltctl] ALTCTL Mode Set Failed"); \
    ROS_INFO_NAMED("px4drone", "[setAltctl] setStabilized start"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[setAltctl] ALTCTL Mode Sent");

  return ackSetAltctl;
}

bool Px4Drone::setPosctl(void) {
  bool ackSetPosctl = false;

  mSetMode.request.custom_mode = "POSCTL";
  mSetMode_client.call(mSetMode);
  ackSetPosctl = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackSetPosctl == true, \
    this->setAltctl(); \
    ROS_FATAL_NAMED("px4drone", "[setPosctl] POSCTL Mode Set Failed"); \
    ROS_INFO_NAMED("px4drone", "[setPosctl] setAltctl start"); \
    ROS_BREAK());
  ROS_INFO_THROTTLE_NAMED(1.0, "px4drone", "[setPosctl] POSCTL Mode Sent");

  return ackSetPosctl;
}

bool Px4Drone::setOffboard(void) {
  bool ackSetOffboard = false;

  mSetMode.request.custom_mode = "OFFBOARD";
  mSetMode_client.call(mSetMode);
  ackSetOffboard = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackSetOffboard == true, \
    this->goHold(); \
    ROS_FATAL_NAMED("px4drone", "[setOffboard] OFFBOARD Mode Set Failed"); \
    ROS_INFO_NAMED("px4drone", "[setOffboard] goHold start"); \
    ROS_BREAK());
  ROS_INFO_THROTTLE_NAMED(1.0, "px4drone", "[setOffboard] OFFBOARD Mode Sent");

  return ackSetOffboard;
}

bool Px4Drone::goTakeOff(void) {
  bool ackGoTakeOff = false;

  mSetMode.request.custom_mode = "AUTO.TAKEOFF";
  mSetMode_client.call(mSetMode);
  ackGoTakeOff = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackGoTakeOff == true, \
    this->goLanding(); \
    ROS_FATAL_NAMED("px4drone", "[goTakeOff] Take Off Failed"); \
    ROS_INFO_NAMED("px4drone", "[goTakeOff] goLanding start"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[goTakeOff] TakeOff Sent");

  return ackGoTakeOff;
}

bool Px4Drone::goLanding(void) {
  bool ackGoLanding = false;

  mSetMode.request.custom_mode = "AUTO.LAND";
  mSetMode_client.call(mSetMode);
  ackGoLanding = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackGoLanding == true, \
    ROS_FATAL_NAMED("px4drone", "[goLanding] Landing Failed"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[goLanding] Landing Sent");

  return ackGoLanding;
}

bool Px4Drone::goReturn(void) {
  bool ackSetReturn = false;

  mSetMode.request.custom_mode = "AUTO.RTL";
  mSetMode_client.call(mSetMode);
  ackSetReturn = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackSetReturn == true, \
    this->goHold(); \
    ROS_FATAL_NAMED("px4drone", "[ackSetReturn] RTL Mode Set Failed"); \
    ROS_INFO_NAMED("px4drone", "[ackSetReturn] goHold start"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[ackSetReturn] RTL Mode Sent");

  return ackSetReturn;
}

bool Px4Drone::goHold(void) {
  bool ackGoHold = false;

  mSetMode.request.custom_mode = "AUTO.LOITER";
  mSetMode_client.call(mSetMode);
  ackGoHold = mSetMode.response.mode_sent;

  ROS_ASSERT_CMD(ackGoHold == true, \
    this->goLanding(); \
    ROS_FATAL_NAMED("px4drone", "[goHold] Hold Mode Set Failed"); \
    ROS_INFO_NAMED("px4drone", "[goHold] goLanding start"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[goHold] Hold Mode Sent");

  return ackGoHold;
}

void Px4Drone::goForce(double xForward_N_, double y_Left_N_, double z_Up_N_) { // , goalR_ deg? dps? rps?
  return;
}

void Px4Drone::goAccel(double xForward_mpss_, double y_Left_mpss_, double z_Up_mpss_) { // , goalR_ deg? dps? rps?
  return;
}

void Px4Drone::goAngularVelocity(void) {
  return;
}

void Px4Drone::goAttitude(void) {
  return;
}

void Px4Drone::goVelocity(double xEast_mps_, double yNorth_mps_, double zUp_mps_, double headingCCW_dps_) {
  geometry_msgs::TwistStamped targetVelocity;
  targetVelocity.twist.linear.x = xEast_mps_;
  targetVelocity.twist.linear.y = yNorth_mps_;
  targetVelocity.twist.linear.z = zUp_mps_;
  targetVelocity.twist.angular.z = headingCCW_dps_ *const_D2R();
  mCmdVel_pub.publish(targetVelocity);
  return;
}

void Px4Drone::goVelocityBody(double xForward_mps_, double yLeft_mps_, double zUp_mps_, double headingCCW_dps_) {
  geometry_msgs::TwistStamped targetVelocityBody;
  targetVelocityBody.twist.linear.x = \
    xForward_mps_ *cos(mCurRpy_rad.getZ()) - yLeft_mps_ *sin(mCurRpy_rad.getZ());
  targetVelocityBody.twist.linear.y = \
    xForward_mps_ *sin(mCurRpy_rad.getZ()) + yLeft_mps_ *cos(mCurRpy_rad.getZ());
  targetVelocityBody.twist.linear.z = zUp_mps_;
  targetVelocityBody.twist.angular.z = headingCCW_dps_ *const_D2R();
  mCmdVel_pub.publish(targetVelocityBody);
  return;
}

void Px4Drone::goPosition(double xEast_m_, double yNorth_m_, double zUp_m_, double headingCCW_deg_) {
  geometry_msgs::PoseStamped targetPosition;
  targetPosition.pose.position.x = xEast_m_;
  targetPosition.pose.position.y = yNorth_m_;
  targetPosition.pose.position.z = zUp_m_;
  quaternionTFToMsg(tf::Quaternion(0, 0, headingCCW_deg_ *const_D2R()), \
    targetPosition.pose.orientation);
  mCmdPos_pub.publish(targetPosition);
  return;
}

bool Px4Drone::doInitialization(ros::Rate rate_) {
  ROS_INFO_NAMED("px4drone", "[doInitialization] Initialization start");

  bool ackConnected = false;
  bool AckParamSet = true;

  // wait for FCU connection
  while(ros::ok() && !(ackConnected == true)) {
    ROS_INFO_NAMED("px4drone", "[doInitialization] FCU Try to Connect Failed");
    ackConnected = mState.connected;
    ros::spinOnce();
    rate_.sleep();
  }
  ROS_INFO_NAMED("px4drone", "[doInitialization] FCU Connected");

  // set px4 parameters
  AckParamSet &= this->setParamWithAck("COM_ARM_IMU_ACC", mArmAccErr_mpss);
  AckParamSet &= this->setParamWithAck("COM_ARM_IMU_GYR", mArmGyroErr_rps);
  AckParamSet &= this->setParamWithAck("COM_ARM_MAG_ANG", static_cast<int>(mArmMagErr_deg));
  AckParamSet &= this->setParamWithAck("COM_RC_OVERRIDE", \
    static_cast<int>((mAutoOverride ? 1:0) | (mOffboardOverride ? 2:0)));
  AckParamSet &= this->setParamWithAck("GF_ACTION", static_cast<int>(mGeofenceAction));
  AckParamSet &= this->setParamWithAck("GF_MAX_HOR_DIST", mGeofenceXY_m);
  AckParamSet &= this->setParamWithAck("GF_MAX_VER_DIST", mGeofenceZ_m);
  AckParamSet &= this->setParamWithAck("MIS_TAKEOFF_ALT", mTakeoffAlt_m);
  AckParamSet &= this->setParamWithAck("MPC_TKO_SPEED", mTakeoffSpd_mps);
  AckParamSet &= this->setParamWithAck("MPC_LAND_SPEED", mLandSpd_mps);
  AckParamSet &= this->setParamWithAck("MPC_XY_VEL_MAX", mXYMaxSpd_mps);
  AckParamSet &= this->setParamWithAck("MPC_Z_VEL_MAX_UP", mZUpMaxSpd_mps);
  AckParamSet &= this->setParamWithAck("MPC_Z_VEL_MAX_DN", mZDownMaxSpd_mps);

  ROS_ASSERT_CMD(AckParamSet == true, \
    ROS_FATAL_NAMED("px4drone", "[doInitialization] Px4 Params Set Failed"); \
    ROS_BREAK());
  ROS_INFO_NAMED("px4drone", "[doInitialization] Px4 Params Set");

  return ackConnected && AckParamSet;
}

void Px4Drone::doMission(int goalService_, double goalX_, double goalY_, double goalZ_, double goalR_) {
  ROS_INFO_ONCE_NAMED("px4drone", "[doMission] doMission start");

  static bool ackGoTakeoff = false;
  static bool ackGoLanding = false;

  if (mSystemStatusVector[static_cast<int>(mState.system_status)] == "STANDBY") {
    ackGoTakeoff = false;
    ackGoLanding = false;
  }

  switch (goalService_) {
    case -1: // Initial value
      break;

    case 0: // Auto Takeoff Reset
      // reset Auto Takeoff
      mEnableAutoTakeoff = true;
      ros::param::set("/Px4_Drone/USER/ENABLE_AUTO_TAKEOFF", mEnableAutoTakeoff);
      break;

    case 1: // Auto arm (POSCTL) - Takeoff
      if (mEnableAutoTakeoff == true) {
        // set POSCTL Mode
        if (mSystemStatusVector[static_cast<int>(mState.system_status)] == "STANDBY") {
          this->setPosctl();
        }

        // set Arming
        if (mState.armed == false) {
          this->setArm();
        }

        // go TakeOff
        if ((ackGoTakeoff == false) && (mState.mode == "POSCTL") && (mState.armed == true)) {
          ackGoTakeoff = this->goTakeOff();
        }

        // set Auto Takeoff Disable
        if (mState.mode == "AUTO.TAKEOFF") {
          mEnableAutoTakeoff = false;
          ros::param::set("/Px4_Drone/USER/ENABLE_AUTO_TAKEOFF", mEnableAutoTakeoff);
        }
      }
      break;

    case 2: // Auto landing (Disarm)
      // go Landing
      if ((ackGoLanding == false) && \
          (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        ackGoLanding = this->goLanding();
      }
      // Auto disarming 2 seconds after landing
      break;

    case 11: // Force control
      // go Force
      this->goForce(goalX_, goalY_, goalZ_); // , goalR_
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    case 12: // Acceleration control
      // go Accel
      this->goAccel(goalX_, goalY_, goalZ_); // , goalR_
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    case 13: // Angular velocity control
      // go AngularVelocity
//      this->goAngularVelocity(goalX_, goalY_, goalZ_, goalR_, goalT_);
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    case 14: // Attitude control
      // go Attitude
//      this->goAngularVelocity(goalX_, goalY_, goalZ_, goalW_, goalT_);
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    case 21: // Velocity control
      // x_east_mps, y_north_mps, z_up_mps, heading_CCW_dps
      this->goVelocity(goalX_, goalY_, goalZ_, goalR_);
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    case 22: // Velocity Body control
      // x_forward_mps, y_left_mps, z_up_mps, heading_CCW_dps
      this->goVelocityBody(goalX_, goalY_, goalZ_, goalR_);
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    case 23: // Position control
      // x_east_m, y_north_m, z_up_m, heading_CCW_deg
      this->goPosition(goalX_, goalY_, goalZ_, goalR_);
      // set OFFBOARD Mode
      if (!(mState.mode == "OFFBOARD") && \
        (mSystemStatusVector[static_cast<int>(mState.system_status)] == "ACTIVE")) {
        this->setOffboard();
      }
      break;

    default: // Emergency - HOLD Mode
      // HOLD Mode
      this->goHold();
      break;
  }
  return;
}

void Px4Drone::pubRvizTopics(void) const {
  if (mPubRvizTopics == true) {
    ROS_INFO_ONCE_NAMED("px4drone", "[pubRvizTopics] pubRvizTopics Turn On");

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
  }
  else {
    ROS_INFO_ONCE_NAMED("px4drone", "[pubRvizTopics] pubRvizTopics Turn Off");
  }
  return;
}
