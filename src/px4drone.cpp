#include "px4drone/px4drone.hpp"

Px4Drone::Px4Drone(ros::NodeHandle *nh_)
  : mCurPos_m(0,0,0),
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
}

void Px4Drone::cbState(const mavros_msgs::State::ConstPtr& msg_) {
  ROS_INFO_ONCE_NAMED("px4drone", "<cbState> cbState start");

  mState = *msg_;
  return;
}

void Px4Drone::cbOdom(const nav_msgs::Odometry::ConstPtr& msg_) {
  ROS_INFO_ONCE_NAMED("px4drone", "<cbOdom> cbOdom start");

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
  ROS_INFO_ONCE_NAMED("px4drone", "<cbGoalAction> cbGoalAction start");

  mGoalAction = *msg_;
  mGoalService = static_cast<int>(mGoalAction.data[0]);
  mGoalX = mGoalAction.data[1];
  mGoalY = mGoalAction.data[2];
  mGoalZ = mGoalAction.data[3];
  mGoalR = mGoalAction.data[4];
  return;
}

void Px4Drone::showState(void) {
  ROS_INFO_ONCE_NAMED("px4drone", "<showState> showState start");

  static const std::string enum_system_status[9] = \
    {"UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE", 
    "CRITICAL", "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION"};

  std::cout << std::endl;
  std::cout << "\t[Px4Drone] State\t(/mavros/state)" << std::endl;
  std::cout << "m_state.connected     = " << (mState.connected ? "OK!" : "Not yet!") << std::endl;
  std::cout << "m_state.armed         = " << (mState.armed ? "OK!" : "Not yet!") << std::endl;
  std::cout << "m_state.guided        = " << (mState.guided ? "OK!" : "Not yet!") << std::endl;
  std::cout << "m_state.manual_input  = " << (mState.manual_input ? "OK!" : "Not yet!") << std::endl;
  std::cout << "m_state.mode          = " << mState.mode << std::endl;
  std::cout << "m_state.system_status = " << "MAV_STATE_" << enum_system_status[mState.system_status+1] << std::endl;
  std::cout << std::endl;
  return;
}

void Px4Drone::showOdom(void) {
  ROS_INFO_ONCE_NAMED("px4drone", "<showOdom> showOdom start");

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
  std::cout << "m_odom.rpy_rate_rps [r,p,y]    = [" << mCurRpyRate_rps.getX() << \
    ", \t" << mCurRpyRate_rps.getY() << ", \t" << mCurRpyRate_rps.getZ() << "]" << std::endl;
  std::cout << std::endl;
  return;
}

void Px4Drone::showGoalAction(void) {
  ROS_INFO_ONCE_NAMED("px4drone", "<showGoalAction> showGoalAction start");

  std::cout << std::endl;
  std::cout << "\t[Px4Drone] GoalAction\t(/GoalAction)" << std::endl;
  std::cout << "goal_service    = [" << mGoalService << std::endl;
  std::cout << "goal [x,y,z,r] = [" << mGoalX << ", \t" << mGoalY << ", \t" << \
    mGoalZ << ", \t" << mGoalR << "]" << std::endl;
  std::cout << std::endl;
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

float Px4Drone::getRosRate(void) {
  return mRosRate;
}

//TODO: use Template including int, float
bool Px4Drone::setParamWithAck(std::string param_id_, int value_) {
  mavros_msgs::ParamSet paramSet;
  mavros_msgs::ParamGet paramGet;

  paramSet.request.param_id = param_id_;
  paramSet.request.value.integer = value_;
  mParamSet_client.call(paramSet);
  paramGet.request.param_id = param_id_;
  mParamGet_client.call(paramGet);

  return paramSet.response.success && paramGet.response.success && \
    (value_ == paramGet.response.value.integer);
}

bool Px4Drone::setParamWithAck(std::string param_id_, float value_) {
  mavros_msgs::ParamSet paramSet;
  mavros_msgs::ParamGet paramGet;

  paramSet.request.param_id = param_id_;
  paramSet.request.value.real = value_;
  mParamSet_client.call(paramSet);
  paramGet.request.param_id = param_id_;
  mParamGet_client.call(paramGet);

  return paramSet.response.success && paramGet.response.success && \
    (value_ == paramGet.response.value.real);
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

bool Px4Drone::goTakeOff(void) {
  mSetMode.request.custom_mode = "AUTO.TAKEOFF";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::goLanding(void) {
  mSetMode.request.custom_mode = "AUTO.LAND";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::goReturn(void) {
  mSetMode.request.custom_mode = "AUTO.RTL";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

bool Px4Drone::goHold(void) {
  mSetMode.request.custom_mode = "AUTO.LOITER";
  mSetMode_client.call(mSetMode);
  return mSetMode.response.mode_sent;
}

void Px4Drone::goForce(void) {
  
}

void Px4Drone::goAccel(void) {
  
}

void Px4Drone::goAngularVelocity(void) {
  
}

void Px4Drone::goAttitude(void) {
  
}

void Px4Drone::goVelocity(double xEast_mps_, double yNorth_mps_, double zUp_mps_, double headingCCW_rps_) {
  geometry_msgs::TwistStamped targetVelocity;
  targetVelocity.twist.linear.x = xEast_mps_;
  targetVelocity.twist.linear.y = yNorth_mps_;
  targetVelocity.twist.linear.z = zUp_mps_;
  targetVelocity.twist.angular.z = headingCCW_rps_;
  mCmdVel_pub.publish(targetVelocity);
  return;
}

void Px4Drone::goVelocityBody(double xForward_mps_, double yLeft_mps_, double zUp_mps_, double headingCCW_rps_) {
  geometry_msgs::TwistStamped targetVelocityBody;
  targetVelocityBody.twist.linear.x = \
    xForward_mps_ *cos(mCurRpy_rad.getZ()) - yLeft_mps_ *sin(mCurRpy_rad.getZ());
  targetVelocityBody.twist.linear.y = \
    xForward_mps_ *sin(mCurRpy_rad.getZ()) + yLeft_mps_ *cos(mCurRpy_rad.getZ());
  targetVelocityBody.twist.linear.z = zUp_mps_;
  targetVelocityBody.twist.angular.z = headingCCW_rps_;
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
  ROS_INFO_NAMED("px4drone", "<doInitialization> doInitialization start");

  bool ackConnected = false;
  bool ackParamSet = false;
  bool ackArmAccErr, ackArmGyroErr, ackArmMagErr, ackComRCOverride, \
    ackGeofenceAction, ackGeofenceXY, ackGeofenceZ, \
    ackTakeoffAlt, ackTakeoffSpd, ackLandSpd, \
    ackXYMaxSpd, ackZUpMaxSpd, ackZDownMaxSpd;

  // wait for FCU connection
  while(ros::ok() && !(ackConnected == true)) {
    ackConnected = mState.connected;
    ros::spinOnce();
    rate_.sleep();
  }
  ROS_INFO_NAMED("px4drone", "<doInitialization> FCU Connected");

  // set px4 parameters
  ackArmAccErr = this->setParamWithAck("COM_ARM_IMU_ACC", mArmAccErr_mpss);
  ackArmGyroErr = this->setParamWithAck("COM_ARM_IMU_GYR", mArmGyroErr_rps);
  ackArmMagErr = this->setParamWithAck("COM_ARM_MAG_ANG", static_cast<int>(mArmMagErr_deg));
  ackComRCOverride = this->setParamWithAck("COM_RC_OVERRIDE", \
    static_cast<int>(mAutoOverride ? 1:0) | (mOffboardOverride ? 2:0));
  ackGeofenceAction = this->setParamWithAck("GF_ACTION", static_cast<int>(mGeofenceAction));
  ackGeofenceXY = this->setParamWithAck("GF_MAX_HOR_DIST", mGeofenceXY_m);
  ackGeofenceZ = this->setParamWithAck("GF_MAX_VER_DIST", mGeofenceZ_m);
  ackTakeoffAlt = this->setParamWithAck("MIS_TAKEOFF_ALT", mTakeoffAlt_m);
  ackTakeoffSpd = this->setParamWithAck("MPC_TKO_SPEED", mTakeoffSpd_mps);
  ackLandSpd = this->setParamWithAck("MPC_LAND_SPEED", mLandSpd_mps);
  ackXYMaxSpd = this->setParamWithAck("MPC_XY_VEL_MAX", mXYMaxSpd_mps);
  ackZUpMaxSpd = this->setParamWithAck("MPC_Z_VEL_MAX_UP", mZUpMaxSpd_mps);
  ackZDownMaxSpd = this->setParamWithAck("MPC_Z_VEL_MAX_DN", mZDownMaxSpd_mps);

  ackParamSet = ackArmAccErr && ackArmGyroErr && ackArmMagErr && ackComRCOverride && \
    ackGeofenceAction && ackGeofenceXY && ackGeofenceZ && \
    ackTakeoffAlt && ackTakeoffSpd && ackLandSpd && \
    ackXYMaxSpd && ackZUpMaxSpd && ackZDownMaxSpd;
  if (ackParamSet == true) {
    ROS_INFO_NAMED("px4drone", "<doInitialization> Px4 Params Set");
  }

  return ackConnected && ackParamSet;
}

void Px4Drone::doMission(int goalService_, double goalX_, double goalY_, double goalZ_, double goalR_, ros::Rate rate_) {
  ROS_INFO_ONCE_NAMED("px4drone", "<doMission> doMission start");

  bool ackGoTakeoff = false;
  bool ackSetOffboard = false;
//TODO: check if states have changed
//TODO: fix takeoff while takeoff
  switch (goalService_) {
    case -1: // Initial value
      break;

    case 0: // Auto arm(POSCTL) - Takeoff - OFFBOARD
      //TODO: check why "POSCTL Mode" is on the screen many times
      if(mState.armed == false) {
        // wait for POSCTL Mode
        this->setPosctl();
        while(ros::ok() && !(mState.mode == "POSCTL")) {
          ros::spinOnce();
          rate_.sleep();
        }
        std::cout << "POSCTL Mode" << std::endl;
        
        // wait for Arming
        this->setArm();
        while(ros::ok() && mState.armed) {
          ros::spinOnce();
          rate_.sleep();
        }
        std::cout << "Armed" << std::endl;
      }

      //TODO: check if the asserts should be here or in the set/go methods
      // wait for TakeOff
      if (ackGoTakeoff == false) {
        ackGoTakeoff = this->goTakeOff();
        ROS_ASSERT_CMD(ackGoTakeoff == true, \
          this->goLanding(); \
          ROS_FATAL_NAMED("px4drone", "<doMission> Take Off Fail"); \
          ROS_INFO_NAMED("px4drone", "<doMission> goLanding start"); \
          ROS_BREAK());
      }
      else {
        ROS_INFO_NAMED("px4drone", "<doMission> TakeOff");
      }

      // wait for OFFBOARD Mode
      if (ackSetOffboard == false) {
        ackSetOffboard = this->setOffboard();
        ROS_ASSERT_CMD(ackSetOffboard == true, \
          this->goLanding(); \
          ROS_FATAL_NAMED("px4drone", "<doMission> OFFBOARD Mode Fail"); \
          ROS_INFO_NAMED("px4drone", "<doMission> goLanding start"); \
          ROS_BREAK());
      }
      else {
        ROS_INFO_NAMED("px4drone", "<doMission> OFFBOARD Mode");
      }
      break;

    case 1: // Auto arm(POSCTL) - takeoff
      if(mState.armed == false) { // Arming Request
        // wait for POSCTL Mode
        this->setPosctl();
        while(ros::ok() && !(mState.mode == "POSCTL")) {
          ros::spinOnce();
          rate_.sleep();
        }
        std::cout << "POSCTL Mode" << std::endl;

        // wait for Arming
        this->setArm();
        while(ros::ok() && !mState.armed) {
          ros::spinOnce();
          rate_.sleep();
        }
        std::cout << "Armed" << std::endl;
      }

      // wait for TakeOff
      this->goTakeOff();
      while(ros::ok() && !(mState.mode == "AUTO.LOITER")) {
        ros::spinOnce();
        rate_.sleep();
      }
      std::cout << "TakeOff" << std::endl;
      break;

    case 2: // Auto landing - disarm
      // wait for Landing
      this->goLanding();
      while(ros::ok() && !(mState.mode == "AUTO.LAND")) {
        ros::spinOnce();
        rate_.sleep();
      }
      std::cout << "Landing" << std::endl;

      // Auto disarming 2seconds after landing
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
      this->goPosition(goalX_, goalY_, goalZ_, goalR_);
      break;

    default: // Emergency - POSCTL Mode
      // wait for POSCTL Mode
      this->goHold();
      while(ros::ok() && !(mState.mode == "AUTO.LOITER")){
        ros::spinOnce();
        rate_.sleep();
      }
      std::cout << "Hold(Loiter) Mode" << std::endl;
      break;
  }
  return;
}

void Px4Drone::pubRvizTopics(void) {
  ROS_INFO_ONCE_NAMED("px4drone", "<pubRvizTopics> pubRvizTopics start");

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
