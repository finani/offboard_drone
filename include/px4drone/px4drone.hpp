#pragma once

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

constexpr double const_pi() { return atan(1)*4; }
constexpr double const_R2D() { return 180.0 / const_pi(); }
constexpr double const_D2R() { return const_pi() / 180.0; }

class Px4Drone {
 public:
  Px4Drone(ros::NodeHandle *nh_);
  ~Px4Drone();

  void cbState(const mavros_msgs::State::ConstPtr& msg_);
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg_);
  void cbGoalAction(const std_msgs::Float32MultiArray::ConstPtr& msg_);

  void showState(void);
  void showOdom(void);
  void showGoalAction(void);

  mavros_msgs::State getState(void);
  nav_msgs::Odometry getOdom();
  float getGoalService(void);
  float getGoalX(void);
  float getGoalY(void);
  float getGoalZ(void);
  float getGoalR(void);
  float getGoalVXY(void);
  float getGoalVZUp(void);
  float getGoalVZDown(void);
  float getRosRate(void);

  bool setParamWithAck(std::string param_id_, int value_);
  bool setParamWithAck(std::string param_id_, float value_);
  bool setArm(void);
  bool setDisarm(void);
  bool setAcro(void);
  bool setStabilized(void);
  bool setAltctl(void);
  bool setPosctl(void);
  bool setOffboard(void);

  bool goTakeOff(float takeoffAlt_m_, float takeoffSpd_mps_);
  bool goLanding(float landSpd_mps_);
  void goForce(void);
  void goAccel(void);
  void goAngularVelocity(void);
  void goAttitude(void);
  void goVelocity(double xEast_mps_, double yNorth_mps_, double zUp_mps_, double headingCCW_deg_);
  void goVelocityBody(double xForward_mps_, double yLeft_mps_, double zUp_mps_, double headingCCW_deg_);
  void goPosition(double xEast_m_, double yNorth_m_, double zUp_m_, double headingCCW_deg_, double VXY_mps_, double VZUp_mps_, double VZDown_mps_);

  bool doInitialization(ros::Rate rate_);
  void doMission(int goalService_, double goalX_, double goalY_, double goalZ_, double goalR_, double goalVXY_, double goalVZUp_, double goalVZDown_, ros::Rate rate_);

  void pubRvizTopics(void);

 private:
  ros::Subscriber mState_sub;
  ros::Subscriber mOdom_sub;
  ros::Subscriber mGoalAction_sub;
  ros::Publisher mCmdVel_pub;
  ros::Publisher mCmdPos_pub;
  ros::Publisher mUavPath_pub;
  ros::Publisher mUavPos_pub;
  ros::ServiceClient mArming_client;
  ros::ServiceClient mSetMode_client;
  ros::ServiceClient mParamSet_client;
  ros::ServiceClient mParamGet_client;

  mavros_msgs::State mState;
  nav_msgs::Odometry mOdom;
  mavros_msgs::CommandBool mArmCmd;
  mavros_msgs::SetMode mSetMode;
  mavros_msgs::ParamSet mParamSet;
  mavros_msgs::ParamGet mParamGet;

  tf::Vector3 mCurPos_m;
  tf::Vector3 mCurVel_mps;
  tf::Quaternion mCurQ;
  tf::Vector3 mCurRpy_rad;
  tf::Vector3 mCurRpy_deg;
  tf::Vector3 mCurRpyRate_dps;

  bool mAutoOverride;
  bool mOffboardOverride;

  std_msgs::Float32MultiArray mGoalAction;
  int mGoalService;
  double mGoalX;
  double mGoalY;
  double mGoalZ;
  double mGoalR;
  double mGoalVXY;
  double mGoalVZUp;
  double mGoalVZDown;
  float mRosRate;
};