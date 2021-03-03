#include "px4drone/px4drone.hpp"

using namespace std;

int main(int argc, char **argv)
{
  cout << fixed << setprecision(2);

  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;
  Px4Drone px4drone = Px4Drone(&nh);
  ros::Rate rate(px4drone.getRosRate());

  // do Initialization
  ROS_ASSERT_CMD(px4drone.doInitialization(rate) == true, \
    ROS_FATAL_NAMED("offboard_node", "\t[offboard_node] Initialization Fail"); \
    ROS_BREAK());
  ROS_INFO_NAMED("offboard_node", "\t[offboard_node] Initialization Done");

  // px4drone.doMission(0, #, #, #, #, rate); // Auto arm(POSCTL) - Takeoff - OFFBOARD

  ROS_INFO_NAMED("offboard_node", "\t[offboard_node] main loop start");
  while(ros::ok())
  {
    // system("clear");
    //TODO: rosparam -> On/Off or launch args and make seperate monitor launch
    // px4.drone.showHelp();
    //TODO: rosparam -> On/Off or launch args and make seperate monitor launch

    px4drone.showState();
    px4drone.showOdom();
    px4drone.showGoalAction();
    px4drone.pubRvizTopics();

    // px4drone.doMission(23, 1, 2, 3, 45, rate);
    px4drone.doMission(px4drone.getGoalService(), px4drone.getGoalX(), px4drone.getGoalY(), px4drone.getGoalZ(), px4drone.getGoalR());
    //TODO: pub with rate keep calling doMission

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
