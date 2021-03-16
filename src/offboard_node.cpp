#include "px4drone/px4drone.hpp"

int main(int argc, char **argv)
{
  std::cout << std::fixed << std::setprecision(2);

  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;
  Px4Drone px4drone = Px4Drone(&nh);
  ros::Rate rate(px4drone.getRosRate());

  // do Initialization
  ROS_ASSERT_CMD(px4drone.doInitialization(&nh, &rate) == true, \
    ROS_FATAL_NAMED("offboard_node", "\t[offboard_node] Initialization Fail"); \
    ROS_BREAK());
  ROS_INFO_NAMED("offboard_node", "\t[offboard_node] Initialization Done");

  while(ros::ok())
  {
    ROS_INFO_ONCE_NAMED("offboard_node", "\t[offboard_node] main loop start");

    px4drone.showState();
    px4drone.showOdom();
    px4drone.showGoalAction();
    px4drone.pubRvizTopics();

    // px4drone.doMission(23, 1, 2, 3, 45);
    px4drone.doMission(px4drone.getGoalService(), px4drone.getGoalX(), px4drone.getGoalY(), px4drone.getGoalZ(), px4drone.getGoalR());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
