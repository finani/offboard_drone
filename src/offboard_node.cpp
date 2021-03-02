#include "px4drone/px4drone.hpp"

using namespace std;

int main(int argc, char **argv)
{
  cout << fixed << setprecision(2);

  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;
  Px4Drone px4drone = Px4Drone(&nh);
  ros::Rate rate(px4drone.getRosRate());

  // wait for Initialization
  // bool ackDoInitialization = px4drone.doInitialization(rate); //TODO: check if it is needed
  ROS_ASSERT_MSG(px4drone.doInitialization(rate) == true, "Initialization Fail");

  // px4drone.doMission(0, #, #, #, #, rate); // Auto arm(POSCTL) - Takeoff - OFFBOARD

  ROS_INFO_NAMED("offboard_node", "<offboard_node> main loop start");
  while(ros::ok())
  {
    // system("clear");
    //TODO: rosparam -> On/Off or launch args and make seperate monitor launch
    // px4.drone.showHelp();
    //TODO: rosparam -> On/Off or launch args and make seperate monitor launch

    // px4drone.showState();
    // px4drone.showOdom();
    // px4drone.showGoalAction();

    // px4drone.doMission(23, 1, 2, 3, 45, rate);
    px4drone.doMission(px4drone.getGoalService(), px4drone.getGoalX(), px4drone.getGoalY(), px4drone.getGoalZ(), px4drone.getGoalR(), rate);
    //TODO: pub with rate keep calling doMission

    px4drone.pubRvizTopics();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
