# offboard_drone
> Control px4 drone as offboard mode
 
## Tips
- Try to run again when it is failed to set the parameters
- Set auto takeoff requires manual stick input
    - You should run QGroundControl and connect a controller like ps4 dualshock
    - Or You can takeoff using the mavlink shell ```pxh>```
    ```commander takeoff```

# vel/pos gain tuning
- Youtube Video
[![px4 sitl vel/pos response (w/, w/o gain tuning](https://img.youtube.com/vi/QMwc--2zklE/0.jpg)](https://www.youtube.com/watch?v=QMwc--2zklE)

# Failsafe Sequence
- ROS_ASSERT_CMD

<img src="https://github.com/finani/offboard_drone/blob/master/images/Offboard_Drone_Failsafe_Sequence.png" width="800px" title="Failsafe Sequence"></img>

# Do Mission
- Goal Service

<img src="https://github.com/finani/offboard_drone/blob/master/images/Offboard_Drone_Do_Mission.png" width="800px" title="Do Mission"></img>
