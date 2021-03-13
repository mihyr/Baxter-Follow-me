- `nmcli connection up Rethink`
- `ping baxter.local`
- ```
    export ROS_MASTER_URI=http://10.42.0.2:11311
    export ROS_IP=10.42.0.1
    unset ROS_HOSTNAME
    ```
- `rostopic echo /robot/joint_states`
- `rosrun baxter_tools enable_robot.py -e`

    ```
    roslaunch baxter_control basic.launch use_simulation:=true 
    ```
- `rosrun baxter_interface joint_trajectory_action_server.py` included in launch file
- `rosrun baxter_tools enable_robot.py -d`