# __Steps to Launch on Actual Robot__
- Connect to Rethink
  ```
  nmcli connection up Rethink
  ```
- Check connection using 
  ```
  ping baxter.local
  ```
- Set ROS Master URI in each terminal you use using:
  ```
  export ROS_MASTER_URI=http://10.42.0.2:11311
  export ROS_IP=10.42.0.1
  unset ROS_HOSTNAME
  ```
    or use script
    ```
    ./connect_baxter.sh
    ```
- Check if `Joint_states` are published by the robot
  ```
  rostopic echo /robot/joint_states
  ```
- Enable Baxter
  ```
  rosrun baxter_tools enable_robot.py -e
  ```
- Check status of Robot
  ```
  rosrun baxter_tools enable_robot.py -s
  ```
- Source rethink_ws, moveit_robots_ws and this project_ws before using launch file below
  - Verify if all workspaces are set to `ROS_Package_Path` using
  - ```
    printenv | grep ROS_PACKAGE_PATH
    ```
- Launch RViz interface and trajectory node
  ```
  roslaunch baxter_control basic.launch run_trajectory_node:=true 
  ```
  *joint_trajectory_action_server.py` is included in the launch file*

- Disable the robot
  ```
  rosrun baxter_tools enable_robot.py -d
  ```

# __Steps to Launch in Simulation__
- Use command below to launch baxter robot in simulation, optionally set `run_trajectory_node:=true` to run `trajectory` node
  ```
  roslaunch baxter_control basic.launch use_simulation:=true
  ```
  **Wait, this may take time**
## Launch file configuration options:

Args | Default | Description
------------ | ------------- | -------------
`run_trajectory_node:=true`|`false`| Optionally run trajectory node
`use_simulation:=true` |`false`| Optionally load Baxter robot in simulation
`headless:=true`|`true`| Run Gazebo in headless mode
`gui:=true`|`false`| Run Gazebo with GUI
`left_electric_gripper:=true`|`true`| Use baxter with left gripper enabled
`right_electric_gripper:=true` |`true`|Use baxter with right gripper enabled
---

## *Dependencies*
Title | Link
------------ | -------------
RethinkRobotics| [Github](https://github.com/RethinkRobotics)
moveit_robots | [Github](https://github.com/ros-planning/moveit_robots)
---

## Baxter Home Positions:

Left Gripper

position: 
  x: 0.6494685697319907
  y: 0.8371804136224866
  z: 0.043937503445779125
orientation: 
  x: -0.3801916530737042
  y: 0.9235212614290408
  z: 0.020782280350508237
  w: 0.0461614930979309

Right Gripper

position: 
  x: 0.6544330865980147
  y: -0.841170466336113
  z: 0.054577985020326486
orientation: 
  x: 0.3804649229042976
  y: 0.9229889441554372
  z: -0.023015433956129042
  w: 0.05299189755112533
