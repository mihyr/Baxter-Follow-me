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


baxter home positions:

Left 

position: 
  x: 0.6494685697319907
  y: 0.8371804136224866
  z: 0.043937503445779125
orientation: 
  x: -0.3801916530737042
  y: 0.9235212614290408
  z: 0.020782280350508237
  w: 0.0461614930979309

Right

position: 
  x: 0.6544330865980147
  y: -0.841170466336113
  z: 0.054577985020326486
orientation: 
  x: 0.3804649229042976
  y: 0.9229889441554372
  z: -0.023015433956129042
  w: 0.05299189755112533
