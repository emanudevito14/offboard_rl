Before cloning this repository, you need to follow the README at this [link](https://github.com/emanudevito14/hexacopter_model) to use PX4_Autopilot with my hex_6 model. 
Furthermore, this repository must be used with the px4_msgs package [here](https://github.com/PX4/px4_msgs/tree/main), 
the QGroundControl application, and the DDS_run.sh file (which you can find [here](https://github.com/RoboticsLab2025/aerial_robotics)).


Clone this repository in ros2_ws/src
```
git clone https://github.com/emanudevito14/offboard_rl.git

```
Put px4_msgs in ros2_ws/src

Start the QGroundControl application

Go in PX4-Autopilot folder
```
make px4_sitl gz_hex_6
```

Open new terminal and go in the folder where there is DDS_run.sh 
```
. DDS_run.sh
```
Open another terminal in ros2_ws
```
colcon build
source install/setup.bash
ros2 launch offboard_rl traj_planning.launch.py
```

Open new terminal 
```
ros2 run plotjuggler plotjuggler

```
following this [video](https://www.youtube.com/watch?v=GE66A7KPls4) to visualize plot xy, velocity and acceleration


