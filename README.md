# mogi_chess_ros_framework

## Working MoveIt Dependencies:
- srdfdom 0.6.0
- geometric_shapes 0.7.3
- moveit_msgs 0.11.1
- moveit 1.0.7

## Dependencies:
https://github.com/dudasdavid/Universal_Robots_ROS_Driver/tree/ur3e-tool
https://github.com/dudasdavid/universal_robot/tree/chess-detector
https://github.com/dudasdavid/RH-P12-RN-A/tree/ur3e-gripper
https://github.com/dudasdavid/mogi_chess_ros_framework/tree/main
https://github.com/dudasdavid/usb_cam/tree/develop

# How to use?

## 0. Make sure every gazebo path is correct for the simulation!
add to .bashrc:
```bash
export GAZEBO_MODEL_PATH=~/catkin_ws/src/mogi_chess_ros_framework/mogi_chess_gazebo/gazebo_models/
```
## 1. Start low level robot nodes:
### Bringup in Gazebo
```bash
roslaunch ur_e_gazebo ur3e.launch limited:=true world_file:='$(find mogi_chess_gazebo)/world/chessboard.world' z:=1.02
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true
```
### Bringup real robot
#### UR3e
```bash
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=10.0.0.244 limited:=true
roslaunch rh_p12_rn_a_tools bringup.launch
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch limited:=true
```
#### UR5e
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.50.221 limited:=true
roslaunch rh_p12_rn_a_tools bringup.launch
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch limited:=true
```

## 2. Start HW interface, chess manager and player nodes
### Gazebo:
```bash
roslaunch mogi_chess_manager manager.launch sim:=true
```
### Real robot
```bash
roslaunch mogi_chess_manager manager.launch
```
### Always do the calibration with the chess manager first...
... by pressing `c`, this can validate the x, y offset of the table, height, orientation of the robot, etc is correct

### If calibration is all right we can start a test game without camera feedback
```bash
roslaunch mogi_chess_manager stockfish_player.launch side:=b
roslaunch mogi_chess_manager stockfish_player.launch side:=w

or

roslaunch mogi_chess_manager human_player.launch side:=w
```
## Continue only if everything is working fine including chess clock position and hit pieces drop location!!!

## 3. Prepare learning set and teach the neural network, if network is ready go to 4.1
### Start vision package and save samples
```bash
roslaunch mogi_chess_vision split_and_save.launch
roslaunch mogi_chess_manager manager.launch save:=true
roslaunch mogi_chess_manager stockfish_player.launch side:=b
roslaunch mogi_chess_manager stockfish_player.launch side:=w
```

### Gazebo:
```bash
roslaunch mogi_chess_manager manager.launch sim:=true save:=true
roslaunch mogi_chess_vision split_and_save.launch sim:=true
roslaunch mogi_chess_manager stockfish_player.launch side:=b
roslaunch mogi_chess_manager stockfish_player.launch side:=w
```

### Train the model:
```bash
python3.8 train_simple_model.py -s true
```
Do not use train.py because that was a not working dead end with CNN detection for every piece
Simple model is just distinguishing black/white/empty fields

### 4.0. Setup the clock serial port:
```bash
microlab@microlab:~$ sudo usermod -a -G tty microlab
microlab@microlab:~$ sudo usermod -a -G dialout microlab
sudo chmod 666 /dev/ttyACM0
```

## 4.1. start manual game and optical tracker:
### Real robot
```bash
roslaunch mogi_chess_manager manager.launch
roslaunch mogi_chess_vision split_and_track.launch
roslaunch mogi_chess_manager manual_player.launch
roslaunch mogi_chess_manager stockfish_player.launch side:=b level:=20
```
### Gazebo
```bash
roslaunch mogi_chess_manager manager.launch sim:=true
roslaunch mogi_chess_vision split_and_track.launch sim:=true
rosrun mogi_chess_gazebo virtual_chess_clock.py
roslaunch mogi_chess_manager manual_player.launch
roslaunch mogi_chess_manager stockfish_player.launch side:=b
```
and move the pieces in Gazebo

# save additional extra samples:
```bash
roslaunch mogi_chess_vision save_snapshots.launch
```

# Useful links:
http://chess.fortherapy.co.uk/home/chess-piece-identification-technology/

Note:
manual player's promotion is always queen!

Python dependencies:
sudo apt-get install python3-tk
python3.8 -m pip install chess
python3.8 -m pip install cairoSVG



### UPDATE:
roslaunch ur_gazebo ur3e_bringup.launch (gazebo_world:='$(find mogi_chess_gazebo)/world/chessboard.world' spawn_z:=1.02)
roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch sim:=true

### UR5e:
roslaunch ur_gazebo ur5e_bringup.launch spawn_y:=-0.3
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=true
roslaunch mogi_chess_manager manager.launch sim:=true config:=ur5e.yaml


### AUTO PLAY:
roslaunch ur_gazebo ur3e_bringup.launch gazebo_world:='$(find mogi_chess_gazebo)/world/chessboard_black_wins.world'
roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch sim:=true

roslaunch mogi_chess_hw_interface robot_interface.launch sim:=true

bash autolaunch.bash
(roslaunch mogi_chess_manager auto_manager.launch)





# Obsolete Dependencies
BioIK:
https://github.com/dudasdavid/bio_ik