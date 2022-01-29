# mogi_chess_ros_framework

# Dependencies
BioIK:
https://github.com/dudasdavid/bio_ik

# How to use?

add to .bashrc:
export GAZEBO_MODEL_PATH=~/catkin_ws/src/mogi_chess_ros_framework/mogi_chess_gazebo/gazebo_models/


## 1. Start low level robot nodes:
### Bringup in Gazebo
roslaunch ur_e_gazebo ur3e.launch limited:=true world_file:='$(find mogi_chess_gazebo)/world/chessboard.world' z:=1.02
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true

### Bringup real robot
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=10.0.0.244 limited:=true
roslaunch rh_p12_rn_a_tools bringup.launch
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch limited:=true

## 2. Start HW interface, chess manager and player nodes
### Gazebo:
roslaunch mogi_chess_manager manager.launch sim:=true
### Real robot
roslaunch mogi_chess_manager manager.launch


roslaunch mogi_chess_manager stockfish_player.launch side:=b
roslaunch mogi_chess_manager stockfish_player.launch side:=w
or
roslaunch mogi_chess_manager human_player.launch side:=w

## 3. Start vision package and save samples
roslaunch mogi_chess_vision split_and_save.launch
roslaunch mogi_chess_manager manager.launch save:=true
roslaunch mogi_chess_manager stockfish_player.launch side:=b
roslaunch mogi_chess_manager stockfish_player.launch side:=w

### Gazebo:
roslaunch mogi_chess_manager manager.launch sim:=true save:=true
roslaunch mogi_chess_vision split_and_save.launch sim:=true
roslaunch mogi_chess_manager stockfish_player.launch side:=b
roslaunch mogi_chess_manager stockfish_player.launch side:=w


# Run the piece recognition CNN
roslaunch mogi_chess_vision split_squares.launch

## Train:
python3.8 train.py -s true

## Clock:
david@david-ros:~$ sudo usermod -a -G tty david
david@david-ros:~$ sudo usermod -a -G dialout david



## 4. start manual game and optical tracker:
### Real robot
BRINGUP
roslaunch mogi_chess_manager manager.launch
roslaunch mogi_chess_vision split_and_track.launch
roslaunch mogi_chess_manager manual_player.launch
roslaunch mogi_chess_manager stockfish_player.launch side:=b level:=20

### Gazebo
BRINGUP
roslaunch mogi_chess_manager manager.launch sim:=true
roslaunch mogi_chess_vision split_and_track.launch sim:=true
rosrun mogi_chess_gazebo virtual_chess_clock.py
roslaunch mogi_chess_manager manual_player.launch
roslaunch mogi_chess_manager stockfish_player.launch side:=b

move the pieces in Gazebo

# save additional extra samples:
roslaunch mogi_chess_vision save_snapshots.launch


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
