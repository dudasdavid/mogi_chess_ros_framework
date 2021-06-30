# mogi_chess_ros_framework

# Dependencies


# How to use?

## 1. Start low level robot nodes:
### Bringup in Gazebo
export GAZEBO_MODEL_PATH=~/catkin_ws/src/mogi_chess_ros_framework/mogi_chess_gazebo/gazebo_models/
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
roslaunch mogi_chess_vision camera.launch
roslaunch mogi_chess_vision split_and_save.launch
roslaunch mogi_chess_manager manager.launch save:=true

### Gazebo:
roslaunch mogi_chess_manager manager.launch sim:=true save:=true
roslaunch mogi_chess_vision split_and_save.launch sim:=true
