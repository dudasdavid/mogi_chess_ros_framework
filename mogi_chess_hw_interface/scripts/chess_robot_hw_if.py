#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
from copy import deepcopy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from robotis_controller_msgs.msg import SyncWriteItem
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState, Constraints
from gazebo_msgs.msg import ContactsState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from mogi_chess_msgs.srv import RobotCommand, RobotCommandResponse, RobotStatus, RobotStatusResponse

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    param_sim = rospy.get_param('~sim', "false")
    if param_sim == True:
        self.simulation = True
    else:
        self.simulation = False

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Create a publisher for the real ROBOTIS gripper
    robotis_publisher = rospy.Publisher('/robotis/direct/sync_write_item', SyncWriteItem, queue_size=5)

    # And create another one for the Gazebo simulated gripper
    gazebo_publisher = rospy.Publisher('/gripper_gazebo_controller/command', JointTrajectory, queue_size=1)


    # Chess steps subscriber
    self.subscribe_ena = False
    rospy.Subscriber("chess_steps", String, self.chess_step_callback)

    # Create chess manager service handler
    s_command = rospy.Service('robot_command', RobotCommand, self.serve_robot_command)
    s_status = rospy.Service('robot_status', RobotStatus, self.serve_robot_status)

    # Subscribe to touch sensor topics (Gazebo only)
    if self.simulation:
      rospy.Subscriber("left_contact", ContactsState, self.get_contacts)
      rospy.Subscriber("right_contact", ContactsState, self.get_contacts)
      self.attached = False
      self.attached_to = None
      self.left_collider = None
      self.right_collider = None
      self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
      self.attach_srv.wait_for_service()
      self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
      self.detach_srv.wait_for_service()
      self.detach_time = 0


    # Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    self.constraints = Constraints()

    # Robotis gripper
    self.goal_position_msg = SyncWriteItem()
    self.goal_position_msg.item_name = "goal_position"
    self.goal_position_msg.joint_name = ["gripper"]

    # Gazebo gripper
    self.gazebo_trajectory_command = JointTrajectory()
    self.gazebo_trajectory_command.joint_names = ["gripper"]
    self.gazebo_trajectory_point = JointTrajectoryPoint()
    self.gazebo_trajectory_point.time_from_start = rospy.rostime.Duration(1,0)
    self.gazebo_trajectory_point.velocities = [0.0]

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.robotis_publisher = robotis_publisher
    self.gazebo_publisher = gazebo_publisher

    # Chess related variables
    ### THESE VALUES ARE SET BASED ON BOARD AND TABLE
    # Y coordinate
    self.rows = {"1": 0.453, "2": 0.415, "3": 0.378, "4": 0.340, "5": 0.303, "6": 0.265, "7": 0.228, "8": 0.190}
    
    # X coordinate
    self.columns  = {"a": 0.131, "b": 0.094, "c": 0.056, "d": 0.019, "e": -0.019, "f": -0.056, "g": -0.094, "h": -0.131}

    self.z_table_offset = 0.115 # THIS MUST BE SET TO THE REAL TABLE!
    ### THESE VALUES ARE SET BASED ON FIGURES
    self.z_high = self.z_table_offset + 0.06 #0.222
    self.z_low = self.z_table_offset + 0.01 #0.17
    self.z_drop = self.z_table_offset + 0.012 #0.172
    ### THIS VALUE IS USED DURING CALIBRATION
    self.z_touch_table = self.z_table_offset + 0.005 #THIS IS USED DURING CALIBRATION, IT ALMOST TOUCHES THE TABLE!
    ### THESE VALUES ARE SET TO BE OUTSIDE THE TABLE
    self.z_drop_to_table = self.z_table_offset + 0.005
    self.x_drop_to_table = self.columns["h"] - 0.08
    self.y_drop_to_table = self.rows["8"] + 0.02
    self.x_drop_to_table_offset = 0.03
    
    # Slightly adjust a few parameters for the simulation!
    if self.simulation:
      self.x_drop_to_table_offset = 0.04
      self.z_drop = self.z_low
      self.z_drop_to_table = self.z_low
      

    self.drop_slots = [(self.x_drop_to_table, self.y_drop_to_table)]
    for i in range(0,32):
        self.y_drop_to_table += 0.03
        if self.y_drop_to_table >= self.rows["3"]:
          self.y_drop_to_table = self.rows["8"] + 0.018
          self.x_drop_to_table -= self.x_drop_to_table_offset
        self.drop_slots.append((self.x_drop_to_table, self.y_drop_to_table))
    print(self.drop_slots)

    if self.simulation:
        self.wait_time = 0.2
        self.gripper_wait_time = 1.5
    else:
        self.wait_time = 0.2
        self.gripper_wait_time = 0.5

    self.robot_is_moving = False

    self.clock_z_up = self.z_table_offset + 0.07
    self.clock_z_down = self.z_table_offset + 0.037
    self.clock_x_b = 0.260
    self.clock_y_b = 0.300
    self.clock_x_w = 0.260
    self.clock_y_w = 0.380

  def serve_robot_status(self, req):
      return RobotStatusResponse(self.robot_is_moving)

  def serve_robot_command(self, cmd):
    self.robot_is_moving = True
    cmd_list = cmd.command.split(";")
    print(cmd_list)
    side = cmd_list[0]
    if cmd_list[1] == "n":
        start = cmd_list[2]
        end = cmd_list[3]
        x = self.columns[start[0]]
        y = self.rows[start[1]]
        self.go_and_grab((x,y))
        x = self.columns[end[0]]
        y = self.rows[end[1]]
        self.go_and_drop((x,y))
        self.push_the_clock(side)
        self.go_to_home()
    elif cmd_list[1] == "x":
        hit_start = cmd_list[2]
        hit_id = cmd_list[3]
        start = cmd_list[4]
        end = cmd_list[5]
        x = self.columns[hit_start[0]]
        y = self.rows[hit_start[1]]
        self.go_and_grab((x,y))
        self.go_and_drop(self.drop_slots[int(hit_id)], True)
        x = self.columns[start[0]]
        y = self.rows[start[1]]
        self.go_and_grab((x,y))
        x = self.columns[end[0]]
        y = self.rows[end[1]]
        self.go_and_drop((x,y))
        self.push_the_clock(side)
        self.go_to_home()
    elif cmd_list[1] == "c":
        start1 = cmd_list[2]
        end1 = cmd_list[3]
        start2 = cmd_list[4]
        end2 = cmd_list[5]
        x = self.columns[start1[0]]
        y = self.rows[start1[1]]
        self.go_and_grab((x,y))
        x = self.columns[end1[0]]
        y = self.rows[end1[1]]
        self.go_and_drop((x,y))
        x = self.columns[start2[0]]
        y = self.rows[start2[1]]
        self.go_and_grab((x,y))
        x = self.columns[end2[0]]
        y = self.rows[end2[1]]
        self.go_and_drop((x,y))
        self.push_the_clock(side)
        self.go_to_home()
    elif cmd_list[1] == "e":
        start = cmd_list[2]
        end = cmd_list[3]
        hit_start = cmd_list[4]
        hit_id = cmd_list[5]
        x = self.columns[start[0]]
        y = self.rows[start[1]]
        self.go_and_grab((x,y))
        x = self.columns[end[0]]
        y = self.rows[end[1]]
        self.go_and_drop((x,y))
        x = self.columns[hit_start[0]]
        y = self.rows[hit_start[1]]
        self.go_and_grab((x,y))
        self.go_and_drop(self.drop_slots[int(hit_id)], True)
        self.push_the_clock(side)
        self.go_to_home()
    elif cmd_list[1] == "p":
        promotion_id = cmd_list[2]
        promotion_end = cmd_list[3]
        hit_start = cmd_list[4]
        hit_id = cmd_list[5]
        self.go_and_grab(self.drop_slots[int(promotion_id)], True)
        x = self.columns[promotion_end[0]]
        y = self.rows[promotion_end[1]]
        self.go_and_drop((x,y))
        x = self.columns[hit_start[0]]
        y = self.rows[hit_start[1]]
        self.go_and_grab((x,y))
        self.go_and_drop(self.drop_slots[int(hit_id)], True)
        self.push_the_clock(side)
        self.go_to_home()

    elif cmd_list[1] == "invalid":
        self.push_the_clock(side)
        self.go_to_home()
    else:
        self.robot_is_moving = False
        return RobotCommandResponse(False)

    self.robot_is_moving = False
    return RobotCommandResponse(True)

  def enable_subscribe(self):
    self.subscribe_ena = True

  def chess_step_callback(self, data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if self.subscribe_ena:
      (steps, hit) = data.data.split(";")
      print(steps)
      print(hit)
      if hit == "True": hit = True
      else: hit = False
      #self.do_chess_step(steps[:2], steps[2:], hit)

  def get_contacts(self, msg):
    if (len(msg.states) == 0):
        #rospy.loginfo("No contacts were detected!")
        self.left_collider = None
        self.right_collider = None
    else:
        
        if 'rh_p12_rn_l2' in msg.states[0].collision1_name:
            other_object = msg.states[0].collision2_name.split("::")[0]
            #rospy.loginfo("Left collision detected with %s." % other_object)
            self.left_collider = other_object
        elif 'rh_p12_rn_l2' in msg.states[0].collision2_name:
            other_object = msg.states[0].collision1_name.split("::")[0]
            #rospy.loginfo("Left collision detected with %s." % other_object)
            self.left_collider = other_object
        if 'rh_p12_rn_r2' in msg.states[0].collision1_name:
            other_object = msg.states[0].collision2_name.split("::")[0]
            #rospy.loginfo("Right collision detected with %s." % other_object)
            self.right_collider = other_object
        elif 'rh_p12_rn_r2' in msg.states[0].collision2_name:
            other_object = msg.states[0].collision1_name.split("::")[0]
            #rospy.loginfo("Right collision detected with %s." % other_object)
            self.right_collider = other_object
        else:
            pass
            #rospy.loginfo("Unknown collision!")
        
        if rospy.get_time() > (self.detach_time + 5) and self.attached == False and self.right_collider is not None and self.right_collider == self.left_collider:
            self.attached = True
            self.attached_to = self.right_collider
            req = AttachRequest()
            req.model_name_1 = "robot"
            req.link_name_1 = "wrist_3_link"
            req.model_name_2 = self.attached_to
            req.link_name_2 = "link_0"

            self.attach_srv.call(req)
            rospy.loginfo("Attached: %s!" % self.attached_to)




  def set_gripper(self, status):
      if status == "open":
        # Real gripper value
        self.goal_position_msg.value = [560]
        # Gazebo gripper value
        self.gazebo_trajectory_point.positions = [0.8]

        if self.simulation:
          if self.attached == True:
              req = AttachRequest()
              req.model_name_1 = "robot"
              req.link_name_1 = "wrist_3_link"
              req.model_name_2 = self.attached_to
              req.link_name_2 = "link_0"

              self.detach_time = rospy.get_time()
              self.detach_srv.call(req)
              rospy.loginfo("Detached: %s!" % self.attached_to)
              self.attached = False
              self.left_collider = None
              self.right_collider = None
              self.attached_to = None

      else:
        self.goal_position_msg.value = [740]
        self.gazebo_trajectory_point.positions = [1.135]
        

      # Publish real gripper position
      self.robotis_publisher.publish(self.goal_position_msg)

      # Publish gazebo gripper position
      self.gazebo_trajectory_command.header.stamp = rospy.Time.now()
      self.gazebo_trajectory_command.points = [self.gazebo_trajectory_point]
      self.gazebo_publisher.publish(self.gazebo_trajectory_command)


  def go_and_grab(self, xy, out = False):

    x = xy[0]
    y = xy[1]

    if out:
        orientation = 90
    else:
        orientation = 45


    # 0) Make sure that the gripper is open
    self.set_gripper("open")

    # 1.1) Go above end position
    self.go_to_pose_goal(x, y, self.z_high, orientation)
    rospy.sleep(self.wait_time)

    # 1.2) Go down
    self.go_to_pose_goal(x, y, self.z_low, orientation)
    rospy.sleep(self.wait_time)

    # 1.3) Grab the figure
    self.set_gripper("closed")
    rospy.sleep(self.gripper_wait_time)

    # 1.4) Move up
    self.go_to_pose_goal(x, y, self.z_high, orientation)
    rospy.sleep(self.wait_time)


  def go_and_drop(self, xy, out = False):

    x = xy[0]
    y = xy[1]

    if out:
        orientation = 90
        z_drop = self.z_drop_to_table
    else:
        orientation = 45
        z_drop = self.z_drop

    # 6) Go above end position
    self.go_to_pose_goal(x, y, self.z_high, orientation)
    rospy.sleep(self.wait_time)

    # 7) Move down
    self.go_to_pose_goal(x, y, z_drop, orientation)
    rospy.sleep(self.wait_time)

    # 8) Open gripper
    self.set_gripper("open")
    rospy.sleep(self.gripper_wait_time)

    # 9) Move up
    self.go_to_pose_goal(x, y, self.z_high, orientation)
    rospy.sleep(self.wait_time)

  def push_the_clock(self, side):

    if side == "w":
        clock_x = self.clock_x_w
        clock_y = self.clock_y_w
    else:
        clock_x = self.clock_x_b
        clock_y = self.clock_y_b

    self.set_gripper("closed")    

    # 6) Go above end position
    self.go_to_pose_goal(clock_x, clock_y, self.clock_z_up, 90)
    rospy.sleep(self.wait_time)

    # 7) Move down
    self.go_to_pose_goal(clock_x, clock_y, self.clock_z_down, 90)
    rospy.sleep(self.wait_time)

    # 9) Move up
    self.go_to_pose_goal(clock_x, clock_y, self.clock_z_up, 90)
    rospy.sleep(self.wait_time)

  # Obsolete
  def do_chess_step(self, start, end, hit=False):

      # 0) Make sure that the gripper is open
      self.set_gripper("open")
      if self.simulation:
        wait_time = 0.5
        gripper_wait_time = 3
      else:
        wait_time = 0.2
        gripper_wait_time = 0.5
        

      # 1) If it's a hit:
      if hit:
        # 1.1) Go above end position
        self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
        rospy.sleep(wait_time)

        # 1.2) Go down
        self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_low)
        rospy.sleep(wait_time)

        # 1.3) Grab the figure
        self.set_gripper("closed")
        rospy.sleep(gripper_wait_time)

        # 1.4) Move up
        self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
        rospy.sleep(wait_time)

        # 1.5) Go out of the chess table
        self.go_to_pose_goal(self.x_drop_to_table, self.y_drop_to_table, self.z_high, orientation = 90)
        rospy.sleep(wait_time)

        # 1.6) Go down
        self.go_to_pose_goal(self.x_drop_to_table, self.y_drop_to_table, self.z_drop_to_table, orientation = 90)
        rospy.sleep(wait_time)

        # 1.7) Release the figure
        self.set_gripper("open")
        rospy.sleep(gripper_wait_time)

        # 1.8) Move up
        self.go_to_pose_goal(self.x_drop_to_table, self.y_drop_to_table, self.z_high, orientation = 90)
        rospy.sleep(wait_time)

        # 1.9) Set nex drop X and Y coordinates


      # 2) Go above start position
      self.go_to_pose_goal(self.columns[start[0]], self.rows[start[1]], self.z_high)
      rospy.sleep(wait_time)

      # 3) Go down
      self.go_to_pose_goal(self.columns[start[0]], self.rows[start[1]], self.z_low)
      rospy.sleep(wait_time)

      # 4) Grab the figure
      self.set_gripper("closed")
      rospy.sleep(gripper_wait_time)

      # 5) Move up
      self.go_to_pose_goal(self.columns[start[0]], self.rows[start[1]], self.z_high)
      rospy.sleep(wait_time)

      # 6) Go above end position
      self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
      rospy.sleep(wait_time)

      # 7) Move down
      self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_drop)
      rospy.sleep(wait_time)

      # 8) Open gripper
      self.set_gripper("open")
      rospy.sleep(gripper_wait_time)

      # 9) Move up
      self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
      rospy.sleep(wait_time)

      # 10) Go home
      self.go_to_home()
      rospy.sleep(wait_time)


  def go_to_home(self):

    ## Planning to a Joint Goal
    ## The UR's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = -1.5708
    joint_goal[1] = -1.5708
    joint_goal[2] = -1.0472
    joint_goal[3] = -1.0472
    joint_goal[4] = 1.5708
    joint_goal[5] = 0.7854

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # Open gripper
    self.set_gripper("open")

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, x, y, z, orientation = 45):
    ## Planning to a Pose Goal
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #pose_goal = geometry_msgs.msg.Pose()
    # set proper quaternion for the vertical orientation: https://quaternions.online/
    if orientation == 90:
      # 90 deg gripper position during dropping the pieces
      #pose_goal.orientation.x = -0.707
      #pose_goal.orientation.y = 0.707
      orientation_x = -0.707
      orientation_y = 0.707
    else:
      # default 45 deg gripper position
      #pose_goal.orientation.x = -0.383
      #pose_goal.orientation.y = 0.924
      orientation_x = -0.383
      orientation_y = 0.924
    
    #pose_goal.position.x = x
    #pose_goal.position.y = y
    #pose_goal.position.z = z

    # Use cartesian plan to the new pose
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x = x
    wpose.position.y = y
    wpose.position.z = z
    wpose.orientation.x = orientation_x
    wpose.orientation.y = orientation_y
    wpose.orientation.z = 0
    wpose.orientation.w = 0
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.005,        # eef_step
                                       0.0)         # jump_threshold

    self.move_group.execute(plan, wait=True)

    # This was the old go to pose plan!
    #self.move_group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    #plan = self.move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    #return all_close(pose_goal, current_pose, 0.01)
    return all_close(wpose, current_pose, 0.01)

  def do_calibration(self):
    raw_input("============ Press `Enter` to go A8 (down) ...")
    self.set_gripper("closed")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["8"], z = self.z_touch_table)
    #raw_input("============ Press `Enter` to go D8 (down) ...")
    #self.go_to_pose_goal(x = self.columns["d"], y = self.rows["8"], z = self.z_touch_table)
    #raw_input("============ Press `Enter` to go E8 (down) ...")
    #self.go_to_pose_goal(x = self.columns["e"], y = self.rows["8"], z = self.z_touch_table)
    #raw_input("============ Press `Enter` to go F8 (down) ...")
    #self.go_to_pose_goal(x = self.columns["f"], y = self.rows["8"], z = self.z_touch_table)
    #raw_input("============ Press `Enter` to go G8 (down) ...")
    #self.go_to_pose_goal(x = self.columns["g"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go H8 (down) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go H1 (down) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["1"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go A1 (down) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["1"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go A8 (down) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go A8 (up) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["8"], z = self.z_high)
    #raw_input("============ Press `Enter` to go D8 (up) ...")
    #self.go_to_pose_goal(x = self.columns["d"], y = self.rows["8"], z = self.z_high)
    #raw_input("============ Press `Enter` to go E8 (up) ...")
    #self.go_to_pose_goal(x = self.columns["e"], y = self.rows["8"], z = self.z_high)
    #raw_input("============ Press `Enter` to go F8 (up) ...")
    #self.go_to_pose_goal(x = self.columns["f"], y = self.rows["8"], z = self.z_high)
    #raw_input("============ Press `Enter` to go G8 (up) ...")
    #self.go_to_pose_goal(x = self.columns["g"], y = self.rows["8"], z = self.z_high)
    raw_input("============ Press `Enter` to go H8 (up) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["8"], z = self.z_high)
    raw_input("============ Press `Enter` to go H1 (up) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["1"], z = self.z_high)
    raw_input("============ Press `Enter` to go A1 (up) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["1"], z = self.z_high)
    raw_input("============ Press `Enter` to go home ...")
    self.go_to_home()

  def display_trajectory(self, plan):
    ## Displaying a Trajectory
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    ## Executing a Plan
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
  try:

    moveit_commander = MoveGroupPythonInteface()

    # Set max velocity
    moveit_commander.move_group.set_max_velocity_scaling_factor(0.4)
    # Set tolerances, without that IK cannot do a valid plan
    moveit_commander.move_group.set_goal_position_tolerance(0.0005)
    moveit_commander.move_group.set_goal_orientation_tolerance(0.001)
    
    raw_input("============ Press `Enter` to go home...")
    moveit_commander.go_to_home()
    while 1:
      print("============ Select mode:")
      print("============   p: Play")
      print("============   c: Calibration")
      print("============   s: Subscribe to chess_steps topic")
      ret = raw_input()
      if ret in ["p", "c", "s"]:
        break
      else:
        pass

    if ret == "c":
      moveit_commander.do_calibration()

    elif ret == "p":
      raw_input("============ Press `Enter` to start playing...")
      moveit_commander.do_chess_step("d7", "d5")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("e7", "e6")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("g8", "f6")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("f8", "b4")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("e8", "a4")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("b4", "c3", hit = True)

    elif ret == "s":
      moveit_commander.enable_subscribe()
      rospy.spin()
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
