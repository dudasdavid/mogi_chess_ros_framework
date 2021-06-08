#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from Chessnut import Game
from stockfish import Stockfish
import rospkg
import time
from mogi_chess_msgs.srv import ReadStatus, MakeMovement

def read_status_client():
    rospy.wait_for_service('read_status')
    try:
        read_status_service = rospy.ServiceProxy('read_status', ReadStatus)
        resp1 = read_status_service()
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def make_movement_client(player, movement, fen, robot):
    rospy.wait_for_service('make_movement')
    try:
        make_movement_service = rospy.ServiceProxy('make_movement', MakeMovement)
        resp1 = make_movement_service(player, movement, fen, robot)
        return resp1.valid
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def stockfish_step(fen):
    global stockfish

    stockfish.set_fen_position(fen)
    stockfish_step = stockfish.get_best_move()

    print(stockfish.get_evaluation())
    print(stockfish_step)

    return stockfish_step

def player_step(fen):
    global stockfish

    stockfish.set_fen_position(fen)
    print(stockfish.get_board_visual())
    while 1:
        ret = input("Your move (e.g. e2e4): ")
        if ret == "exit" or ret=="q":
            exit()
        if ret == "":
            continue
        if stockfish.is_move_correct(ret):
            print("Valid movement: %s" % ret)
            break
        else:
            print("Invalid movement: %s!" % ret)

    return ret

# Set up ROS stuff

rospy.init_node('chess_player', anonymous=True)

rospack = rospkg.RosPack()
path = rospack.get_path('mogi_chess_stockfish')
stockfish_path = path + "/stockfish/stockfish_13_linux_x64_bmi2"
print(stockfish_path)
stockfish = Stockfish(stockfish_path, parameters={"Threads": 2, "Minimum Thinking Time": 30})
stockfish.set_skill_level(20)
print(f"Using Stockfish v{stockfish.get_stockfish_major_version()}, skill level: {stockfish.get_parameters()['Skill Level']}")

chessgame = Game()

print_once_flag = True
robot_move = True

param_side = rospy.get_param('~side', "w")
param_type = rospy.get_param('~type', "human")

while not rospy.is_shutdown():
    resp = read_status_client().split(";")
    print(resp)
    status = resp[0]
    point = resp[1]
    current_side = resp[2]
    fen = resp[3]
    is_moving = resp[4]

    stockfish.set_fen_position(fen)
    print(stockfish.get_evaluation())

    if is_moving == "True":
        print("Waiting for robot to stop moving...")
        time.sleep(1)
        continue

    # If the manager node was just initialized, do nothing
    if status == "init":
        time.sleep(1)
        continue
    # If the status is mate you won or lose, no more moves
    elif status == "mate" and point == "0":
        if current_side != param_side:
            print(30*"*")
            print("*          You won!          *")
            print(30*"*")
            break 
        else:
            # Only human player can lose here...
            print(30*"*")
            print("*         You lose!          *")
            print(30*"*")
            break

    # If status is centipawns the game can go on
    elif status == "cp":
        pass

    if current_side == param_side:
        print(f"Current point: {point}")

        if param_type == "human":
            move = player_step(fen)

        elif param_type == "stockfish":
            move = stockfish_step(fen)
        

        if move == None:
            print(30*"*")
            print("*         You lose!          *")
            print(30*"*")
            break

        chessgame.set_fen(fen)
        chessgame.apply_move(move)
        
        resp = make_movement_client(param_side, move, str(chessgame), robot_move)

        print_once_flag = True
    else:
        if print_once_flag:
            print("Waiting for other player...")
            print_once_flag = False


    time.sleep(1)