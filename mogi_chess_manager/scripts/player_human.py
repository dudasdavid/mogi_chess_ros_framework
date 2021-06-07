#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from Chessnut import Game
from stockfish import Stockfish
import rospkg
import time
from mogi_chess_msgs.srv import ReadStatus

def chess_status_callback(data):
    print(data)

def read_status_client():
    rospy.wait_for_service('read_status')
    try:
        read_status_service = rospy.ServiceProxy('read_status', ReadStatus)
        resp1 = read_status_service()
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Set up ROS stuff
player_pub = rospy.Publisher('chess_player_step', String, queue_size=1)
#rospy.Subscriber("chess_status", String, chess_status_callback)

rospy.init_node('chess_player_human')

rospack = rospkg.RosPack()
path = rospack.get_path('mogi_chess_stockfish')
stockfish_path = path + "/stockfish/stockfish_13_linux_x64_bmi2"
print(stockfish_path)
stockfish = Stockfish(stockfish_path, parameters={"Threads": 2, "Minimum Thinking Time": 30})
stockfish.set_skill_level(20)
print(f"Using Stockfish v{stockfish.get_stockfish_major_version()}, skill level: {stockfish.get_parameters()['Skill Level']}")

chessgame = Game()

print_once_flag = True

param_side = rospy.get_param('~side', "w")

while not rospy.is_shutdown():
    resp = read_status_client().split(";")
    #print(resp)
    status = resp[0]
    point = resp[1]
    current_side = resp[2]
    fen = resp[3]

    # If the manager node was just initialized, do nothing
    if status == "init":
        continue
    # If the status is mate you won or lose, no more moves
    elif status == "mate":
        if current_side == param_side:
            print("You lose!")
            break
    # If status is centipawns the game can go on
    elif status == "cp":
        pass

    if current_side == param_side:
        print(f"Current point: {point}")
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

        chessgame.set_fen(fen)
        chessgame.apply_move(ret)
        pub_str = "%s;%s" % (param_side, str(chessgame))
        player_pub.publish(pub_str)

        print_once_flag = True
    else:
        if print_once_flag:
            print("Waiting for other player...")
            print_once_flag = False




    time.sleep(1)