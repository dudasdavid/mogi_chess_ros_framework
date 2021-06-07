#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from Chessnut import Game
from stockfish import Stockfish
import rospkg
import time
import fen_parser
from mogi_chess_msgs.srv import ReadStatus,ReadStatusResponse

def player_step_callback(data):
    global valid_step, error_string
    (player, new_fen) = data.data.split(";")
    if player == current_side:
        move = fen_parser.fen_diff(current_fen, new_fen)
        if move != False:
            if stockfish.is_move_correct(move):
                stockfish.set_fen_position(new_fen)
                error_string = "no_error"
                valid_step = True
            else:
                error_string = "invalid_movement_stockfish"
                print("invalid_movement_stockfish:")
                print(current_fen)
                print(new_fen)
                valid_step = False

        else:
            error_string = "invalid_movement_fen_parser"
            print("invalid_movement_fen_parser:")
            print(current_fen)
            print(new_fen)
            valid_step = False
    else:
        error_string = "wrong_player"
        print("wrong_plaxer")
        valid_step = False

    waiting_for_next_step = False

def serve_read_status(req):
    response = "%s;%s;%s;%s" % (status, point, current_side, current_fen)
    return ReadStatusResponse(response)

valid_step = False
error_string = "not_initialized"
status = "init"
point = "0"
current_side = "none"
current_fen = "none"

# Set up ROS stuff
status_pub = rospy.Publisher('chess_status', String, queue_size=1)
rospy.Subscriber("chess_player_step", String, player_step_callback)
s = rospy.Service('read_status', ReadStatus, serve_read_status)
rospy.init_node('chess_manager')

fen_start = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

rospack = rospkg.RosPack()
path = rospack.get_path('mogi_chess_stockfish')
stockfish_path = path + "/stockfish/stockfish_13_linux_x64_bmi2"
print(stockfish_path)
stockfish = Stockfish(stockfish_path, parameters={"Threads": 2, "Minimum Thinking Time": 30})
stockfish.set_skill_level(20)
print(f"Using Stockfish v{stockfish.get_stockfish_major_version()}, skill level: {stockfish.get_parameters()['Skill Level']}")

stockfish.set_fen_position(fen_start)


chessgame = Game()

rate = rospy.Rate(1)


valid_step = True
error_string = "no_error"
while not rospy.is_shutdown():

    eval = stockfish.get_evaluation()
    point = eval['value']
    status = eval['type']
    current_fen = stockfish.get_fen_position()
    current_side = current_fen.split(' ')[1]
    

    #print(f"Valid: {valid_step}, Error: {error_string}, Status: {status}, point: {point}, player: {current_side}, fen: {current_fen}")
    pub_str = "%s;%s;%s;%s;%s;%s" % (valid_step, error_string, status, point, current_side, current_fen)
    status_pub.publish(pub_str)

    rate.sleep()

    #waiting_for_next_step = True
