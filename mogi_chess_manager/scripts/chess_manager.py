#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from Chessnut import Game
from stockfish import Stockfish
import rospkg
import time
import fen_parser
from mogi_chess_msgs.srv import ReadStatus, ReadStatusResponse, MakeMovement, MakeMovementResponse

def calculate_robot_request(fen, move):

    # Detect if king was moved
    moved_piece = fen_parser.get_piece(fen, move[:2])
    print("Piece moved:", moved_piece)
    if moved_piece in "kK":
        print("King moved!")
        if move[0] == "e" and move[2] == "g":
            print("Castling happened to the king side!")
        if move[0] == "e" and move[2] == "c":
            print("Castling happened to the queen side!")

    # Detect promotion
    # TODO

    # Detect en passant -- hit will happen!
    # TODO

    # Detect normal hit:
    target_piece = fen_parser.get_piece(fen, move[2:4])
    print("Target piece:", target_piece)
    if target_piece != "-":
        print("Hit happened!")

    # If none of above, it's just a normal step

def serve_movement(req):
    global current_side, point, status, current_fen, current_side, robot_is_moving

    print(req)

    if req.player == current_side:

        if stockfish.is_move_correct(req.movement):
            previous_fen = current_fen
            stockfish.set_fen_position(req.fen)
            print(stockfish.get_board_visual())
            error_string = "no_error"
            valid_step = True
            eval = stockfish.get_evaluation()
            point = eval['value']
            status = eval['type']
            current_fen = stockfish.get_fen_position()
            current_side = current_fen.split(' ')[1]

            if req.robot:
                robot_is_moving = True
                # this has to detect hits and special moves, too!
                calculate_robot_request(previous_fen, req.movement)

            
        else:
            error_string = f"invalid_movement_{req.movement}"
            print(error_string)
            print(current_fen)
            print(req.fen)
            valid_step = False

            # TODO: if step is invalid and it wasn't carried out by the robot, the clock must reset, and maybe the robot should put the oppontnt's piece back...

    else:
        error_string = "wrong_player"
        print("wrong_player")
        valid_step = False

    return MakeMovementResponse(valid_step)


def serve_read_status(req):
    global robot_is_moving, robot_moving_timeout
    response = "%s;%s;%s;%s;%s" % (status, point, current_side, current_fen, str(robot_is_moving))

    # Fake robot is moving countdown, this has to be replaced by input from real robot
    if robot_is_moving:
        robot_moving_timeout += 1
        if robot_moving_timeout > 1:
            robot_is_moving = False
            robot_moving_timeout = 0

    return ReadStatusResponse(response)

valid_step = False
error_string = "not_initialized"
status = "init"
point = "0"
current_side = "none"
current_fen = "none"
robot_is_moving = False
robot_moving_timeout = 0

# Set up ROS stuff
status_pub = rospy.Publisher('chess_status', String, queue_size=1)
s_read = rospy.Service('read_status', ReadStatus, serve_read_status)
s_move = rospy.Service('make_movement', MakeMovement, serve_movement)
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
eval = stockfish.get_evaluation()
point = eval['value']
status = eval['type']
current_fen = stockfish.get_fen_position()
current_side = current_fen.split(' ')[1]

rospy.spin()
