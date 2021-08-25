#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from Chessnut import Game
from stockfish import Stockfish
import rospkg
import time
import fen_parser
from mogi_chess_msgs.srv import ReadStatus, ReadStatusResponse, MakeMovement, MakeMovementResponse, RobotCommand, RobotStatus, SaveFenSamples, MakeInvalidMovement, MakeInvalidMovementResponse

def send_robot_command_client(command):
    rospy.wait_for_service('robot_command')
    try:
        robot_command_service = rospy.ServiceProxy('robot_command', RobotCommand)
        resp1 = robot_command_service(command)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def read_robot_status_client():
    rospy.wait_for_service('robot_status')
    try:
        robot_status_service = rospy.ServiceProxy('robot_status', RobotStatus)
        resp1 = robot_status_service()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def save_fen_samples_client(fen):
    rospy.wait_for_service('save_fen_samples')
    try:
        save_fen_samples_service = rospy.ServiceProxy('save_fen_samples', SaveFenSamples)
        resp1 = save_fen_samples_service(fen)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def calculate_robot_request(fen, move, side):
    global hit_slot, hit_list

    moved_piece = fen_parser.get_piece(fen, move[:2])
    target_piece = fen_parser.get_piece(fen, move[2:4])
    print("Piece moved:", moved_piece)
    print("Target piece:", target_piece)

    # Detect if king was moved
    if moved_piece in "kK":
        print("King moved!")
        if move[0] == "e" and move[2] == "g":
            print("Castling happened to the king side!")
            movement_str = f"{side};c;{move[:2]};{move[2:4]};h{move[1]};f{move[1]}"
            return movement_str
        if move[0] == "e" and move[2] == "c":
            print("Castling happened to the queen side!")
            movement_str = f"{side};c;{move[:2]};{move[2:4]};a{move[1]};d{move[1]}"
            return movement_str

    # Detect promotion
    if len(move) == 5:
        if moved_piece in "pP":

            if moved_piece == "p":
                requested_piece = move[4]
            else:
                requested_piece = move[4].upper()

            print(f"Promotion happened! Requested piece: {requested_piece}, available pieces in hit list:")
            print(hit_list)

            # if requested piece is not available in hit list
            if requested_piece not in hit_list:
                print(f"Requested piece is not available in hit list! Please put a {requested_piece} to {move[2:4]}")
                movement_str = f"{side};px;{move[:2]};{hit_slot}"
                hit_slot += 1
                hit_list.append(moved_piece)
                return movement_str

            else:
                print("Requested piece is available in hit list!")
                if moved_piece == "p":
                    promotion_slot = hit_list.index(move[4])
                    movement_str = f"{side};p;{promotion_slot};{move[2:4]};{move[:2]};{promotion_slot}"
                    hit_list[promotion_slot] = "p"
                    return movement_str

                elif moved_piece == "P":
                    promotion_slot = hit_list.index(move[4].upper())
                    movement_str = f"{side};p;{promotion_slot};{move[2:4]};{move[:2]};{promotion_slot}"
                    hit_list[promotion_slot] = "P"
                    return movement_str

        else:
            print(f"ERROR: {move} is 5 character long but moved pieces is: {moved_piece} not p or P!")
            return None

    # Detect en passant -- hit will happen!
    if moved_piece in "pP":
        # TODO: can we improve en passant logic???
        if move[0] != move[2] and target_piece == "-":
            print("En passant happened!")
            if moved_piece == "p":
                movement_str = f"{side};e;{move[:2]};{move[2:4]};{move[2]}{int(move[3])+1};{hit_slot}"
                hit_list.append("P")
            if moved_piece == "P":
                movement_str = f"{side};e;{move[:2]};{move[2:4]};{move[2]}{int(move[3])-1};{hit_slot}"
                hit_list.append("p")
            hit_slot += 1
            return movement_str

    # Detect normal hit:
    if target_piece != "-":
        print("Hit happened!")
        low_move, orientation = fen_parser.can_move_low(fen, move[:2], move[2:4])
        if low_move:
            movement_str = f"{side};xl;{move[2:4]};{hit_slot};{move[:2]};{move[2:4]};{orientation}"
        else:
            movement_str = f"{side};x;{move[2:4]};{hit_slot};{move[:2]};{move[2:4]}"
        hit_slot += 1
        hit_list.append(target_piece)
        return movement_str

    # If none of above, it's just a normal step
    low_move, orientation = fen_parser.can_move_low(fen, move[:2], move[2:4])
    if low_move:
        movement_str = f"{side};nl;{move[:2]};{move[2:4]};{orientation}"
    else:
        movement_str = f"{side};n;{move[:2]};{move[2:4]}"
    return movement_str

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

            fen_str.data = current_fen
            fen_pub.publish(fen_str)

            if req.robot:
                robot_is_moving = True
                # this has to detect hits and special moves, too!
                robot_req = calculate_robot_request(previous_fen, req.movement, req.player)
                print(robot_req)
                success = send_robot_command_client(robot_req).success
                if success:
                    print("Robot movement was successful")
                else:
                    print("Robot movement was NOT successful")

            
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

def serve_invalid_movement(req):

    robot_req = "b;invalid"
    success = send_robot_command_client(robot_req).success

    if success:
        print("Robot movement was successful")
    else:
        print("Robot movement was NOT successful")

    return MakeInvalidMovementResponse(success)

def serve_read_status(req):
    global robot_is_moving, robot_moving_timeout

    robot_is_moving = read_robot_status_client().is_moving

    if robot_is_moving == False and param_save:
        ret = save_fen_samples_client(current_fen)
        print(ret)

    response = "%s;%s;%s;%s;%s" % (status, point, current_side, current_fen, str(robot_is_moving))

    return ReadStatusResponse(response)

valid_step = False
error_string = "not_initialized"
status = "init"
point = "0"
current_side = "none"
current_fen = "none"
robot_is_moving = True
robot_moving_timeout = 0
hit_slot = 0
hit_list = []

# Set up ROS stuff
status_pub = rospy.Publisher('chess_status', String, queue_size=1)
# set latch flag to true, to make sure that the initial FEN is available for subscribers at start
fen_pub = rospy.Publisher('chess_manager/fen', String, queue_size=1, latch=True)
s_read = rospy.Service('read_status', ReadStatus, serve_read_status)
s_move = rospy.Service('make_movement', MakeMovement, serve_movement)
s_invalid_move = rospy.Service('make_invalid_movement', MakeInvalidMovement, serve_invalid_movement)
rospy.init_node('chess_manager')

param_save = rospy.get_param('~save', "false")

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

fen_str = String()
fen_str.data = current_fen
fen_pub.publish(fen_str)

rospy.spin()
