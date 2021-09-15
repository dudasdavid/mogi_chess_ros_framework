#!/usr/bin/env python3.8

from Chessnut import Game

def get_piece(fen, field):
    # each piece is identified by a single letter taken from the standard English names
    # (pawn = "P", knight = "N", bishop = "B", rook = "R", queen = "Q" and king = "K").
    # White pieces are designated using upper-case letters ("PNBRQK")
    # while black pieces use lowercase ("pnbrqk").

    rank = int(field[1])
    column = field[0]
    #print(column)
    column_number = ord(column) - 97
    #print(column_number)

    assert rank < 9
    assert column_number < 8

    #print(fen)
    #print(field)
    ranks = fen.split(" ")[0].split("/")
    #print(ranks)
    selected_rank = ranks[8 - rank]
    #print(selected_rank)

    scanned_column = 0
    for i in selected_rank:
        #print("current scan: %d" % scanned_column)
        if i.isnumeric():
            scanned_column += int(i)
        else:
            if scanned_column == column_number:
                #print(i)
                return i
            scanned_column += 1
        if scanned_column > column_number:
            #print("empty field")
            return "-"

    raise ValueError("Couldn't find the piece!")

def fen_diff(fen_prev, fen_next):
    chessgame = Game()
    chessgame.set_fen(fen_prev)
    moves = chessgame.get_moves()
    #print(moves)

    valid = False
    for move in moves:
        chessgame.set_fen(fen_prev)
        chessgame.apply_move(move)
        #print(20*"=")
        #print(str(chessgame))
        #print(fen_next)
        if str(chessgame) == fen_next:
            valid = move
            break

    return valid

def can_move_low(fen, start, end):
    print(f"Decide if low movement is possible! Start: {start}, end: {end}")

    # detect vertical movement, e.g. e2 -> e4
    if start[0] == end[0]:
        print("Vertical movement detected!")
        if abs(int(start[1]) - int(end[1])) == 1:
            print("Only one square movement, low movement is possible!")
            # low movement is possible, desired orientation is 0 degree
            return True, 0

        elif int(start[1]) < int(end[1]):
            print(f"Rank of start {start} is lower than end {end}")
            is_empty = True
            for i in range(int(start[1]) + 1, int(end[1])):
                if get_piece(fen, start[0] + str(i)) != "-":
                    is_empty = False

        elif int(start[1]) > int(end[1]):
            print(f"Rank of start {start} is higher than end {end}")
            is_empty = True
            for i in range(int(end[1]) + 1, int(start[1])):
                if get_piece(fen, start[0] + str(i)) != "-":
                    is_empty = False

        else:
            print("Vertical low movement detection error!")
            return False, 45

        if is_empty:
            return True, 0
        else:
            return False, 45

    # detect horizontal movement, e.g. a1 -> c1
    elif start[1] == end[1]:
        print("Horizontal movement detected!")
        if abs(ord(start[0]) - ord(end[0])) == 1:
            print("Only one square movement, low movement is possible!")
            return True, 90

        elif ord(start[0]) < ord(end[0]):
            print(f"Column of start {start} is lower than end {end}")
            is_empty = True
            for i in range(ord(start[0]) + 1, ord(end[0])):
                if get_piece(fen, chr(i) + start[1]) != "-":
                    is_empty = False

        elif ord(start[0]) > ord(end[0]):
            print(f"Column of start {start} is higher than end {end}")
            is_empty = True
            for i in range(ord(end[0]) + 1, ord(start[0])):
                if get_piece(fen, chr(i) + start[1]) != "-":
                    is_empty = False

        else:
            print("Horizontal low movement detection error!")
            return False, 45

        if is_empty:
            return True, 90
        else:
            return False, 45

    # detect diagonal movement increasing to the right, e.g. a1 -> c3 or h5 -> f3
    elif ord(start[0]) - int(start[1]) == ord(end[0]) - int(end[1]):
        print("Diagonal movement to the right detected!")
        if abs(ord(start[0]) - ord(end[0])) == 1:
            print("Only one square movement, low movement is possible!")
            return True, 135

        elif ord(start[0]) < ord(end[0]):
            print(f"Column of start {start} is lower than end {end}")
            is_empty = True
            j = 1
            for i in range(ord(start[0]) + 1, ord(end[0])):
                print(f"Checking square: {chr(i)}{str(int(start[1]) + j)}, result: {get_piece(fen, chr(i) + str(int(start[1]) + j))}")
                if get_piece(fen, chr(i) + str(int(start[1]) + j)) != "-":
                    is_empty = False
                j +=1

        elif ord(start[0]) > ord(end[0]):
            print(f"Column of start {start} is higher than end {end}")
            is_empty = True
            j = 1
            for i in range(ord(end[0]) + 1, ord(start[0])):
                print(f"Checking square: {chr(i)}{str(int(end[1]) + j)}, result: {get_piece(fen, chr(i) + str(int(end[1]) + j))}")
                if get_piece(fen, chr(i) + str(int(end[1]) + j)) != "-":
                    is_empty = False
                j +=1

        else:
            print("Diagonal right movement detection error!")
            return False, 45

        if is_empty:
            return True, 135
        else:
            return False, 45

    # detect diagonal movement increasing to the left, e.g. a8 -> g2 or c5 -> a7
    elif ord(start[0]) + int(start[1]) == ord(end[0]) + int(end[1]):
        print("Diagonal movement to the left detected!")
        if abs(ord(start[0]) - ord(end[0])) == 1:
            print("Only one square movement, low movement is possible!")
            return True, 45

        elif ord(start[0]) < ord(end[0]):
            print(f"Column of start {start} is lower than end {end}")
            is_empty = True
            j = 1
            for i in range(ord(start[0]) + 1, ord(end[0])):
                print(f"Checking square: {chr(i)}{str(int(start[1]) - j)}, result: {get_piece(fen, chr(i) + str(int(start[1]) - j))}")
                if get_piece(fen, chr(i) + str(int(start[1]) - j)) != "-":
                    is_empty = False
                j +=1

        elif ord(start[0]) > ord(end[0]):
            print(f"Column of start {start} is higher than end {end}")
            is_empty = True
            j = 1
            for i in range(ord(end[0]) + 1, ord(start[0])):
                print(f"Checking square: {chr(i)}{str(int(end[1]) - j)}, result: {get_piece(fen, chr(i) + str(int(end[1]) - j))}")
                if get_piece(fen, chr(i) + str(int(end[1]) - j)) != "-":
                    is_empty = False
                j +=1

        else:
            print("Diagonal right movement detection error!")
            return False, 45

        if is_empty:
            return True, 45
        else:
            return False, 45

    else:
        print("Low movement is not possible!")

    return False, 45

