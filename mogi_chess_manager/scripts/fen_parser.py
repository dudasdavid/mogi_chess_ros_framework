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

