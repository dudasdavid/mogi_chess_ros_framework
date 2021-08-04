#!/usr/bin/env python3.8

from Chessnut import Game

def get_fen(input_array):
    fen_out = ""
    for i, item in enumerate(input_array):
        if i % 8 == 0 and i != 0:
            fen_out += "/"
        if item == "0":
            if len(fen_out) == 0:
                fen_out += "1"
            elif fen_out[-1].isnumeric():
                fen_out = fen_out[:-1] + str(int(fen_out[-1]) + 1)
            else:
                fen_out += "1"
        else:
            fen_out += item

    return fen_out

def get_list_fom_fen(fen):
    out_list = []

    fen_list = fen.split(" ")[0].split("/")

    for i, rank in enumerate(fen_list):
        for j, char in enumerate(rank):
            if char.isnumeric():
                for k in range(0, int(char)):
                    out_list.append("empty")
            else:
                out_list.append(char)

    return out_list

def get_king(fen, side):
    fen = fen.split(" ")[0]

    if side == "w":
        king = "K"
    elif side == "b":
        king = "k"
    else:
        print("INVALID SIDE")
        return None

    ranks = fen.split("/")
    for i, r in enumerate(ranks):
        if king in r:
            j = 0
            for char in r:
                if char.isnumeric():
                    j += int(char)
                elif char == king:
                    #print(chr(j + 97) + str(8-i))
                    return chr(j + 97) + str(8-i)
                else:
                    j += 1


def label2class(label_text):
    if label_text == 'empty':
        class_num = 0
    elif label_text == 'b_p':
        class_num = 1
    elif label_text == 'b_r':
        class_num = 2
    elif label_text == 'b_n':
        class_num = 3
    elif label_text == 'b_b':
        class_num = 4
    elif label_text == 'b_k':
        class_num = 5
    elif label_text == 'b_q':
        class_num = 6
    elif label_text == 'w_P':
        class_num = 7
    elif label_text == 'w_R':
        class_num = 8
    elif label_text == 'w_N':
        class_num = 9
    elif label_text == 'w_B':
        class_num = 10
    elif label_text == 'w_K':
        class_num = 11
    elif label_text == 'w_Q':
        class_num = 12
    else:
        raise ValueError('Unknown class: %s!' % label_text)

    return class_num

def class2label(class_num):
    if class_num == 0:
        label_text = "Empty"
        label_short = "0"
    elif class_num == 1:
        label_text = "Black pawn"
        label_short = "p"
    elif class_num == 2:
        label_text = "Black rook"
        label_short = "r"
    elif class_num == 3:
        label_text = "Black knight"
        label_short = "n"
    elif class_num == 4:
        label_text = "Black bishop"
        label_short = "b"
    elif class_num == 5:
        label_text = "Black king"
        label_short = "k"
    elif class_num == 6:
        label_text = "Black queen"
        label_short = "q"
    elif class_num == 7:
        label_text = "White pawn"
        label_short = "P"
    elif class_num == 8:
        label_text = "White rook"
        label_short = "R"
    elif class_num == 9:
        label_text = "White knight"
        label_short = "N"
    elif class_num == 10:
        label_text = "White bishop"
        label_short = "B"
    elif class_num == 11:
        label_text = "White king"
        label_short = "K"
    elif class_num == 12:
        label_text = "White queen"
        label_short = "Q"
    else:
        raise ValueError('Unknown class: %s!' % class_num)

    return label_text, label_short

def short2label(short_name):
    if short_name == "empty":
        label = "empty"
    elif short_name == "p":
        label = "Black pawn"
    elif short_name == "r":
        label = "Black rook"
    elif short_name == "n":
        label = "Black knight"
    elif short_name == "b":
        label = "Black bishop"
    elif short_name == "k":
        label = "Black king"
    elif short_name == "q":
        label = "Black queen"
    elif short_name == "P":
        label = "White pawn"
    elif short_name == "R":
        label = "White rook"
    elif short_name == "N":
        label = "White knight"
    elif short_name == "B":
        label = "White bishop"
    elif short_name == "K":
        label = "White king"
    elif short_name == "Q":
        label = "White queen"
    else:
        raise ValueError('Unknown short: %s!' % short_name)

    return label

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

def get_piece_ij(fen, i, j):
    # each piece is identified by a single letter taken from the standard English names
    # (pawn = "P", knight = "N", bishop = "B", rook = "R", queen = "Q" and king = "K").
    # White pieces are designated using upper-case letters ("PNBRQK")
    # while black pieces use lowercase ("pnbrqk").

    rank_number = i
    column_number = j

    assert rank_number < 8
    assert column_number < 8

    ranks = fen.split("/")
    selected_rank = ranks[rank_number]

    scanned_column = 0
    for i in selected_rank:
        #print("current scan: %d" % scanned_column)
        if i.isnumeric():
            scanned_column += int(i)
        else:
            if scanned_column == column_number:
                #print(i)
                if i.islower():
                    return "b_" + i
                else:
                    return "w_" + i
            scanned_column += 1
        if scanned_column > column_number:
            #print("empty field")
            return "empty"

    raise ValueError("Couldn't find the piece!")

def get_empty_fields(fen):
    empty_fields = []

    fen_list = fen.split("/")
    fen_list.reverse()

    for i, rank in enumerate(fen_list):
        rank_char = str(i + 1)
        scanned_column = 1
        for j, char in enumerate(rank):
            if char.isnumeric():
                for k in range(scanned_column, scanned_column + int(char)):
                    empty_fields.append(chr(k + 96) + rank_char)
                scanned_column += int(char)
            else:
                scanned_column += 1

    return empty_fields

def get_side_fields(fen, side = "w"):
    side_fields = []

    fen_list = fen.split("/")
    fen_list.reverse()

    for i, rank in enumerate(fen_list):
        rank_char = str(i + 1)
        scanned_column = 1
        for j, char in enumerate(rank):
            if char.isnumeric():
                scanned_column += int(char)
            else:
                if side == "w":
                    if char.isupper():
                        side_fields.append(chr(scanned_column + 96) + rank_char)
                elif side == "b":
                    if char.islower():
                        side_fields.append(chr(scanned_column + 96) + rank_char)
                else:
                    print(f"ERROR: invalid side {side}")
                scanned_column += 1

    return side_fields


def track_fen(prev_fen, new_guess):
    # new guess is used only to find new empty and occupied fields.

    prev_fen_split = prev_fen.split(" ")
    fen_side = prev_fen_split[1]

    prev_empty_fields = get_empty_fields(prev_fen_split[0])
    new_empty_fields = get_empty_fields(new_guess)

    print(prev_empty_fields)
    print(new_empty_fields)

    prev_side_fields = get_side_fields(prev_fen_split[0], fen_side)
    new_side_fields = get_side_fields(new_guess, fen_side)

    new_empty = []
    new_occupied = []

    for item in prev_empty_fields:
        if item in new_empty_fields:
            idx = new_empty_fields.index(item)
            new_empty_fields.pop(idx)
        else:
            new_occupied.append(item)

    new_empty = new_empty_fields

    print(f"New occupied fields {new_occupied}")
    print(f"New empty fields {new_empty}")

    side_appeared = []
    side_disappeared = []

    for item in prev_side_fields:
        if item in new_side_fields:
            idx = new_side_fields.index(item)
            new_side_fields.pop(idx)
        else:
            side_disappeared.append(item)

    side_appeared = new_side_fields

    print(f"New {fen_side} fields {side_appeared}")
    print(f"Removed {fen_side} fields {side_disappeared}")

    for i in new_empty:
        piece = get_piece(prev_fen_split[0], i)
        print(f"Piece moved: {piece}")

    chessgame = Game()
    chessgame.set_fen(prev_fen)
    possible_moves = chessgame.get_moves()

    print(f"possible moves: {possible_moves}")

    if len(new_empty) == 0:
        if len(side_appeared) != 0 or len(side_disappeared) != 0:
            print(f"A {fen_side} piece moved from {side_disappeared} to {side_appeared} that was invalid!")
            return "invalid", "invalid"
        else:    
            # no movement happened, return previous FEN
            print("No movement happened")
            return prev_fen, "no_movement"
    elif len(new_empty) == 1:
        start = new_empty[0]
        # it can be a normal move, a hit or a promotion
        piece = get_piece(prev_fen_split[0], new_empty[0])
        if piece.islower():
            side = 'b'
        else:
            side = 'w'

        if piece == 'P' and int(new_empty[0][1]) == 7:
            print(f"Promotion happened, side {side} moved")
            end = new_occupied[0] + "q"

        elif piece == 'p' and int(new_empty[0][1]) == 2:
            print(f"Promotion happened, side {side} moved")
            end = new_occupied[0] + "q"

        elif len(new_occupied) == 0:
            if len(side_appeared) == 1:
                print(f"Hit happened, side {side} moved, possible moves:")
                for i in possible_moves:
                    if i[0:2] == new_empty[0]:
                        if i[2:4] == side_appeared[0]:
                            end = side_appeared[0]
                            print(f"{i} matches with new {fen_side} occupied {side_appeared}")
                            break
                        else:
                            print(f"{i} doesn't match with new {fen_side} occupied {side_appeared}")
                            end = "invalid_hit"
            elif len(side_appeared) == 0:
                print(f"A piece suddenly disappeared from {new_empty}, put it back now!")
                return "invalid", "invalid"

            else:
                print("Something unexpected happened here...")
            
        elif len(new_occupied) == 1:
            print(f"Normal move happened, side {side} moved")
            end = new_occupied[0]
        else:
            print(f"ERROR: 1 new empty fields, moved piece: {piece}!")
            return "invalid", "invalid"

    elif len(new_empty) == 2:
        # it can be castling or en passant
        piece1 = get_piece(prev_fen_split[0], new_empty[0])
        piece2 = get_piece(prev_fen_split[0], new_empty[1])

        if piece1 in 'pP' and piece2 in 'pP':
            
            # prev_fen_split[3] indicates if there is a possible en passant
            # e.g rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq e6 0 2
            # e6 is potential en passant
            if new_occupied[0] == prev_fen_split[3]:
                if int(new_empty[0][1]) == 4 and int(new_empty[1][2]) == 4:
                    side = 'b'
                    if piece1 == 'p':
                        start = new_empty[0]
                    else:
                        start = new_empty[1]

                elif int(new_empty[0][1]) == 5 and int(new_empty[1][2]) == 5:
                    side = 'w'
                    if piece1 == 'P':
                        start = new_empty[0]
                    else:
                        start = new_empty[1]

                else:
                    print(f"ERROR: invalid en passant! Fields {new_empty[0]} and {new_empty[1]} are empty, {new_occupied[0]} is occupied and possible en passant from FEN is {prev_fen_split[3]}!")

                end = new_occupied[0]
                print(f"En passant happened, side {side} moved")
            
            else:
                print(f"ERROR: some strange invalid en passant happened!")


        elif piece1 in 'kK' and piece2 in 'rR':
            start = new_empty[0]
            if piece1.islower():
                side = 'b'
            else:
                side = 'w'

            if new_empty[1][0] == "a":
                end = "c" + start[1]
            elif new_empty[1][0] == "h":
                end = "g" + start[1]
            else:
                print(f"ERROR: invalid castling, piece 2 moved from {new_empty[1]}!")
                return "invalid", "invalid"


            print(f"Castling happened, side {side} moved")
        elif piece2 in 'kK' and piece1 in 'rR':
            start = new_empty[1]
            if piece2.islower():
                side = 'b'
            else:
                side = 'w'

            if new_empty[0][0] == "a":
                end = "c" + start[1]
            elif new_empty[0][0] == "h":
                end = "g" + start[1]
            else:
                print(f"ERROR: invalid castling, piece 2 moved from {new_empty[1]}!")
                return "invalid", "invalid"

            print(f"Castling happened, side {side} moved")
        else:
            print(f"ERROR: 2 new empty fields, moved pieces: {piece1} and {piece2}!")
            return "invalid", "invalid"

    else:
        print(f"ERROR: {len(new_empty)} new empty fields!")
        return "invalid", "invalid"

    move = start + end
    print(f"selected movement: {move}")

    if move not in possible_moves:
        print(f"{move} is invalid, see list of valid moves! Set it back now!")

        return "invalid", "invalid"

    else:
        chessgame.apply_move(move)

        print(f"New FEN: {str(chessgame)}")

        return str(chessgame), move

