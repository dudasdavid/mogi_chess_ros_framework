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