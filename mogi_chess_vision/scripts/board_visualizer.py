#!/usr/bin/env python3.8

import numpy as np
import tkinter as tk

import matplotlib
matplotlib.use('TkAgg')

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

import _thread
import time
import chess
import chess.svg
from io import BytesIO
import cairosvg
import signal
import sys

import rospy
from std_msgs.msg import String

import helper_lib

def handle_fen(msg):
    global fen
    fen = msg.data

# test publisher:
# rostopic pub -1 /chess_manager/fen std_msgs/String "rnbqkbnr/ppp4/8/8/8/PP1PP1PP/RNBQKBNR w KQkq - 0 1"
rospy.init_node('board_visualizer')
fen_topic = "/chess_manager/fen"
rospy.Subscriber(fen_topic, String, handle_fen)

board = chess.Board()
fen = None
fen_prev = None

root = tk.Tk()

fig = plt.figure(1)

a = chess.svg.board(board, size=350)
b = cairosvg.svg2png(bytestring=a, write_to=None)
c = plt.imread(BytesIO(b))
plt.imshow(c)

canvas = FigureCanvasTkAgg(fig, master=root)
plot_widget = canvas.get_tk_widget()

exit_flag = True

def quit(sig = None, frame = None):
    global exit_flag
    print("exit")
    exit_flag = False
    root.quit()
    root.update()
    root.destroy()

def update():
    global fen, fen_prev
    while exit_flag:
        if fen != fen_prev:
            fen_prev = fen
            board = chess.Board(fen)

            side = fen.split(" ")[1]

            if board.is_check():
                print(f"Current side: {side} is in check!")
                check_square = helper_lib.get_king(fen, "w")
                check_square = chess.parse_square(check_square)
                print(check_square)
                a = chess.svg.board(board, size=350, check=check_square)
            else:
                a = chess.svg.board(board, size=350)


            #squares = None#board.attacks(chess.A2)
            #squares = chess.SquareSet([chess.A8, chess.A1])
            #a = chess.svg.board(board, squares=squares, size=350, check=chess.B8, arrows=[chess.svg.Arrow(chess.E2, chess.F7),chess.svg.Arrow(chess.E8, chess.A4),chess.svg.Arrow(chess.B5, chess.B5),chess.svg.Arrow(chess.C6, chess.C6, color="red")])
            
            b = cairosvg.svg2png(bytestring=a, write_to=None)
            c = plt.imread(BytesIO(b))
            plt.imshow(c)

            fig.canvas.draw_idle() # fix some funky segfault

            text2_string.set(fen)
            time.sleep(0.1)

        else:
            time.sleep(0.1)

    print("Thread stopped")

plot_widget.grid(row=0, column=0, rowspan = 2)

text2_string = tk.StringVar()
text2_string.set("test2")
text2 = tk.Label(root, width=50, textvariable=text2_string)
text2.grid(row=0, column=1)

text3_string = tk.StringVar()
text3_string.set("test3")
text3 = tk.Label(root, width=50, textvariable=text3_string)
text3.grid(row=1, column=1)

_thread.start_new_thread(update,())

# Set signal before starting
signal.signal(signal.SIGINT, quit)
root.protocol("WM_DELETE_WINDOW", quit)

# We don't have to use rospy.spin() because tk's mainloop is blocking anyway:
# https://answers.ros.org/question/106781/rospy-and-tkinter-spin-and-mainloop/
root.mainloop()
sys.exit()