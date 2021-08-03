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

board = chess.Board()
FENs = ["1k1r4/pp1b1R2/3q2pp/4p3/2B5/4Q3/PPP2B2/2K5 b - - 0 1", "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", "rnbqkbnr/ppp1pppp/3p4/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"]

root = tk.Tk()

fig = plt.figure(1)

a = chess.svg.board(board, size=350)
b = cairosvg.svg2png(bytestring=a, write_to=None)
c = plt.imread(BytesIO(b))
plt.imshow(c)

canvas = FigureCanvasTkAgg(fig, master=root)
plot_widget = canvas.get_tk_widget()

def update():
    i = 0
    while True:
        board = chess.Board(FENs[i])
        squares = None#board.attacks(chess.A2)
        squares = chess.SquareSet([chess.A8, chess.A1])
        a = chess.svg.board(board, squares=squares, size=350, check=chess.B8, arrows=[chess.svg.Arrow(chess.E2, chess.F7),chess.svg.Arrow(chess.E8, chess.A4),chess.svg.Arrow(chess.B5, chess.B5),chess.svg.Arrow(chess.C6, chess.C6, color="red")])
        b = cairosvg.svg2png(bytestring=a, write_to=None)
        c = plt.imread(BytesIO(b))
        plt.imshow(c)

        fig.canvas.draw()

        text2_string.set(FENs[i])
        time.sleep(1)

        i = (i + 1) % len(FENs)

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

# We don't have to use rospy.spin() because tk's mainloop is blocking anyway:
# https://answers.ros.org/question/106781/rospy-and-tkinter-spin-and-mainloop/
root.mainloop()