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
        a = chess.svg.board(board, size=350)
        b = cairosvg.svg2png(bytestring=a, write_to=None)
        c = plt.imread(BytesIO(b))
        plt.imshow(c)

        fig.canvas.draw()
        time.sleep(1)

        i = (i + 1) % 3

plot_widget.grid(row=0, column=0)

_thread.start_new_thread(update,())

root.mainloop()