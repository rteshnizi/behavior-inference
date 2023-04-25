import tkinter as tk

class Canvas(object):
	def __init__(self, master, app, row, col):
		self.master = master
		self.app = app
		self.tkCanvas = tk.Canvas(master=master)
		self.tkCanvas.grid(row=row, column=col, columnspan=9, sticky=tk.N + tk.S + tk.E + tk.W)
