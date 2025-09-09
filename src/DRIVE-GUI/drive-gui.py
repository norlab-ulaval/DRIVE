import tkinter as tk
from tkinter import ttk

import os


ROBOTICIST = "Roboticist"
ROBOT = "Robot"
FIELD_EXPERIMENT = "GROUND"

BG_COLOR = "#0D1164"
TITLE = "Drive Menu"
SIZE_MENU = "700x600"
SIZE_SUBMENU = "300x200"

ROBOTICISTS_DATA_FILE = "roboticists.json"
DEFAULT_ROBOTICISTS = [
    {
        "Name": "Alice",
        "Lastname": "Smith",
        "contact email": "alice@example.com",
        "Organization": "Robotics Lab"
    },
    {
        "Name": "Bob",
        "Lastname": "Jones",
        "contact email": "bob@example.com",
        "Organization": "AI Institute"
    }
]


class MainMenu(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(TITLE)
        self.geometry(SIZE_MENU)

        menubar = tk.Menu(self)
        self.config(menu=menubar, bg=BG_COLOR)

        submenu = tk.Menu(menubar, tearoff=0)
        submenu.add_command(label=ROBOTICIST, command=self.open_roboticist)
        submenu.add_command(label=ROBOT, command=self.open_robot)
        submenu.add_command(label=FIELD_EXPERIMENT, command=self.open_field)

        menubar.add_cascade(label="Menu", menu=submenu)

    def open_roboticist(self):
        SubMenu(self, ROBOTICIST)

    def open_robot(self):
        SubMenu(self, ROBOT)

    def open_field(self):
        SubMenu(self, FIELD_EXPERIMENT)


class SubMenu(tk.Toplevel):
    def __init__(self, parent, title, questions):
        super().__init__(parent)
        self.title(title)
        self.questions(questions)
        self.geometry(SIZE_SUBMENU)
        ttk.Label(self, text=f"{title} Menu", font=("Arial", 12)).pack(pady=20)
        ttk.Button(self, text="Close", command=self.destroy).pack()


if __name__ == "__main__":
    app = MainMenu()
    app.mainloop()
