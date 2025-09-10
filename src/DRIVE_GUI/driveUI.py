import tkinter as tk
from DRIVE_GUI.roboticist import RoboticistMenu
from DRIVE_GUI.robot import RobotMenu

ROBOTICIST = "Roboticist"
ROBOT = "Robot"
FIELD_EXPERIMENT = "Ground"

TITLE = "Drive Menu"
SIZE_MENU = "700x600"
SIZE_SUBMENU = "500x500"


class MainMenu(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(TITLE)
        self.geometry(SIZE_MENU)

        menubar = tk.Menu(self)
        self.config(menu=menubar)

        submenu = tk.Menu(menubar, tearoff=0)
        submenu.add_command(label=ROBOTICIST, command=self.open_roboticist)
        submenu.add_command(label=ROBOT, command=self.open_robot)
        submenu.add_command(label=FIELD_EXPERIMENT, command=self.open_ground)

        menubar.add_cascade(label="Menu", menu=submenu)

    def open_roboticist(self):
        RoboticistMenu(self)

    def open_robot(self):
        RobotMenu(self)

    def open_ground(self):
        RoboticistMenu(self, FIELD_EXPERIMENT)


if __name__ == "__main__":
    app = MainMenu()
    app.mainloop()
