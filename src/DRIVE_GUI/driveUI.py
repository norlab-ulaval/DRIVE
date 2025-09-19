import tkinter as tk
from tkinter import messagebox
import customtkinter as ctk
from DRIVE_GUI.roboticist import RoboticistMenu
from DRIVE_GUI.robot import RobotMenu
from DRIVE_GUI.ground import GroundMenu
from DRIVE_GUI.utils import Utils

ROBOTICISTS_DATA_FILE = "./Experience/roboticists.json"
ROBOT_DATA_FILE = "./Experience/robot.json"
GROUND_DATA_FILE = "./Experience/ground.json"

class HomePage(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("DRIVE Protocol Home")
        self.geometry("800x500")
        
        instructions = (
            "To use the DRIVE protocol, you must fill the Roboticist, Robot, and Terrain forms.\n\n"

            "Once all forms are filled, select each and click 'Launch Drive' to start your experiment."
        )
        ctk.CTkLabel(self, text="DRIVE Protocol", font=("Arial", 24, "bold")).pack(pady=10)
        ctk.CTkLabel(self, text=instructions, font=("Arial", 15), wraplength=650, justify="left").pack(pady=5)

        #load data already filled
        self.utils = Utils()
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE) or []
        self.robots = self.utils.load_file(ROBOT_DATA_FILE) or []
        self.grounds = self.utils.load_file(GROUND_DATA_FILE) or []

        # Frame configuration
        select_frame = ctk.CTkFrame(self)
        select_frame.pack(pady=70, padx=10, expand=True)
        select_frame.grid_columnconfigure(1, weight=2)
        select_frame.grid_rowconfigure(0, weight=2)
        select_frame.grid_rowconfigure(1, weight=2)
        select_frame.grid_columnconfigure(0, minsize=120)

        # Roboticists
        ctk.CTkLabel(select_frame, text="Roboticist:", font=("Arial", 14)).grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        self.combo_roboticist = ctk.CTkComboBox(
            select_frame,
            values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
        )
        self.combo_roboticist.grid(row=0, column=1, padx=10, pady=10)
        ctk.CTkButton(
            select_frame,
            text="Add Roboticist",
            width=120,
            anchor="center",
            command=self.open_roboticist
        ).grid(row=0, column=2, padx=10, pady=10)
        if not self.roboticists:
            self.combo_roboticist.set("")

        # Robot
        ctk.CTkLabel(select_frame, text="Robot:", font=("Arial", 14)).grid(row=1, column=0, sticky="ew", padx=10, pady=10)
        self.combo_robot = ctk.CTkComboBox(
            select_frame,
            values=[f"{r['robot']} (v{r['version']})" for r in self.robots]
        )
        self.combo_robot.grid(row=1, column=1, padx=10, pady=10)
        ctk.CTkButton(
            select_frame,
            text="Add Robot",
            width=120,
            anchor="center",            command=self.open_robot
        ).grid(row=1, column=2, padx=10, pady=10)
        
        if not self.robots:
            self.combo_robot.set("")

        # Ground 
        ctk.CTkLabel(select_frame, text="Terrain:", font=("Arial", 14)).grid(row=2, column=0, sticky="ew", padx=10, pady=10)
        self.combo_terrain = ctk.CTkComboBox(
            select_frame,
            values=[g.get("name", "Unnamed") for g in self.grounds]
        )
        self.combo_terrain.grid(row=2, column=1, padx=10, pady=10)
        ctk.CTkButton(
            select_frame,
            text="Add Ground",
            width=120,
            anchor="center",
            command=self.open_ground
        ).grid(row=2, column=2, padx=10, pady=10)
        if not self.grounds:
            self.combo_terrain.set("")


        # Launch_Drive
        ctk.CTkButton(
            self,
            text="Launch Drive",
            width=200,
            fg_color="#4EC23C",
            hover_color="#4F8C46",
            font=("Arial", 16, "bold"),
            command=self.launch_drive).pack(pady=30)

    def open_roboticist(self):
        RoboticistMenu(self)
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE)
        self.combo_roboticist.configure(values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists])

    def open_robot(self):
        RobotMenu(self)
        self.robots = self.utils.load_file(ROBOT_DATA_FILE)
        self.combo_robot.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.robots])

    def open_ground(self):
        GroundMenu(self)
        self.grounds = self.utils.load_file(GROUND_DATA_FILE) or []
        self.combo_robot.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.grounds])

    def launch_drive(self):
        if not self.combo_roboticist.get() or not self.combo_robot:
            messagebox.showerror("Error", "Please select a roboticist and robot before launching DRIVE")
            return
        messagebox.showinfo("Launch DRIVE", "DRIVE experiment would be launched here.")

if __name__ == "__main__":
    app = HomePage()
    app.mainloop()