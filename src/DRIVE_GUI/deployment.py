import tkinter as tk
import customtkinter as ctk
from tkinter import messagebox
from DRIVE_GUI.roboticist import RoboticistMenu
from DRIVE_GUI.robot import RobotMenu
from DRIVE_GUI.field import FieldMenu
from DRIVE_GUI.utils import Utils
from DRIVE_GUI.tooltip import add_tooltip, create_info_icon

ROBOTICISTS_DATA_FILE = "./Experience/roboticists.json"
ROBOT_DATA_FILE = "./Experience/robot.json"
FIELD_DATA_FILE = "./Experience/field.json"


class Deployment(ctk.CTk):
    def __init__(self, allow_add=True, allow_edit=True):
        super().__init__()
        self.title("DEPLOYMENT")
        self.geometry("900x800")
        self.resizable(True, True)

        self.allow_add = allow_add
        self.allow_edit = allow_edit

        self.utils = Utils()
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE)
        self.robots = self.utils.load_file(ROBOT_DATA_FILE)
        self.fields = self.utils.load_file(FIELD_DATA_FILE)

        ctk.CTkLabel(self, text="DEPLOYMENT", font=("Arial", 20, "bold")).pack(pady=20)

        # DRIVE
        experiment_frame = ctk.CTkFrame(self)
        experiment_frame.pack(pady=20, padx=40, fill="both", expand=True)

        experiment_title = ctk.CTkLabel(experiment_frame, text="Using DRIVE Protocol", font=("Arial", 20, "bold"))
        experiment_title.pack(pady=10)

        # Frame configuration
        select_frame = ctk.CTkFrame(experiment_frame)
        select_frame.pack(pady=20, padx=20, fill="x")
        select_frame.grid_columnconfigure(0, minsize=120)
        select_frame.grid_columnconfigure(1, weight=1)

        # Roboticist
        ctk.CTkLabel(select_frame, text="Roboticist:", font=("Arial", 14)).grid(
            row=0, column=0, sticky="w", padx=10, pady=10
        )
        self.combo_roboticist = ctk.CTkComboBox(
            select_frame,
            values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists],
            fg_color="#cccccc",
            width=250,
            height=35,
        )
        self.combo_roboticist.grid(row=0, column=1, padx=2, pady=10, sticky="ew")

        self.create_action_buttons(select_frame, 0, self.open_roboticist)
        if not self.roboticists:
            self.combo_roboticist.set("")

        # Robot
        ctk.CTkLabel(select_frame, text="Robot:", font=("Arial", 14)).grid(
            row=1, column=0, sticky="w", padx=10, pady=10
        )
        self.combo_robot = ctk.CTkComboBox(
            select_frame,
            values=[f"{r['robot']} (v{r['version']})" for r in self.robots],
            fg_color="#cccccc",
            width=250,
            height=35,
        )
        self.combo_robot.grid(row=1, column=1, padx=2, pady=10, sticky="ew")

        self.create_action_buttons(select_frame, 1, self.open_robot)
        if not self.robots:
            self.combo_robot.set("")

        # Terrain
        ctk.CTkLabel(select_frame, text="Terrain:", font=("Arial", 14)).grid(
            row=2, column=0, sticky="w", padx=10, pady=10
        )
        self.combo_terrain = ctk.CTkComboBox(
            select_frame,
            values=[g.get("name", "Unnamed") for g in self.fields],
            fg_color="#cccccc",
            width=250,
            height=35,
        )
        self.combo_terrain.grid(row=2, column=1, padx=2, pady=10, sticky="ew")
        ctk.CTkButton(select_frame, text="Fill Terrain", width=120, anchor="center", command=self.open_terrain).grid(
            row=2, column=2, padx=10, pady=10
        )
        if not self.fields:
            self.combo_terrain.set("")

        # Section des boutons d'action
        action_buttons_frame = ctk.CTkFrame(self, fg_color="transparent")
        action_buttons_frame.pack(pady=20)

        calibration_btn_frame = ctk.CTkFrame(action_buttons_frame, fg_color="transparent")
        calibration_btn_frame.pack(pady=5)

        # Bouton RUN CALIBRATION NODE
        calibration_btn = ctk.CTkButton(
            calibration_btn_frame,
            text="RUN CALIBRATION NODE",
            width=250,
            font=("Arial", 14, "bold"),
            fg_color="#3498db",
            hover_color="#2980b9",
            text_color="white",
            command=self.launch_calibration,
        )
        calibration_btn.pack(side="left", padx=(0, 10))

        calibration_tooltip_text = (
            "To use the DRIVE protocol, you had to pipe multiple topics from your robot to the DRIVE protocol.\n"
            "To help you verify that everything works correctly, you can execute the calibration protocol.\n"
            "To do so, click on the button below and follow the instructions in the foxglove layout."
        )
        calibration_info_icon = create_info_icon(
            calibration_btn_frame, calibration_tooltip_text, color="#3498db", wraplength=400
        )
        calibration_info_icon.pack(side="left")

        # Frame pour bouton LAUNCH DRIVE
        launch_drive_btn_frame = ctk.CTkFrame(action_buttons_frame, fg_color="transparent")
        launch_drive_btn_frame.pack(pady=5)

        # Bouton LAUNCH DRIVE
        launch_drive_btn = ctk.CTkButton(
            launch_drive_btn_frame,
            text="LAUNCH DRIVE",
            width=250,
            fg_color="#4EC23C",
            hover_color="#4F8C46",
            font=("Arial", 16, "bold"),
            command=self.launch_drive,
        )
        launch_drive_btn.pack(side="left", padx=(0, 10))

        # Icône d'infobulle pour Using DRIVE Protocol
        launch_drive_tooltip_text = (
            "To start an experiment of the DRIVE protocol, you must first fill the two following forms:\n"
            "1. Roboticist\n"
            "2. Robot\n\n"
            "Once you have filled these two forms, each time you are going to launch a DRIVE experiment\n"
            "you will have to select the robot and the roboticist that are used in the experiment and fill\n"
            "the terrain form. The terrain form has multiple questions that are easier to answer on site.\n"
            "Thus, these questions are required to launch DRIVE. Once the terrain form is filled, you can\n"
            "select the terrain form and click on launch DRIVE. Once it is launched, the following\n"
            "instructions can be found in the foxglove layout."
        )
        launch_drive_info_icon = create_info_icon(
            launch_drive_btn_frame, launch_drive_tooltip_text, color="#27ae60", wraplength=500
        )
        launch_drive_info_icon.pack(side="left")

    def create_action_buttons(self, parent_frame, row, callback_function):
        if not self.allow_add and not self.allow_edit:
            return

        button_frame = ctk.CTkFrame(parent_frame, fg_color="transparent")
        button_frame.grid(row=row, column=2, padx=10, pady=10)

        button_count = 0
        if self.allow_add:
            ctk.CTkButton(button_frame, text="⊞", width=50, anchor="center", command=callback_function).pack(
                side="left", padx=2
            )
            button_count += 1

        if self.allow_edit:
            ctk.CTkButton(button_frame, text="✏", width=50, anchor="center", command=callback_function).pack(
                side="left", padx=2
            )
            button_count += 1

        if button_count == 1:
            for child in button_frame.winfo_children():
                child.configure(width=100)

    def launch_calibration(self):
        messagebox.showinfo(
            "Calibration",
            "Calibration protocol would be launched here.\n(Bash script to open Foxglove and launch calibration node)",
        )

    def open_roboticist(self):
        current_selection = self.combo_roboticist.get() if self.combo_roboticist.get() else None
        RoboticistMenu(self, initial_selection=current_selection)
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE)
        self.combo_roboticist.configure(values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists])

    def open_robot(self):
        current_selection = self.combo_robot.get() if self.combo_robot.get() else None
        RobotMenu(self, initial_selection=current_selection)
        self.robots = self.utils.load_file(ROBOT_DATA_FILE)
        self.combo_robot.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.robots])

    def open_terrain(self):
        FieldMenu(self)
        self.fields = self.utils.load_file(FIELD_DATA_FILE)
        self.combo_terrain.configure(values=[g.get("name", "Unnamed") for g in self.fields])

    def launch_drive(self):
        if not self.combo_roboticist.get() or not self.combo_robot.get():
            messagebox.showerror("Error", "Please select a roboticist and robot before launching DRIVE.")
            return
        messagebox.showinfo(
            "Launch DRIVE",
            "DRIVE experiment would be launched here.\n(Bash script to open Foxglove and launch DRIVE node)",
        )


if __name__ == "__main__":
    app = Deployment()
    app.mainloop()
