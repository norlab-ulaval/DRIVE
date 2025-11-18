import tkinter as tk
import customtkinter as ctk
import json
import shutil
from pathlib import Path
from tkinter import messagebox
from DRIVE_GUI.roboticist import RoboticistMenu
from DRIVE_GUI.robot import RobotMenu
from DRIVE_GUI.field import FieldMenu
from DRIVE_GUI.utils import Utils
from DRIVE_GUI.tooltip import create_info_icon

PROJECT_ROOT = Path(__file__).parent.parent.parent
DRIVE_LIBRARY_PATH = PROJECT_ROOT / "drive_library"
ROBOTICISTS_DATA_FILE = DRIVE_LIBRARY_PATH / "roboticists.json"
ROBOT_DATA_FILE = DRIVE_LIBRARY_PATH / "robot.json"
FIELD_DATA_FILE = DRIVE_LIBRARY_PATH / "ground.json"


class Deployment(ctk.CTkFrame):
    def __init__(self, parent, allow_add=True, allow_edit=True):
        super().__init__(parent)
        self.parent = parent

        self.allow_add = allow_add
        self.allow_edit = allow_edit
        DRIVE_LIBRARY_PATH.mkdir(parents=True, exist_ok=True)

        self.experience_name = None
        self.deployment_name = None
        self.deployment_path = None
        self.deployment_metadata_path = None
        self.template_path = None

        self.utils = Utils()
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE)
        self.robots = self.utils.load_file(ROBOT_DATA_FILE)
        self.fields = self.utils.load_file(FIELD_DATA_FILE)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=0)

        self.title_label = ctk.CTkLabel(self, text="DEPLOYMENT", font=("Arial", 20, "bold"))
        self.title_label.grid(row=0, column=0, pady=20, sticky="ew")

        experiment_frame = ctk.CTkScrollableFrame(self)
        experiment_frame.grid(row=1, column=0, sticky="nsew", padx=40, pady=(0, 20))
        experiment_frame.grid_columnconfigure(0, weight=1)

        experiment_title = ctk.CTkLabel(experiment_frame, text="Using DRIVE Protocol", font=("Arial", 20, "bold"))
        experiment_title.grid(row=0, column=0, pady=10, sticky="ew")

        select_frame = ctk.CTkFrame(experiment_frame)
        select_frame.grid(row=1, column=0, sticky="ew", padx=20, pady=20)
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
            self.combo_roboticist.set("Select a roboticist")

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
            self.combo_robot.set("Select a robot")

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
        self.create_action_buttons(select_frame, 2, self.open_terrain)
        if not self.fields:
            self.combo_terrain.set("Select a terrain")

        action_buttons_frame = ctk.CTkFrame(experiment_frame, fg_color="transparent")
        action_buttons_frame.grid(row=2, column=0, sticky="ew", pady=20)

        calibration_btn_frame = ctk.CTkFrame(action_buttons_frame, fg_color="transparent")
        calibration_btn_frame.pack(pady=5, expand=True)

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
        launch_drive_btn_frame.pack(pady=5, expand=True)

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
            ctk.CTkButton(button_frame, text="Add", width=50, anchor="center", command=callback_function).pack(
                side="left", padx=2
            )
            button_count += 1

        if self.allow_edit:
            ctk.CTkButton(button_frame, text="Edit", width=50, anchor="center", command=callback_function).pack(
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
        if self.allow_add:
            RoboticistMenu(self, initial_selection=current_selection, save_to_library=True)
            # En mode Add, loader depuis la bibliothèque
            self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE)
            self.combo_roboticist.configure(values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists])
        elif self.allow_edit and self.deployment_metadata_path:
            save_path = str(self.deployment_metadata_path / "roboticists.json")
            RoboticistMenu(self, initial_selection=current_selection, save_path=save_path)
            self.load_deployment_metadata()
        else:
            RoboticistMenu(self, initial_selection=current_selection)
            self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE)
            self.combo_roboticist.configure(values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists])

    def open_robot(self):
        current_selection = self.combo_robot.get() if self.combo_robot.get() else None
        if self.allow_add:
            RobotMenu(self, initial_selection=current_selection, save_to_library=True)
            self.robots = self.utils.load_file(ROBOT_DATA_FILE)
            self.combo_robot.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.robots])
        elif self.allow_edit and self.deployment_metadata_path:
            save_path = str(self.deployment_metadata_path / "robot.json")
            RobotMenu(self, initial_selection=current_selection, save_path=save_path)
            self.load_deployment_metadata()
        else:
            RobotMenu(self, initial_selection=current_selection)
            self.robots = self.utils.load_file(ROBOT_DATA_FILE)
            self.combo_robot.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.robots])

    def open_terrain(self):
        current_selection = self.combo_terrain.get() if self.combo_terrain.get() else None
        if self.allow_add:
            FieldMenu(self, initial_selection=current_selection, save_to_library=True)
            self.fields = self.utils.load_file(FIELD_DATA_FILE)
            self.combo_terrain.configure(values=[g.get("name", "Unnamed") for g in self.fields])
        elif self.allow_edit and self.deployment_metadata_path:
            save_path = str(self.deployment_metadata_path / "ground.json")
            FieldMenu(self, initial_selection=current_selection, save_path=save_path)
            self.load_deployment_metadata()
        else:
            FieldMenu(self, initial_selection=current_selection)
            self.fields = self.utils.load_file(FIELD_DATA_FILE)
            self.combo_terrain.configure(values=[g.get("name", "Unnamed") for g in self.fields])

    def launch_drive(self):
        if not self.combo_roboticist.get() or not self.combo_robot.get():
            messagebox.showerror("Error", "Please select a roboticist and robot before launching DRIVE.")
            return

        if self.deployment_metadata_path:
            self.save_deployment_metadata()

        messagebox.showinfo(
            "Launch DRIVE",
            "DRIVE experiment would be launched here.\n(Bash script to open Foxglove and launch DRIVE node)",
        )

    def set_deployment_info(
        self, experience_name, deployment_name, deployment_path, deployment_metadata_path, template_path
    ):
        self.experience_name = experience_name
        self.deployment_path = deployment_path
        self.deployment_metadata_path = deployment_metadata_path
        self.template_path = template_path

        if deployment_path:
            self.deployment_name = deployment_path.name
            self.title_label.configure(text=f"{self.deployment_name}")
        else:
            self.deployment_name = deployment_name
            if deployment_name:
                self.title_label.configure(text=f"{deployment_name}")

        if self.allow_edit and self.deployment_metadata_path:
            self.load_deployment_metadata()

    def load_deployment_metadata(self):
        if not self.deployment_metadata_path:
            return

        metadata_roboticists_file = self.deployment_metadata_path / "roboticists.json"
        if metadata_roboticists_file.exists():
            metadata_roboticists = self.utils.load_file(metadata_roboticists_file)
            if metadata_roboticists:
                self.roboticists = metadata_roboticists
                self.combo_roboticist.configure(values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists])
                if self.roboticists:
                    self.combo_roboticist.set(f"{self.roboticists[0]['Name']} {self.roboticists[0]['Lastname']}")

        metadata_robot_file = self.deployment_metadata_path / "robot.json"
        if metadata_robot_file.exists():
            metadata_robots = self.utils.load_file(metadata_robot_file)
            if metadata_robots:
                self.robots = metadata_robots
                self.combo_robot.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.robots])
                if self.robots:
                    self.combo_robot.set(f"{self.robots[0]['robot']} (v{self.robots[0]['version']})")

        metadata_ground_file = self.deployment_metadata_path / "ground.json"
        if metadata_ground_file.exists():
            metadata_fields = self.utils.load_file(metadata_ground_file)
            if metadata_fields:
                self.fields = metadata_fields
                self.combo_terrain.configure(values=[g.get("name", "Unnamed") for g in self.fields])
                if self.fields:
                    self.combo_terrain.set(self.fields[0].get("name", "Unnamed"))

    def save_deployment_metadata(self):
        if not self.deployment_metadata_path:
            messagebox.showerror("Error", "No deployment metadata path defined.")
            return

        try:
            selected_roboticist = self.combo_roboticist.get()
            selected_robot = self.combo_robot.get()
            selected_terrain = self.combo_terrain.get()

            if not all([selected_roboticist, selected_robot, selected_terrain]):
                messagebox.showwarning("Warning", "Please select a roboticist, robot, and terrain before saving.")
                return

            for roboticist in self.roboticists:
                if f"{roboticist['Name']} {roboticist['Lastname']}" == selected_roboticist:
                    with open(self.deployment_metadata_path / "roboticists.json", "w") as f:
                        json.dump([roboticist], f, indent=2)
                    break

            robot_name = None
            for robot in self.robots:
                if f"{robot['robot']} (v{robot['version']})" == selected_robot:
                    with open(self.deployment_metadata_path / "robot.json", "w") as f:
                        json.dump([robot], f, indent=2)
                    robot_name = robot["robot"]
                    break

            terrain_name = None
            for field in self.fields:
                if field.get("name", "Unnamed") == selected_terrain:
                    with open(self.deployment_metadata_path / "ground.json", "w") as f:
                        json.dump([field], f, indent=2)
                    terrain_name = field.get("name", "Unnamed")
                    break

            image_files = ["image_closeup.jpg", "image_overview.jpg", "image_robot.jpg"]
            for image_file in image_files:
                source_file = self.template_path / image_file
                if source_file.exists():
                    shutil.copy2(source_file, self.deployment_metadata_path / image_file)

            if self.deployment_path and robot_name and terrain_name:
                from datetime import datetime

                if self.deployment_path.name.startswith("Deployment-"):
                    today = datetime.now().strftime("%Y-%m-%d")
                    new_name = f"{today}_{robot_name}_{terrain_name}"
                    new_path = self.deployment_path.parent / new_name

                    counter = 1
                    original_new_path = new_path
                    while new_path.exists():
                        new_path = self.deployment_path.parent / f"{original_new_path.name}_{counter}"
                        counter += 1

                    self.deployment_path.rename(new_path)
                    self.deployment_path = new_path
                    self.deployment_metadata_path = new_path / "Metadata"
                    self.deployment_name = new_path.name
                    self.title_label.configure(text=f"{self.deployment_name}")

            print(f"Deployment metadata saved to: {self.deployment_metadata_path}")
            messagebox.showinfo("Success", "Deployment metadata saved successfully!")

            root = self.winfo_toplevel()
            if hasattr(root, "refresh_experiences_list"):
                root.refresh_experiences_list()

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save deployment metadata: {str(e)}")
            print(f"Error saving metadata: {e}")
