import tkinter as tk
import customtkinter as ctk
from customtkinter import CTkFont
from DRIVE_GUI.utils import Utils

ROBOT_DATA_FILE = "./Experience/robot.json"
SIZE_SUBMENU = "700x800"

TRACTION_OPTIONS = ["Wheels", "Tracks", "Legs", "Other"]
SENSOR_OPTIONS = ["IMU", "RADAR", "LIDAR", "GPS", "CAMERAS", "Wheel", "Encoder", "Microphones", "Other (specify)"]


class RobotMenu(ctk.CTkToplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Robot Menu")
        self.geometry(SIZE_SUBMENU)
        self.resizable(True, True)

        self.utils = Utils()
        self.robots = self.utils.load_file(ROBOT_DATA_FILE) or []
        self.selected_index = 0
        self.current_page = 1

        my_font = CTkFont(family="Roboto", size=13, weight="normal")
        frame = ctk.CTkFrame(self)
        frame.pack(pady=(10, 10), padx=10, fill="x")
        ctk.CTkLabel(
            frame, width=70, height=20, corner_radius=20, text="Choose a Robot", text_color="white", font=my_font
        ).pack(pady=5)

        self.combo = ctk.CTkComboBox(
            self, values=[f"{r['robot']} (v{r['version']})" for r in self.robots], command=self.on_select
        )
        self.combo.pack()

        self.entries = {}
        self.first_page_fields = [
            ("robot", "Robot manufacturer name", True),
            ("version", "Robot version", True),
            ("modifications", "Changes to traction/motor/battery?", True),
            ("weight", "Total weight (robot + sensor rack)", True),
            ("asymmetry", "Any asymmetry in movement?", True),
            ("traction", "Traction mechanism", True),
            ("tyre_model", "Tyre model (if wheels)", False),
            ("thread_depth", "Thread depth (if wheels)", False),
            ("tyre_pressure", "Tyre pressures (clockwise from front right)", False),
            ("tracks_model", "Tracks model (if tracks)", False),
        ]
        self.second_page_fields = [
            ("has_suspension", "Vehicle equipped with suspension?", True),
            ("suspension_type", "Type of suspension", False),
            ("baseline", "Measured baseline (m)", True),
            ("wheel_radius", "Measured loaded wheel radius (m)", True),
            ("sensors", "Sensors used for localization", True),
            ("fusion_algo", "Sensor fusion algorithm (link/paper/github)", True),
            ("max_speed", "Max validated speed (localization pipeline)", True),
        ]

        self.first_page = ctk.CTkFrame(self)
        self.second_page = ctk.CTkFrame(self)

        for key, label, mandatory in self.first_page_fields:
            if key == "traction":
                ctk.CTkLabel(self.first_page, text=label + ":", font=my_font).pack(anchor="w", padx=10, pady=10)
                self.entries[key] = ctk.CTkComboBox(
                    self.first_page, values=TRACTION_OPTIONS, command=self.on_traction_change
                )
                self.entries[key].pack(fill="x", padx=20)
            else:
                ctk.CTkLabel(self.first_page, text=label + ":", font=my_font).pack(anchor="w", padx=10, pady=10)
                self.entries[key] = ctk.CTkEntry(self.first_page)
                self.entries[key].pack(fill="x", padx=20)

        for key, label, mandatory in self.second_page_fields:
            if key == "sensors":
                ctk.CTkLabel(self.second_page, text=label + ":", font=my_font).pack(anchor="w", padx=10, pady=10)
                self.entries[key] = ctk.CTkComboBox(self.second_page, values=SENSOR_OPTIONS)
                self.entries[key].pack(fill="x", padx=20)
            else:
                ctk.CTkLabel(self.second_page, text=label + ":", font=my_font).pack(anchor="w", padx=10, pady=10)
                self.entries[key] = ctk.CTkEntry(self.second_page)
                self.entries[key].pack(fill="x", padx=20)

        # Navigation
        self.next_btn = ctk.CTkButton(self, text="Next", command=self.show_second_page)
        self.prev_btn = ctk.CTkButton(self, text="Previous", command=self.show_first_page)
        self.save_btn = ctk.CTkButton(
            self,
            text="Save",
            fg_color="#27ae60",
            hover_color="#219150",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.save,
        )
        self.close_btn = ctk.CTkButton(
            self,
            text="Close",
            fg_color="#e74c3c",
            hover_color="#c0392b",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.destroy,
        )
        self.add_btn = ctk.CTkButton(
            self,
            text="Add a new robot",
            fg_color="#3498db",
            hover_color="#2980b9",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.add_new,
        )

        self.add_btn.pack(pady=(10, 0), padx=40, fill="x")
        self.next_btn.pack(pady=10, padx=40, fill="x")
        self.close_btn.pack(pady=(0, 10), padx=40, fill="x")

        if self.robots:
            self.combo.set(f"{self.robots[0]['robot']} (v{self.robots[0]['version']})")
            self.load_fields(0)
        else:
            self.add_new()

        self.place_window_center()
        self.show_first_page()
        self.on_traction_change()

    def show_first_page(self):
        self.second_page.pack_forget()
        self.first_page.pack(fill="both", expand=True)
        self.next_btn.pack(pady=10, padx=40, fill="x")
        self.prev_btn.pack_forget()
        self.save_btn.pack_forget()

    def show_second_page(self):
        self.first_page.pack_forget()
        self.second_page.pack(fill="both", expand=True)
        self.next_btn.pack_forget()
        self.prev_btn.pack(pady=10, padx=40, fill="x")
        self.save_btn.pack(pady=10, padx=40, fill="x")

    def on_select(self, event=None):
        selected = self.combo.get()
        names = [f"{r['robot']} (v{r['version']})" for r in self.robots]
        if selected in names:
            idx = names.index(selected)
            self.load_fields(idx)

    def load_fields(self, idx):
        data = self.robots[idx]
        for key in self.entries:
            value = data.get(key, "")
            entry = self.entries[key]
            if isinstance(entry, ctk.CTkComboBox):
                entry.set(value)
            else:
                entry.delete(0, tk.END)
                entry.insert(0, value)
        self.on_traction_change()

    def add_new(self):
        for key in self.entries:
            entry = self.entries[key]
            if isinstance(entry, ctk.CTkComboBox):
                entry.set("")
            else:
                entry.delete(0, tk.END)
        self.combo.set("")
        self.on_traction_change()

    def save(self):
        for key, _, mandatory in self.first_page_fields + self.second_page_fields:
            entry = self.entries[key]
            value = entry.get().strip()
            if mandatory and not value:
                tk.messagebox.showerror("Erreur", f"{key} is mandatory.")
                return

        new_data = {key: self.entries[key].get().strip() for key in self.entries}
        names = [f"{r['robot']} (v{r['version']})" for r in self.robots]
        current_name = f"{new_data['robot']} (v{new_data['version']})"
        if current_name in names:
            idx = names.index(current_name)
            self.robots[idx] = new_data
        else:
            self.robots.append(new_data)
            self.combo.configure(values=[f"{r['robot']} (v{r['version']})" for r in self.robots])
            self.combo.set(current_name)

        self.utils.save_file(self.robots, ROBOT_DATA_FILE)
        tk.messagebox.showinfo("Saved", "Robot is saved locally.")
        self.add_new()
        self.show_first_page()

    def on_traction_change(self, event=None):
        traction = self.entries["traction"].get()
        if traction == "Wheels":
            self.entries["tyre_model"].pack(fill="x", padx=20)
            self.entries["thread_depth"].pack(fill="x", padx=20)
            self.entries["tyre_pressure"].pack(fill="x", padx=20)
            self.entries["tracks_model"].pack_forget()
        elif traction == "Tracks":
            self.entries["tracks_model"].pack(fill="x", padx=20)
            self.entries["tyre_model"].pack_forget()
            self.entries["thread_depth"].pack_forget()
            self.entries["tyre_pressure"].pack_forget()
        else:
            self.entries["tyre_model"].pack_forget()
            self.entries["thread_depth"].pack_forget()
            self.entries["tyre_pressure"].pack_forget()
            self.entries["tracks_model"].pack_forget()

    def place_window_center(self):
        self.update_idletasks()
        w_height = self.winfo_height()
        w_width = self.winfo_width()
        s_height = self.winfo_screenheight()
        s_width = self.winfo_screenwidth()
        xpos = (s_width - w_width) // 2
        ypos = (s_height - w_height) // 2
        self.geometry(f"+{xpos}+{ypos}")
