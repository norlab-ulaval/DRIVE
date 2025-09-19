import tkinter as tk
import customtkinter as ctk
from tkinter import messagebox, filedialog
from DRIVE_GUI.utils import Utils

GROUND_DATA_FILE = "./Experience/ground.json"

TERRAIN_TYPES = ["Asphalt", "Gravel", "Grass", "Sand", "Ice", "Snow", "Muskeg", "Clay", "Mud", "Other"]
DEFORMABILITY = ["deformable", "hard"]
PARTICLE_SIZE = [
    "Smaller than 0.05 mm (clay)",
    "0.05 mm to 2.0 mm (sand)",
    "2.0 mm to 20 mm (small gravel)",
    "20 to 63 mm (big gravel)",
    "Over 63 mm (Cobbles)",
]
CONTAMINATIONS = [
    "Ice",
    "Dust",
    "Clay (<0.05 mm)",
    "Sand (0.05-2.0 mm)",
    "Small gravel (2.0-20 mm)",
    "Big gravel (20-63 mm)",
    "Cobbles (>63 mm)",
]
FLAT_BUMPY = ["Flat", "Bumpy"]
WETNESS = ["My finger is wet after touching the ground.", "My finger is not humid after touching the ground."]
YES_NO = ["Yes", "No"]


class GroundMenu(ctk.CTkToplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Terrain Form")
        self.geometry("700x900")
        self.resizable(True, True)
        self.utils = Utils()
        self.images = []
        self.entries = {}

        # Charger terrains existants
        self.grounds = self.utils.load_file(GROUND_DATA_FILE) or []
        self.selected_index = None

        # Découpe des champs
        self.first_page_fields = [
            ("start_from", "Start from:", True),
            ("name", "Name:", True),
            ("terrain_type", "What is the terrain the robot is on?", True),
            ("uniform_percent", "Percentage of uniform terrain:", True),
            ("deformability", "Is the terrain deformable or hard?", True),
            ("particle_size", "Size of most present particles (if deformable):", False),
            ("contaminations", "Visible contamination (multiple):", True),
            ("flat_bumpy", "Is the terrain flat or bumpy at the beginning?", True),
            ("wetness", "Is the terrain wet?", True),
        ]
        self.second_page_fields = [
            ("weather", "Weather (sunny, cloudy, etc.):", True),
            ("time_of_day", "Time of the day:", True),
            ("temperature", "Temperature:", True),
            ("ground_frozen", "Is the ground frozen?", True),
            ("terrain_froze_night", "Did the terrain freeze last night?", True),
            ("images", "Upload Images", False),
            ("longitude", "Longitude:", True),
            ("latitude", "Latitude:", True),
        ]

        self.first_page = ctk.CTkFrame(self)
        self.second_page = ctk.CTkFrame(self)

        ctk.CTkLabel(self.first_page, text="Select a terrain:", font=("Arial", 14)).grid(
            row=0, column=0, sticky="w", padx=20, pady=10
        )
        self.combo = ctk.CTkComboBox(
            self.first_page, values=[g.get("name", "Unnamed") for g in self.grounds], command=self.on_select
        )
        self.combo.grid(row=0, column=1, padx=20, pady=10, sticky="ew")

        row = 1
        for key, label, mandatory in self.first_page_fields:
            ctk.CTkLabel(self.first_page, text=label, font=("Arial", 14)).grid(
                row=row, column=0, sticky="w", padx=20, pady=10
            )
            if key == "terrain_type":
                self.entries[key] = ctk.CTkComboBox(self.first_page, values=TERRAIN_TYPES)
                self.entries[key].grid(row=row, column=1, padx=20, pady=10, sticky="ew")
            elif key == "deformability":
                self.entries[key] = ctk.CTkComboBox(
                    self.first_page, values=DEFORMABILITY, command=self.on_deformability_change
                )
                self.entries[key].grid(row=row, column=1, padx=20, pady=10, sticky="ew")
            elif key == "particle_size":
                self.entries[key] = ctk.CTkComboBox(self.first_page, values=PARTICLE_SIZE)
            elif key == "contaminations":
                self.contamination_vars = {}
                contam_frame = ctk.CTkFrame(self.first_page)
                contam_frame.grid(row=row, column=1, padx=20, pady=10, sticky="w")
                for contam in CONTAMINATIONS:
                    var = tk.BooleanVar()
                    chk = ctk.CTkCheckBox(contam_frame, text=contam, variable=var)
                    chk.pack(anchor="w")
                    self.contamination_vars[contam] = var
            elif key in ["flat_bumpy", "wetness"]:
                self.entries[key] = ctk.CTkComboBox(
                    self.first_page, values=FLAT_BUMPY if key == "flat_bumpy" else WETNESS
                )
                self.entries[key].grid(row=row, column=1, padx=20, pady=10, sticky="ew")
            else:
                self.entries[key] = ctk.CTkEntry(self.first_page)
                self.entries[key].grid(row=row, column=1, padx=20, pady=10, sticky="ew")
            row += 1

        row = 0
        for key, label, mandatory in self.second_page_fields:
            ctk.CTkLabel(self.second_page, text=label, font=("Arial", 14)).grid(
                row=row, column=0, sticky="w", padx=20, pady=10
            )
            if key in ["ground_frozen", "terrain_froze_night"]:
                self.entries[key] = ctk.CTkComboBox(self.second_page, values=YES_NO)
                self.entries[key].grid(row=row, column=1, padx=20, pady=10, sticky="ew")
            elif key == "images":
                ctk.CTkLabel(
                    self.second_page,
                    text="Images à fournir :\n"
                     "1) Close-up (30 cm of the ground)\n"
                    "2) Area overview (DRIVE available space)\n"
                    "3) Picture of your robot",
                    font=("Arial", 12),
                    justify="left").grid(row=row, column=0, columnspan=2, sticky="w", padx=20, pady=5)
                ctk.CTkButton(self.second_page, text="Upload Images", command=self.upload_images).grid(
                    row=row, column=1, padx=20, pady=10, sticky="w"
                )
            else:
                self.entries[key] = ctk.CTkEntry(self.second_page)
                self.entries[key].grid(row=row, column=1, padx=20, pady=10, sticky="ew")
            row += 1

        self.next_btn = ctk.CTkButton(self, text="Next", command=self.show_second_page)
        self.prev_btn = ctk.CTkButton(self, text="Previous", command=self.show_first_page)
        self.save_btn = ctk.CTkButton(self, text="Save", fg_color="#27ae60", text_color="white", command=self.save)
        self.close_btn = ctk.CTkButton(self, text="Close", fg_color="#e74c3c", text_color="white", command=self.destroy)

        self.close_btn.pack(pady=(0, 10), padx=40, fill="x")
        self.show_first_page()
        self.on_deformability_change()

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
        names = [g.get("name", "Unnamed") for g in self.grounds]
        if selected in names:
            idx = names.index(selected)
            self.selected_index = idx
            self.load_fields(idx)

    def load_fields(self, idx):
        data = self.grounds[idx]
        for key in self.entries:
            value = data.get(key, "")
            entry = self.entries[key]
            if isinstance(entry, ctk.CTkComboBox):
                entry.set(value)
            else:
                entry.delete(0, tk.END)
                entry.insert(0, value)
        for k, var in self.contamination_vars.items():
            var.set(k in data.get("contaminations", []))
        self.images = data.get("images", [])
        self.on_deformability_change()

    def on_deformability_change(self, event=None):
        deform = self.entries["deformability"].get().strip().lower()
        if deform == "deformable":
            self.entries["particle_size"].grid(row=5, column=1, padx=20, pady=10, sticky="ew")
        else:
            self.entries["particle_size"].grid_remove()

    def upload_images(self):
        files = filedialog.askopenfilenames(
            parent=self, title="Select images", filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.gif")]
        )
        if files:
            self.images = list(files)
            messagebox.showinfo("Images", f"{len(self.images)} image(s) selected.")

    def save(self):
        mandatory_fields = [k for k, _, m in self.first_page_fields + self.second_page_fields if m]
        for key in mandatory_fields:
            if key == "contaminations":
                if not any(var.get() for var in self.contamination_vars.values()):
                    messagebox.showerror("Erreur", "At least one contamination must be selected.")
                    return
            elif key == "particle_size" and self.entries["deformability"].get().strip().lower() != "deformable":
                continue
            elif key in self.entries and not self.entries[key].get().strip():
                messagebox.showerror("Erreur", f"{key} is mandatory.")
                return

        data = {key: self.entries[key].get().strip() for key in self.entries}
        data["contaminations"] = [k for k, v in self.contamination_vars.items() if v.get()]
        data["images"] = self.images

        if self.selected_index is not None:
            self.grounds[self.selected_index] = data
        else:
            self.grounds.append(data)
        self.utils.save_file(self.grounds, GROUND_DATA_FILE)

        messagebox.showinfo("Saved", "Terrain form saved successfully.")
        self.destroy()
