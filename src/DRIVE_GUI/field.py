import tkinter as tk
import customtkinter as ctk
from tkinter import messagebox, filedialog
from PIL import Image, ImageTk
import shutil
import os
from DRIVE_GUI.utils import Utils
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
DRIVE_LIBRARY_PATH = PROJECT_ROOT / "drive_library"
FIELD_DATA_FILE = DRIVE_LIBRARY_PATH / "ground.json"


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


class FieldMenu(ctk.CTkToplevel):
    def __init__(self, parent, initial_selection=None, save_to_library=False, save_path=None):
        super().__init__(parent)
        self.title("Terrain Form")
        self.geometry("700x900")
        self.resizable(True, True)
        self.initial_selection = initial_selection
        self.save_to_library = save_to_library
        self.save_path = save_path
        self.utils = Utils()
        self.entries = {}

        self.fields = self.utils.load_file(str(FIELD_DATA_FILE)) or []
        self.selected_index = None

        self.image_paths = {"image_closeup": "", "image_overview": "", "image_robot": ""}
        self.image_labels = {}

        self.first_page_fields = [
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
            ("image_closeup", "1) Close-up (30 cm of the ground):", True),
            ("image_overview", "2) Picture of overview:", True),
            ("image_robot", "3) Picture of your robot:", True),
            ("longitude", "Longitude:", True),
            ("latitude", "Latitude:", True),
        ]

        self.first_page = ctk.CTkFrame(self)
        self.second_page = ctk.CTkFrame(self)

        ctk.CTkLabel(self.first_page, text="Start from:", font=("Arial", 14)).grid(
            row=0, column=0, sticky="w", padx=20, pady=10
        )
        self.combo_field = ctk.CTkComboBox(
            self.first_page, values=[g.get("name", "Unnamed") for g in self.fields], command=self.on_select
        )
        self.combo_field.grid(row=0, column=1, padx=20, pady=10, sticky="ew")

        if not self.fields:
            self.combo_field.set("")

        row = 1
        for key, label, mandatory in self.first_page_fields:
            if key == "particle_size":
                self.particle_size_label = ctk.CTkLabel(self.first_page, text=label, font=("Arial", 14))
                self.entries[key] = ctk.CTkComboBox(self.first_page, values=PARTICLE_SIZE)
                continue

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
            elif key.startswith("image_"):
                image_frame = ctk.CTkFrame(self.second_page)
                image_frame.grid(row=row, column=1, padx=20, pady=10, sticky="ew")

                upload_btn = ctk.CTkButton(
                    image_frame, text="Upload Image", command=lambda k=key: self.upload_single_image(k)
                )
                upload_btn.pack(side="left", padx=5)

                self.image_labels[key] = ctk.CTkLabel(image_frame, text="No image selected")
                self.image_labels[key].pack(side="left", padx=5)

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

        if self.fields and self.initial_selection:
            names = [g.get("name", "Unnamed") for g in self.fields]
            if self.initial_selection in names:
                idx = names.index(self.initial_selection)
                self.selected_index = idx
                self.load_fields(idx)

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
        selected = self.combo_field.get()
        names = [g.get("name", "Unnamed") for g in self.fields]
        if selected in names:
            idx = names.index(selected)
            self.selected_index = idx
            self.load_fields(idx)

    def load_fields(self, idx):
        data = self.fields[idx]
        for key in self.entries:
            value = data.get(key, "")
            entry = self.entries[key]
            if isinstance(entry, ctk.CTkComboBox):
                entry.set(value)
            else:
                entry.delete(0, tk.END)
                entry.insert(0, value)

        for image_key in self.image_paths:
            stored_path = data.get(image_key, "")
            if stored_path:
                library_image_path = DRIVE_LIBRARY_PATH / stored_path
                if library_image_path.exists():
                    self.image_paths[image_key] = str(library_image_path)
                    self.image_labels[image_key].configure(text=f"✓ {stored_path}")
                else:
                    self.image_paths[image_key] = ""
                    self.image_labels[image_key].configure(text="No image selected")
            else:
                self.image_paths[image_key] = ""
                self.image_labels[image_key].configure(text="No image selected")

        for k, var in self.contamination_vars.items():
            var.set(k in data.get("contaminations", []))
        self.on_deformability_change()

    def on_deformability_change(self, event=None):
        deform = self.entries["deformability"].get().strip().lower()
        if deform == "deformable":
            self.particle_size_label.grid(row=6, column=0, sticky="w", padx=20, pady=10)
            self.entries["particle_size"].grid(row=6, column=1, padx=20, pady=10, sticky="ew")
        else:
            self.particle_size_label.grid_remove()
            self.entries["particle_size"].grid_remove()

    def upload_single_image(self, image_key):
        file = filedialog.askopenfilename(
            parent=self,
            title=f"Select {image_key.replace('_', ' ')}",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.gif")],
        )
        if file:
            try:
                _, ext = os.path.splitext(file)
                destination_name = f"{image_key}{ext}"

                if self.save_to_library:
                    destination_path = DRIVE_LIBRARY_PATH / destination_name
                elif self.save_path:
                    deployment_metadata_path = Path(self.save_path).parent
                    destination_path = deployment_metadata_path / destination_name
                else:
                    destination_path = DRIVE_LIBRARY_PATH / destination_name

                shutil.copy2(file, str(destination_path))

                self.image_paths[image_key] = str(destination_path)
                self.image_labels[image_key].configure(text=f"✓ {destination_name}")

            except Exception as e:
                messagebox.showerror("Error", f"Failed to copy image: {str(e)}")

    def save(self):
        mandatory_fields = [k for k, _, m in self.first_page_fields + self.second_page_fields if m]
        for key in mandatory_fields:
            if key == "contaminations":
                if not any(var.get() for var in self.contamination_vars.values()):
                    messagebox.showerror("Erreur", "At least one contamination must be selected.")
                    return
            elif key == "particle_size" and self.entries["deformability"].get().strip().lower() != "deformable":
                continue
            elif key.startswith("image_"):
                if not self.image_paths[key]:
                    messagebox.showerror("Erreur", f"{key.replace('_', ' ')} image is mandatory.")
                    return
            elif key in self.entries and not self.entries[key].get().strip():
                messagebox.showerror("Erreur", f"{key} is mandatory.")
                return

        data = {key: self.entries[key].get().strip() for key in self.entries}
        data["contaminations"] = [k for k, v in self.contamination_vars.items() if v.get()]

        for image_key, path in self.image_paths.items():
            if path:
                data[image_key] = os.path.basename(path)
            else:
                data[image_key] = ""

        if self.save_to_library:
            if self.selected_index is not None:
                self.fields[self.selected_index] = data
            else:
                self.fields.append(data)
            self.utils.save_file(self.fields, str(FIELD_DATA_FILE))
            messagebox.showinfo("Saved", "Terrain form saved to library.")
        elif self.save_path:
            self.utils.save_file([data], self.save_path)
            messagebox.showinfo("Saved", "Terrain form saved to deployment metadata.")
        else:
            if self.selected_index is not None:
                self.fields[self.selected_index] = data
            else:
                self.fields.append(data)
            self.utils.save_file(self.fields, str(FIELD_DATA_FILE))
            messagebox.showinfo("Saved", "Terrain form saved successfully.")

        self.destroy()
