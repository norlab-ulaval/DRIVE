import tkinter as tk
import customtkinter as ctk
from customtkinter import CTkFont
from tkinter import messagebox
from pathlib import Path
from DRIVE_GUI.utils import Utils
import os

PROJECT_ROOT = Path(__file__).parent.parent.parent
DRIVE_LIBRARY_PATH = PROJECT_ROOT / "drive_library"
ROBOTICISTS_DATA_FILE = DRIVE_LIBRARY_PATH / "roboticists.json"

TITLE = "Drive Menu"
SIZE_MENU = "700x700"
SIZE_SUBMENU = "500x500"


class RoboticistMenu(ctk.CTkToplevel):
    def __init__(self, parent, initial_selection=None, save_to_library=False, save_path=None):
        super().__init__(parent)
        self.title("Roboticist Menu")
        self.geometry(SIZE_SUBMENU)
        self.resizable(True, True)
        self.initial_selection = initial_selection
        self.save_to_library = save_to_library
        self.save_path = save_path

        self.utils = Utils()
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE) or []
        self.selected_index = 0

        my_font = CTkFont(family="Roboto", size=13, weight="normal")

        self.entries = {}
        self.fields = ["Name", "Lastname", "contact email", "Organization"]
        for field in self.fields:
            ctk.CTkLabel(self, text=field + ":", font=my_font).pack(anchor="w", padx=10, pady=10)
            entry = ctk.CTkEntry(self)
            entry.pack(fill="x", padx=20)
            self.entries[field] = entry

        ctk.CTkButton(
            self,
            text="Save",
            fg_color="#27ae60",
            hover_color="#219150",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.save,
        ).pack(pady=(10, 0), padx=40, fill="x")
        ctk.CTkButton(
            self,
            text="Close",
            fg_color="#e74c3c",
            hover_color="#c0392b",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.destroy,
        ).pack(pady=(0, 10), padx=40, fill="x")

        if self.roboticists and self.initial_selection:
            names = [f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
            if self.initial_selection in names:
                idx = names.index(self.initial_selection)
                self.load_fields(idx)

        self.place_window_center()

    def load_fields(self, idx):
        data = self.roboticists[idx]
        for field, entry in self.entries.items():
            entry.delete(0, tk.END)
            entry.insert(0, data.get(field, ""))

    def save(self):
        new_data = {}
        for field, entry in self.entries.items():
            value = entry.get().strip()
            if not value:
                messagebox.showerror("Erreur", f"{field} is mandatory.")
                return
            new_data[field] = value

        if self.save_to_library:
            # Add mode: save to drive_library
            names = [f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
            current_name = f"{new_data['Name']} {new_data['Lastname']}"
            if current_name in names:
                idx = names.index(current_name)
                self.roboticists[idx] = new_data
            else:
                self.roboticists.append(new_data)

            self.utils.save_file(self.roboticists, str(ROBOTICISTS_DATA_FILE))
            messagebox.showinfo("Saved", "Roboticist is saved to library.")
        elif self.save_path:
            # Edit mode: save to deployment metadata
            self.utils.save_file([new_data], self.save_path)
            messagebox.showinfo("Saved", "Roboticist is saved to deployment metadata.")
        else:
            # Fallback: save to library
            names = [f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
            current_name = f"{new_data['Name']} {new_data['Lastname']}"
            if current_name in names:
                idx = names.index(current_name)
                self.roboticists[idx] = new_data
            else:
                self.roboticists.append(new_data)

            self.utils.save_file(self.roboticists, str(ROBOTICISTS_DATA_FILE))
            messagebox.showinfo("Saved", "Roboticist is saved locally.")

    def place_window_center(self):
        self.update_idletasks()
        w_height = self.winfo_height()
        w_width = self.winfo_width()
        s_height = self.winfo_screenheight()
        s_width = self.winfo_screenwidth()
        xpos = (s_width - w_width) // 2
        ypos = (s_height - w_height) // 2
        self.geometry(f"+{xpos}+{ypos}")
