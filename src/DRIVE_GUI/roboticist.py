import tkinter as tk
import customtkinter as ctk
from customtkinter import CTkFont
from tkinter import messagebox
from DRIVE_GUI.utils import Utils
import os

ROBOTICISTS_DATA_FILE = "./Experience/roboticists.json"
TITLE = "Drive Menu"
SIZE_MENU = "700x600"
SIZE_SUBMENU = "500x500"


class RoboticistMenu(ctk.CTkToplevel):
    def __init__(self, parent, initial_selection=None):
        super().__init__(parent)
        self.title("Roboticist Menu")
        self.geometry(SIZE_SUBMENU)
        self.resizable(True, True)
        self.initial_selection = initial_selection

        self.utils = Utils()
        self.roboticists = self.utils.load_file(ROBOTICISTS_DATA_FILE) or []
        self.selected_index = 0

        my_font = CTkFont(family="Roboto", size=13, weight="normal")
        frame = ctk.CTkFrame(self)
        frame.pack(pady=(10, 10), padx=10, fill="x")
        ctk.CTkLabel(
            frame, width=70, height=20, corner_radius=20, text="Choose a Roboticist", text_color="white", font=my_font
        ).pack(pady=5)

        self.combo = ctk.CTkComboBox(
            self, values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists], command=self.on_select
        )
        self.combo.pack()

        self.entries = {}
        self.fields = ["Name", "Lastname", "contact email", "Organization"]
        for field in self.fields:
            ctk.CTkLabel(self, text=field + ":", font=my_font).pack(anchor="w", padx=10, pady=10)
            entry = ctk.CTkEntry(self)
            entry.pack(fill="x", padx=20)
            self.entries[field] = entry

        ctk.CTkButton(
            self,
            text="Add a new roboticist",
            fg_color="#3498db",
            hover_color="#2980b9",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.add_new,
        ).pack(pady=(10, 0), padx=40, fill="x")
        ctk.CTkButton(
            self,
            text="Save",
            fg_color="#27ae60",
            hover_color="#219150",
            text_color="white",
            corner_radius=20,
            font=my_font,
            command=self.save,
        ).pack(pady=10, padx=40, fill="x")
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

        if self.roboticists:
            if self.initial_selection:
                names = [f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
                if self.initial_selection in names:
                    idx = names.index(self.initial_selection)
                    self.combo.set(self.initial_selection)
                    self.load_fields(idx)
                else:
                    self.combo.set(f"{self.roboticists[0]['Name']} {self.roboticists[0]['Lastname']}")
                    self.load_fields(0)
            else:
                self.combo.set(f"{self.roboticists[0]['Name']} {self.roboticists[0]['Lastname']}")
                self.load_fields(0)
        else:
            self.add_new()

        self.place_window_center()

    def on_select(self, event=None):
        selected = self.combo.get()
        names = [f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
        if selected in names:
            idx = names.index(selected)
            self.load_fields(idx)

    def load_fields(self, idx):
        data = self.roboticists[idx]
        for field, entry in self.entries.items():
            entry.delete(0, tk.END)
            entry.insert(0, data.get(field, ""))

    def add_new(self):
        for entry in self.entries.values():
            entry.delete(0, tk.END)
        self.combo.set("")

    def save(self):
        new_data = {}
        for field, entry in self.entries.items():
            value = entry.get().strip()
            if not value:
                messagebox.showerror("Erreur", f"{field} is mandatory.")
                return
            new_data[field] = value

        names = [f"{r['Name']} {r['Lastname']}" for r in self.roboticists]
        current_name = f"{new_data['Name']} {new_data['Lastname']}"
        if current_name in names:
            idx = names.index(current_name)
            self.roboticists[idx] = new_data
        else:
            self.roboticists.append(new_data)
            self.combo.configure(values=[f"{r['Name']} {r['Lastname']}" for r in self.roboticists])
            self.combo.set(current_name)

        self.utils.save_file(self.roboticists, ROBOTICISTS_DATA_FILE)
        messagebox.showinfo("Saved", "Roboticist is saved locally.")
        self.add_new()

    def place_window_center(self):
        self.update_idletasks()
        w_height = self.winfo_height()
        w_width = self.winfo_width()
        s_height = self.winfo_screenheight()
        s_width = self.winfo_screenwidth()
        xpos = (s_width - w_width) // 2
        ypos = (s_height - w_height) // 2
        self.geometry(f"+{xpos}+{ypos}")
