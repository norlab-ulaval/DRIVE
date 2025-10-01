import tkinter as tk
import customtkinter as ctk


class ToolTip:

    def __init__(self, widget, text, delay=500, wraplength=300):

        self.widget = widget
        self.text = text
        self.delay = delay
        self.wraplength = wraplength
        self.tooltip_window = None
        self.after_id = None

        self.widget.bind("<Enter>", self.on_enter)
        self.widget.bind("<Leave>", self.on_leave)
        self.widget.bind("<Motion>", self.on_motion)

    def on_enter(self, event=None):
        self.schedule_tooltip()

    def on_leave(self, event=None):
        self.cancel_tooltip()
        self.hide_tooltip()

    def on_motion(self, event=None):
        self.cancel_tooltip()
        self.schedule_tooltip()

    def schedule_tooltip(self):
        self.cancel_tooltip()
        self.after_id = self.widget.after(self.delay, self.show_tooltip)

    def cancel_tooltip(self):
        if self.after_id:
            self.widget.after_cancel(self.after_id)
            self.after_id = None

    def show_tooltip(self):
        """Affiche l'infobulle."""
        if self.tooltip_window:
            return

        x = self.widget.winfo_rootx() + 25
        y = self.widget.winfo_rooty() + 25

        self.tooltip_window = tk.Toplevel(self.widget)
        self.tooltip_window.wm_overrideredirect(True)
        self.tooltip_window.wm_geometry(f"+{x}+{y}")

        self.tooltip_window.configure(bg="black")

        label = tk.Label(
            self.tooltip_window,
            text=self.text,
            background="black",
            foreground="white",
            relief="solid",
            borderwidth=1,
            font=("Arial", 10),
            wraplength=self.wraplength,
            justify="left",
            padx=8,
            pady=6,
        )
        label.pack()

        self.tooltip_window.lift()

    def hide_tooltip(self):
        if self.tooltip_window:
            self.tooltip_window.destroy()
            self.tooltip_window = None

    def update_text(self, new_text):
        self.text = new_text
        if self.tooltip_window:
            self.hide_tooltip()
            self.show_tooltip()


def add_tooltip(widget, text, delay=500, wraplength=300):
    return ToolTip(widget, text, delay, wraplength)


def create_info_icon(parent, tooltip_text, icon_text="?", color="#3498db", wraplength=400):
    info_icon = ctk.CTkLabel(
        parent,
        text=icon_text,
        font=("Arial", 16, "bold"),
        width=25,
        height=25,
        fg_color=color,
        corner_radius=12,
        text_color="white",
    )

    # Ajouter l'infobulle
    add_tooltip(info_icon, tooltip_text, wraplength=wraplength)

    return info_icon
