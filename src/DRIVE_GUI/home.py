import tkinter as tk
from tkinter import messagebox
import customtkinter as ctk
import os
import json
import shutil
from datetime import datetime
from pathlib import Path
from .deployment import Deployment


class Home(ctk.CTk):
    def __init__(self):
        super().__init__()
        try:
            self.state("zoomed")
        except:
            try:
                self.attributes("-zoomed", True)
            except:
                self.geometry(f"{self.winfo_screenwidth()}x{self.winfo_screenheight()}+0+0")
        self.resizable(True, True)

        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self.new_experience_frame = None
        self.experience_name_entry = None
        self.selected_experience = None

        self.setup_ui()
        self.load_existing_experiences()

    def setup_ui(self):
        # Configuration de la grille principale : 2 colonnes (1/3 gauche, 2/3 droite)
        self.grid_columnconfigure(0, weight=1, minsize=400)  # Colonne gauche : contrôles
        self.grid_columnconfigure(1, weight=2, minsize=800)  # Colonne droite : visualisations futures
        self.grid_rowconfigure(1, weight=1)

        # Header qui s'étend sur toute la largeur
        header_frame = ctk.CTkFrame(self, fg_color="#6B8E23", corner_radius=0)
        header_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=0, pady=0)

        title_label = ctk.CTkLabel(
            header_frame, text="DRIVE PROTOCOL", font=ctk.CTkFont(size=20, weight="bold"), text_color="white"
        )
        title_label.pack(pady=15)

        # Frame gauche pour les contrôles (1/3 de l'écran)
        left_frame = ctk.CTkFrame(self)
        left_frame.grid(row=1, column=0, sticky="nsew", padx=(20, 10), pady=20)
        left_frame.grid_columnconfigure(0, weight=1)
        left_frame.grid_rowconfigure(1, weight=1)

        # Frame droite pour les futures visualisations (2/3 de l'écran)
        self.right_frame = ctk.CTkFrame(self)
        self.right_frame.grid(row=1, column=1, sticky="nsew", padx=(10, 20), pady=20)
        self.right_frame.grid_columnconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(0, weight=1)

        # Placeholder pour les futures visualisations
        placeholder_label = ctk.CTkLabel(
            self.right_frame,
            text="DRIVE Protocol Analysis\n\n(Future visualizations will appear here)",
            font=ctk.CTkFont(size=16, weight="bold"),
            text_color="gray",
        )
        placeholder_label.grid(row=0, column=0, padx=20, pady=20)

        #  "Name Of experience"
        name_frame = ctk.CTkFrame(left_frame)
        name_frame.grid(row=0, column=0, sticky="ew", padx=15, pady=(15, 10))
        name_frame.grid_columnconfigure(1, weight=1)  # Le champ de texte prend l'espace restant

        name_label = ctk.CTkLabel(name_frame, text="Name Of experience", font=ctk.CTkFont(size=14, weight="bold"))
        name_label.grid(row=0, column=0, padx=15, pady=15, sticky="w")

        self.experience_name_entry = ctk.CTkEntry(
            name_frame, placeholder_text="Enter experience name...", font=ctk.CTkFont(size=12), height=35
        )
        self.experience_name_entry.grid(row=0, column=1, padx=(10, 15), pady=15, sticky="ew")

        list_frame = ctk.CTkFrame(left_frame)
        list_frame.grid(row=1, column=0, sticky="nsew", padx=15, pady=10)
        list_frame.grid_columnconfigure(0, weight=1)
        list_frame.grid_rowconfigure(1, weight=1)

        click_label = ctk.CTkLabel(list_frame, text="Click on experience", font=ctk.CTkFont(size=12), text_color="gray")
        click_label.grid(row=0, column=0, padx=15, pady=(15, 5), sticky="w")

        self.experiences_scroll = ctk.CTkScrollableFrame(list_frame)
        self.experiences_scroll.grid(row=1, column=0, sticky="nsew", padx=15, pady=(5, 15))

        buttons_frame = ctk.CTkFrame(left_frame)
        buttons_frame.grid(row=2, column=0, sticky="ew", padx=15, pady=(10, 15))

        new_exp_button = ctk.CTkButton(
            buttons_frame,
            text="New Experience",
            command=self.new_experience_clicked,
            font=ctk.CTkFont(size=14, weight="bold"),
            height=40,
            fg_color="#87CEEB",
            hover_color="#70B8D1",
            text_color="black",
        )
        new_exp_button.pack(side="left", padx=15, pady=15)

        self.new_deploy_button = ctk.CTkButton(
            buttons_frame,
            text="New deployment",
            command=self.new_deployment_clicked,
            font=ctk.CTkFont(size=14, weight="bold"),
            height=40,
            fg_color="#D8BFD8",
            hover_color="#C8A8C8",
            text_color="black",
            state="disabled",
        )
        self.new_deploy_button.pack(side="left", padx=(10, 15), pady=15)

    def load_existing_experiences(self):
        for widget in self.experiences_scroll.winfo_children():
            widget.destroy()

        drive_datasets_path = Path(__file__).parent.parent.parent / "drive_datasets"

        if not drive_datasets_path.exists():
            no_exp_label = ctk.CTkLabel(
                self.experiences_scroll, text="No existing experiences found", font=ctk.CTkFont(size=12)
            )
            no_exp_label.pack(pady=20)
            return

        experiences = []
        for item in drive_datasets_path.iterdir():
            if item.is_dir():
                experiences.append(item)

        if not experiences:
            no_exp_label = ctk.CTkLabel(
                self.experiences_scroll, text="No existing experiences found", font=ctk.CTkFont(size=12)
            )
            no_exp_label.pack(pady=20)
            return

        print(f"Total experiences found: {len(experiences)}")

        experiences.sort(key=lambda x: x.name, reverse=True)

        for exp_path in experiences:
            self.create_experience_widget(exp_path)

    def create_experience_widget(self, exp_path):
        exp_frame = ctk.CTkFrame(self.experiences_scroll)
        exp_frame.pack(fill="x", padx=5, pady=2)

        is_selected = self.selected_experience and self.selected_experience.name == exp_path.name
        exp_button = ctk.CTkButton(
            exp_frame,
            text=exp_path.name,
            command=lambda: self.select_experience(exp_path),
            font=ctk.CTkFont(size=14, weight="bold"),
            height=30,
            anchor="w",
            fg_color="#4CAF50" if is_selected else "transparent",
            text_color="white" if is_selected else ("black", "white"),
            hover_color=("#45a049", "#45a049") if is_selected else ("#E0E0E0", "#404040"),
        )
        exp_button.pack(fill="x", padx=5, pady=2)

        content_frame = ctk.CTkFrame(exp_frame, fg_color="transparent")
        content_frame.pack(fill="x", padx=(20, 5), pady=(0, 5))

        deployments = []
        for item in exp_path.iterdir():
            if item.is_dir() and item.name.startswith("Deployment-"):
                deployments.append(item)

        deployments.sort(key=lambda x: int(x.name.split("-")[1]) if x.name.split("-")[1].isdigit() else 0, reverse=True)

        for deployment_path in deployments:
            deployment_frame = ctk.CTkFrame(content_frame, fg_color="transparent")
            deployment_frame.pack(fill="x", pady=1)

            deployment_label = ctk.CTkLabel(
                deployment_frame, text=f"→ {deployment_path.name}", font=ctk.CTkFont(size=12), anchor="w"
            )
            deployment_label.pack(side="left", padx=5)

            # Ajouter le menu contextuel (clic droit) sur le deployment
            metadata_folder = deployment_path / "Metadata"
            self.add_deployment_context_menu(deployment_label, deployment_path, metadata_folder)

            if metadata_folder.exists():
                metadata_frame = ctk.CTkFrame(content_frame, fg_color="transparent")
                metadata_frame.pack(fill="x", padx=(15, 0), pady=1)

                metadata_label = ctk.CTkLabel(
                    metadata_frame, text="  → Metadata/", font=ctk.CTkFont(size=11), anchor="w", text_color="#2E8B57"
                )
                metadata_label.pack(side="left", padx=5)

    def select_experience(self, exp_path):
        self.selected_experience = exp_path
        self.new_deploy_button.configure(state="normal")
        self.load_existing_experiences()

    def new_experience_clicked(self):
        self.experience_name_entry.focus()
        self.experience_name_entry.delete(0, "end")

        self.show_create_button()

        self.experience_name_entry.bind("<Return>", lambda e: self.create_experience())
        self.experience_name_entry.bind("<KeyRelease>", self.on_name_changed)

    def create_experience(self):
        experience_name = self.experience_name_entry.get().strip()
        if not experience_name:
            messagebox.showwarning("Error", "Please enter an experience name.")
            return

        drive_datasets_path = Path(__file__).parent.parent.parent / "drive_datasets"
        experience_path = drive_datasets_path / experience_name

        if experience_path.exists():
            messagebox.showwarning("Error", f"Experience '{experience_name}' already exists.")
            return

        try:
            experience_path.mkdir(parents=True, exist_ok=True)
            self.experience_name_entry.delete(0, "end")
            self.hide_create_button()
            self.selected_experience = experience_path
            self.new_deploy_button.configure(state="normal")
            self.after(100, self.refresh_experiences_list)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create experience: {str(e)}")

    def refresh_experiences_list(self):
        self.load_existing_experiences()
        self.update_idletasks()
        self.update()

    def show_create_button(self):
        if hasattr(self, "create_button") and self.create_button.winfo_exists():
            return

        name_frame = self.experience_name_entry.master

        self.create_button = ctk.CTkButton(
            name_frame,
            text="Create",
            command=self.create_experience,
            font=ctk.CTkFont(size=12, weight="bold"),
            height=35,
            width=80,
            fg_color="#2E8B57",
            hover_color="#20634A",
            text_color="white",
        )
        self.create_button.grid(row=0, column=2, padx=(10, 15), pady=15)

    def hide_create_button(self):
        if hasattr(self, "create_button") and self.create_button.winfo_exists():
            self.create_button.destroy()

    def on_name_changed(self, event):
        text = self.experience_name_entry.get().strip()
        if text:
            self.show_create_button()
        else:
            self.hide_create_button()

    def new_deployment_clicked(self):
        if not self.selected_experience:
            messagebox.showwarning("Error", "Please select an experience first before creating a deployment.")
            return

        try:
            deployment_number = 1
            while True:
                deployment_folder_name = f"Deployment-{deployment_number}"
                deployment_path = self.selected_experience / deployment_folder_name
                if not deployment_path.exists():
                    break
                deployment_number += 1

            deployment_path.mkdir(parents=True, exist_ok=True)

            deployment_metadata_path = deployment_path / "Metadata"
            deployment_metadata_path.mkdir(exist_ok=True)

            deployment_window = Deployment()
            deployment_window.focus()

            experience_template_path = Path(__file__).parent.parent / "Experience"

            if hasattr(deployment_window, "set_deployment_info"):
                deployment_window.set_deployment_info(
                    experience_name=self.selected_experience.name,
                    deployment_name=deployment_folder_name,
                    deployment_path=deployment_path,
                    deployment_metadata_path=deployment_metadata_path,
                    template_path=experience_template_path,
                )
            else:
                deployment_window.experience_name = self.selected_experience.name
                deployment_window.deployment_name = deployment_folder_name
                deployment_window.deployment_path = deployment_path
                deployment_window.deployment_metadata_path = deployment_metadata_path
                deployment_window.template_path = experience_template_path

            self.load_existing_experiences()

            print(f"Created deployment: {deployment_path}")
            print(f"Metadata folder: {deployment_metadata_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to create deployment: {str(e)}")

    def add_deployment_context_menu(self, deployment_label, deployment_path, metadata_folder):
        """Ajoute un menu contextuel au label de déploiement"""

        def on_right_click(event):
            context_menu = tk.Menu(self, tearoff=0)

            # Option d'édition des métadonnées (seulement si le dossier Metadata existe)
            if metadata_folder.exists():
                context_menu.add_command(
                    label="Edit Metadata",
                    command=lambda: self.edit_deployment_metadata(deployment_path, metadata_folder),
                )

            context_menu.add_separator()
            context_menu.add_command(label="Delete Deployment", command=lambda: self.delete_deployment(deployment_path))
            try:
                context_menu.tk_popup(event.x_root, event.y_root)
            finally:
                context_menu.grab_release()

        deployment_label.bind("<Button-3>", on_right_click)

        deployment_label.configure(cursor="hand2")

    def edit_deployment_metadata(self, deployment_path, metadata_folder):
        try:
            deployment_window = Deployment(allow_add=False, allow_edit=True)
            deployment_window.focus()

            experience_name = deployment_path.parent.name
            deployment_name = deployment_path.name

            experience_template_path = Path(__file__).parent.parent / "Experience"

            if hasattr(deployment_window, "set_deployment_info"):
                deployment_window.set_deployment_info(
                    experience_name=experience_name,
                    deployment_name=deployment_name,
                    deployment_path=deployment_path,
                    deployment_metadata_path=metadata_folder,
                    template_path=experience_template_path,
                )
            else:
                deployment_window.experience_name = experience_name
                deployment_window.deployment_name = deployment_name
                deployment_window.deployment_path = deployment_path
                deployment_window.deployment_metadata_path = metadata_folder
                deployment_window.template_path = experience_template_path

            self.load_existing_metadata_in_deployment(deployment_window, metadata_folder)

        except Exception as e:
            messagebox.showerror("Error", f"Failed to open deployment editor: {str(e)}")

    def load_existing_metadata_in_deployment(self, deployment_window, metadata_folder):
        try:
            roboticist_file = metadata_folder / "roboticists.json"
            if roboticist_file.exists():
                with open(roboticist_file, "r") as f:
                    roboticist_data = json.load(f)
                    if roboticist_data:
                        roboticist_name = f"{roboticist_data[0]['Name']} {roboticist_data[0]['Lastname']}"
                        deployment_window.combo_roboticist.set(roboticist_name)

            robot_file = metadata_folder / "robot.json"
            if robot_file.exists():
                with open(robot_file, "r") as f:
                    robot_data = json.load(f)
                    if robot_data:
                        robot_name = f"{robot_data[0]['robot']} (v{robot_data[0]['version']})"
                        deployment_window.combo_robot.set(robot_name)

            # Charger terrain
            ground_file = metadata_folder / "ground.json"
            if ground_file.exists():
                with open(ground_file, "r") as f:
                    ground_data = json.load(f)
                    if ground_data:
                        terrain_name = ground_data[0].get("name", "Unnamed")
                        deployment_window.combo_terrain.set(terrain_name)

        except Exception as e:
            print(f"Error loading existing metadata: {e}")

    def delete_deployment(self, deployment_path):
        result = messagebox.askyesno(
            "Delete Deployment",
            f"Are you sure you want to delete {deployment_path.name}?\nThis action cannot be undone.",
        )
        if result:
            try:
                import shutil

                shutil.rmtree(deployment_path)
                self.load_existing_experiences()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to delete deployment: {str(e)}")


if __name__ == "__main__":
    app = Home()
    app.mainloop()
