import tkinter as tk
from tkinter import messagebox
import customtkinter as ctk
import json
from datetime import datetime
from pathlib import Path
from DRIVE_GUI.deployment import Deployment


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
        self.selected_deployment = None

        self.setup_ui()
        self.load_existing_experiences()

    def setup_ui(self):
        self.grid_columnconfigure(0, weight=1, minsize=400)
        self.grid_columnconfigure(1, weight=2, minsize=800)
        self.grid_rowconfigure(1, weight=1)

        header_frame = ctk.CTkFrame(self, fg_color="#6B8E23", corner_radius=0)
        header_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=0, pady=0)
        header_frame.grid_columnconfigure(1, weight=1)

        # Bouton Home à gauche
        home_button = ctk.CTkButton(
            header_frame,
            text="🏠 Home",
            command=self.show_home_page,
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color="transparent",
            hover_color="#5A7A1A",
            text_color="white",
            width=100,
        )
        home_button.grid(row=0, column=0, padx=20, pady=15, sticky="w")

        # Titre au centre
        title_label = ctk.CTkLabel(
            header_frame, text="DRIVE PROTOCOL", font=ctk.CTkFont(size=20, weight="bold"), text_color="white"
        )
        title_label.grid(row=0, column=1, pady=15)

        left_frame = ctk.CTkFrame(self)
        left_frame.grid(row=1, column=0, sticky="nsew", padx=(20, 10), pady=20)
        left_frame.grid_columnconfigure(0, weight=1)
        left_frame.grid_rowconfigure(1, weight=1)

        self.right_frame = ctk.CTkFrame(self)
        self.right_frame.grid(row=1, column=1, sticky="nsew", padx=(10, 20), pady=20)
        self.right_frame.grid_columnconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(0, weight=1)

        self.active_right_content = None

        self.show_home_page()

        name_frame = ctk.CTkFrame(left_frame)
        name_frame.grid(row=0, column=0, sticky="ew", padx=15, pady=(15, 10))
        name_frame.grid_columnconfigure(1, weight=0)
        name_frame.grid_columnconfigure(2, weight=0)

        name_label = ctk.CTkLabel(name_frame, text="Name Of experience", font=ctk.CTkFont(size=14, weight="bold"))
        name_label.grid(row=0, column=0, padx=15, pady=15, sticky="w")

        self.experience_name_entry = ctk.CTkEntry(
            name_frame, placeholder_text="Enter experience name...", font=ctk.CTkFont(size=12), height=35, width=200
        )
        self.experience_name_entry.grid(row=0, column=1, padx=(10, 10), pady=15)

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
        self.create_button.grid(row=0, column=2, padx=(0, 15), pady=15)

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
        self.new_deploy_button.pack(side="left", padx=15, pady=15)

    def show_placeholder(self):
        self.clear_right_frame()
        placeholder_label = ctk.CTkLabel(
            self.right_frame,
            text="DRIVE Protocol Analysis\n\n(Future visualizations will appear here)",
            font=ctk.CTkFont(size=16, weight="bold"),
            text_color="gray",
        )
        placeholder_label.grid(row=0, column=0, padx=20, pady=20)
        self.active_right_content = placeholder_label

    def show_home_page(self):
        self.clear_right_frame()

        home_frame = ctk.CTkScrollableFrame(self.right_frame)
        home_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        home_frame.grid_columnconfigure(0, weight=1)

        title = ctk.CTkLabel(
            home_frame, text="Welcome to DRIVE Protocol", font=ctk.CTkFont(size=28, weight="bold"), text_color="#6B8E23"
        )
        title.grid(row=0, column=0, pady=(20, 10), sticky="ew")

        subtitle = ctk.CTkLabel(
            home_frame, text="Dataset for Drive Protocol", font=ctk.CTkFont(size=16, slant="italic"), text_color="gray"
        )
        subtitle.grid(row=1, column=0, pady=(0, 30), sticky="ew")

        # Section: Getting Started
        getting_started_label = ctk.CTkLabel(
            home_frame, text="🚀 Getting Started", font=ctk.CTkFont(size=20, weight="bold"), anchor="w"
        )
        getting_started_label.grid(row=2, column=0, pady=(10, 5), sticky="w", padx=20)

        getting_started_text = ctk.CTkLabel(
            home_frame,
            text="1. Create a New Experience to organize your robot experiments\n"
            "2. Add Deployments to each experience to track different test runs\n"
            "3. Fill in the metadata forms (Roboticist, Robot, and Terrain)\n"
            "4. Launch DRIVE to start collecting data",
            font=ctk.CTkFont(size=14),
            anchor="w",
            justify="left",
        )
        getting_started_text.grid(row=3, column=0, pady=(0, 20), sticky="w", padx=40)

        # Section: What is DRIVE?
        what_is_label = ctk.CTkLabel(
            home_frame, text="📖 What is DRIVE?", font=ctk.CTkFont(size=20, weight="bold"), anchor="w"
        )
        what_is_label.grid(row=4, column=0, pady=(10, 5), sticky="w", padx=20)

        what_is_text = ctk.CTkLabel(
            home_frame,
            text="DRIVE is a standardized protocol for collecting robot navigation datasets.\n"
            "It helps researchers gather consistent, high-quality data about robot\n"
            "performance across different terrains, conditions, and platforms.",
            font=ctk.CTkFont(size=14),
            anchor="w",
            justify="left",
        )
        what_is_text.grid(row=5, column=0, pady=(0, 20), sticky="w", padx=40)

        # Section: Key Features
        features_label = ctk.CTkLabel(
            home_frame, text="✨ Key Features", font=ctk.CTkFont(size=20, weight="bold"), anchor="w"
        )
        features_label.grid(row=6, column=0, pady=(10, 5), sticky="w", padx=20)

        features_frame = ctk.CTkFrame(home_frame, fg_color="transparent")
        features_frame.grid(row=7, column=0, pady=(0, 20), sticky="ew", padx=40)
        features_frame.grid_columnconfigure(0, weight=1)

        features = [
            ("📊", "Standardized Data Collection", "Collect consistent data across experiments"),
            ("🤖", "Robot Metadata", "Track robot specifications and configurations"),
            ("🌍", "Terrain Documentation", "Document terrain conditions with photos"),
            ("👥", "Team Collaboration", "Organize experiments by team members"),
            ("📈", "Data Analysis", "Visualize and analyze collected data"),
        ]

        for idx, (icon, title, desc) in enumerate(features):
            feature_frame = ctk.CTkFrame(features_frame, fg_color="#F0F0F0")
            feature_frame.grid(row=idx, column=0, pady=5, sticky="ew", padx=5)
            feature_frame.grid_columnconfigure(1, weight=1)

            icon_label = ctk.CTkLabel(feature_frame, text=icon, font=ctk.CTkFont(size=24))
            icon_label.grid(row=0, column=0, rowspan=2, padx=15, pady=10)

            title_label = ctk.CTkLabel(feature_frame, text=title, font=ctk.CTkFont(size=14, weight="bold"), anchor="w")
            title_label.grid(row=0, column=1, sticky="w", padx=(0, 15), pady=(10, 0))

            desc_label = ctk.CTkLabel(
                feature_frame, text=desc, font=ctk.CTkFont(size=12), anchor="w", text_color="gray"
            )
            desc_label.grid(row=1, column=1, sticky="w", padx=(0, 15), pady=(0, 10))

        # Section: Quick Actions
        actions_label = ctk.CTkLabel(
            home_frame, text="⚡ Quick Actions", font=ctk.CTkFont(size=20, weight="bold"), anchor="w"
        )
        actions_label.grid(row=8, column=0, pady=(20, 10), sticky="w", padx=20)

        actions_frame = ctk.CTkFrame(home_frame, fg_color="transparent")
        actions_frame.grid(row=9, column=0, pady=(0, 30), sticky="ew", padx=40)

        new_exp_btn = ctk.CTkButton(
            actions_frame,
            text="➕ Create New Experience",
            command=self.new_experience_clicked,
            font=ctk.CTkFont(size=14, weight="bold"),
            height=45,
            fg_color="#87CEEB",
            hover_color="#70B8D1",
            text_color="black",
        )
        new_exp_btn.pack(fill="x", pady=5)

        # Footer
        footer = ctk.CTkLabel(
            home_frame,
            text="Need help? Check the documentation or contact the NORLAB team.",
            font=ctk.CTkFont(size=12),
            text_color="gray",
        )
        footer.grid(row=10, column=0, pady=(20, 20), sticky="ew")

        self.active_right_content = home_frame

    def clear_right_frame(self):
        """Nettoie le contenu du panneau de droite"""
        if self.active_right_content:
            if hasattr(self.active_right_content, "destroy"):
                self.active_right_content.destroy()
            self.active_right_content = None

        for widget in self.right_frame.winfo_children():
            widget.destroy()

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

        deployments = []
        for item in exp_path.iterdir():
            if item.is_dir() and item.name.startswith("Deployment-"):
                deployments.append(item)

        if deployments:
            content_frame = ctk.CTkFrame(exp_frame, fg_color="transparent")
            content_frame.pack(fill="x", padx=(20, 5), pady=(0, 5))

            deployments.sort(
                key=lambda x: int(x.name.split("-")[1]) if x.name.split("-")[1].isdigit() else 0, reverse=True
            )

            for deployment_path in deployments:
                deployment_frame = ctk.CTkFrame(content_frame, fg_color="transparent")
                deployment_frame.pack(fill="x", pady=0)

                metadata_folder = deployment_path / "Metadata"

                # Vérifier si ce déploiement est sélectionné
                is_deployment_selected = (
                    self.selected_deployment
                    and self.selected_deployment.name == deployment_path.name
                    and self.selected_deployment.parent.name == exp_path.name
                )

                deployment_button = ctk.CTkButton(
                    deployment_frame,
                    text=f"→ {deployment_path.name}",
                    font=ctk.CTkFont(size=12),
                    anchor="w",
                    fg_color="#87CEEB" if is_deployment_selected else "transparent",
                    text_color="black" if is_deployment_selected else ("black", "white"),
                    hover_color=("#70B8D1", "#70B8D1") if is_deployment_selected else ("#E0E0E0", "#404040"),
                    command=lambda dp=deployment_path: self.open_deployment_view(dp),
                )
                deployment_button.pack(side="left", fill="x", expand=True, padx=5)

                self.add_deployment_context_menu(deployment_button, deployment_path, metadata_folder)

    def select_experience(self, exp_path):
        self.selected_experience = exp_path
        self.new_deploy_button.configure(state="normal")
        self.load_existing_experiences()

    def new_experience_clicked(self):
        self.experience_name_entry.focus()
        self.experience_name_entry.delete(0, "end")

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
            self.selected_experience = experience_path
            self.new_deploy_button.configure(state="normal")
            self.after(100, self.refresh_experiences_list)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create experience: {str(e)}")

    def refresh_experiences_list(self):
        self.load_existing_experiences()
        self.update_idletasks()
        self.update()

    def open_deployment_view(self, deployment_path):
        metadata_folder = deployment_path / "Metadata"

        if not metadata_folder.exists():
            messagebox.showinfo(
                "No Metadata",
                f"No metadata found for {deployment_path.name}.\nPlease create metadata first by editing this deployment.",
            )
            return

        self.selected_deployment = deployment_path
        self.load_existing_experiences()

        self.show_deployment_in_right_panel(deployment_path, metadata_folder, allow_add=False, allow_edit=True)

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

            self.show_deployment_in_right_panel(
                deployment_path, deployment_metadata_path, allow_add=True, allow_edit=True, is_new=True
            )

            self.load_existing_experiences()

        except Exception as e:
            messagebox.showerror("Error", f"Failed to create deployment: {str(e)}")

    def show_deployment_in_right_panel(
        self, deployment_path, metadata_folder, allow_add=True, allow_edit=True, is_new=False
    ):
        self.clear_right_frame()

        deployment_frame = Deployment(self.right_frame, allow_add=allow_add, allow_edit=allow_edit)
        deployment_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        experience_name = deployment_path.parent.name
        deployment_name = deployment_path.name
        experience_template_path = Path(__file__).parent.parent / "Experience"

        deployment_frame.set_deployment_info(
            experience_name=experience_name,
            deployment_name=deployment_name,
            deployment_path=deployment_path,
            deployment_metadata_path=metadata_folder,
            template_path=experience_template_path,
        )

        if metadata_folder.exists() and not is_new:
            self.load_existing_metadata_in_deployment(deployment_frame, metadata_folder)
        elif is_new:
            deployment_frame.combo_roboticist.set("")
            deployment_frame.combo_robot.set("")
            deployment_frame.combo_terrain.set("")

        self.active_right_content = deployment_frame

    def add_deployment_context_menu(self, deployment_label, deployment_path, metadata_folder):
        def on_right_click(event):
            context_menu = tk.Menu(self, tearoff=0)

            if metadata_folder.exists():
                context_menu.add_command(
                    label="Edit Metadata",
                    command=lambda: self.show_deployment_in_right_panel(
                        deployment_path, metadata_folder, allow_add=False, allow_edit=True
                    ),
                )

            context_menu.add_separator()
            context_menu.add_command(label="Delete Deployment", command=lambda: self.delete_deployment(deployment_path))
            try:
                context_menu.tk_popup(event.x_root, event.y_root)
            finally:
                context_menu.grab_release()

        deployment_label.bind("<Button-3>", on_right_click)

        deployment_label.configure(cursor="hand2")

    def load_existing_metadata_in_deployment(self, deployment_frame, metadata_folder):
        try:
            roboticist_file = metadata_folder / "roboticists.json"
            if roboticist_file.exists():
                with open(roboticist_file, "r") as f:
                    roboticist_data = json.load(f)
                    if roboticist_data:
                        roboticist_name = f"{roboticist_data[0]['Name']} {roboticist_data[0]['Lastname']}"
                        deployment_frame.combo_roboticist.set(roboticist_name)

            robot_file = metadata_folder / "robot.json"
            if robot_file.exists():
                with open(robot_file, "r") as f:
                    robot_data = json.load(f)
                    if robot_data:
                        robot_name = f"{robot_data[0]['robot']} (v{robot_data[0]['version']})"
                        deployment_frame.combo_robot.set(robot_name)

            ground_file = metadata_folder / "ground.json"
            if ground_file.exists():
                with open(ground_file, "r") as f:
                    ground_data = json.load(f)
                    if ground_data:
                        terrain_name = ground_data[0].get("name", "Unnamed")
                        deployment_frame.combo_terrain.set(terrain_name)

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
