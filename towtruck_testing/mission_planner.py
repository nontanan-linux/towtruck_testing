import customtkinter as ctk
from PIL import Image, ImageTk


class PickupDropApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Pick-up & Drop Frame")
        self.geometry("700x400")
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")
        self.create_main_frame()

    def create_main_frame(self):
        main_frame = ctk.CTkFrame(self)
        main_frame.pack(padx=20, pady=20, fill="both", expand=True)
        self.create_pickup_frame(main_frame)
        # self.create_drop_frame(main_frame)

    def create_pickup_frame(self, parent):
        frame_pickup = ctk.CTkFrame(parent)
        frame_pickup.pack(side="left", padx=10, pady=10, fill="both", expand=True)
        ctk.CTkLabel(frame_pickup, text="Pick-up", font=ctk.CTkFont(size=20, weight="bold")).pack(pady=10)
        # Load Image
        pickup_img = ctk.CTkImage(Image.open("/home/nontanan/Pictures/gensurv-logo.jpeg"), size=(150, 150))
        ctk.CTkLabel(frame_pickup, image=pickup_img, text="").pack(pady=5)
        self.pickup_entry = ctk.CTkEntry(frame_pickup, placeholder_text="Pick-up location")
        self.pickup_entry.pack(pady=10)
        btn_frame = ctk.CTkFrame(frame_pickup)
        btn_frame.pack(pady=10)
        for name in ["P01S", "P02S", "P03S", "P04S"]:
            ctk.CTkButton(btn_frame, text=name, width=70, command=lambda n=name: self.pickup_action(n)).pack(side="left", padx=5)

    def create_drop_frame(self, parent):
        frame_drop = ctk.CTkFrame(parent)
        frame_drop.pack(side="right", padx=10, pady=10, fill="both", expand=True)
        ctk.CTkLabel(frame_drop, text="Drop", font=ctk.CTkFont(size=20, weight="bold")).pack(pady=10)
        # Load Image
        drop_img = ctk.CTkImage(Image.open("/home/nontanan/Pictures/gensurv-logo.jpeg"), size=(150, 150))
        ctk.CTkLabel(frame_drop, image=drop_img, text="").pack(pady=5)
        self.drop_entry = ctk.CTkEntry(frame_drop, placeholder_text="Drop location")
        self.drop_entry.pack(pady=10)
        btn_frame = ctk.CTkFrame(frame_drop)
        btn_frame.pack(pady=10)
        for name in ["D01S", "D02S", "D03S", "D04S"]:
            ctk.CTkButton(btn_frame, text=name, width=70, command=lambda n=name: self.drop_action(n)).pack(side="left", padx=5)

    def pickup_action(self, name):
        print(f"Pickup Selected: {name}")

    def drop_action(self, name):
        print(f"Drop Selected: {name}")

if __name__ == "__main__":
    app = PickupDropApp()
    app.mainloop()
