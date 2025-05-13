import customtkinter as ctk
from screeninfo import get_monitors
import tkinter as tk

class App(ctk.CTk):
    def __init__(self):
        super().__init__()

        # === Window Config ===
        self.overrideredirect(True)
        self.geometry("800x500+100+100")
        self.fullscreen = False
        self.configure(bg="#2b2b2b")
        self.withdrawn = False

        # === Custom Title Bar ===
        self.title_bar = ctk.CTkFrame(self, height=30, corner_radius=0)
        self.title_bar.pack(fill="x", side="top")
        self.title_bar.bind("<B1-Motion>", self.move_window)
        self.title_bar.bind("<Button-1>", self.get_pos)

        self.title_label = ctk.CTkLabel(self.title_bar, text=" Custom App", anchor="w")
        self.title_label.pack(side="left", padx=10)

        # === Window Controls ===
        self.hide_button = ctk.CTkButton(self.title_bar, text="-", width=30, command=self.hide_window)
        self.hide_button.pack(side="right", padx=(0, 2))

        self.maximize_button = ctk.CTkButton(self.title_bar, text="🗖", width=30, command=self.toggle_fullscreen)
        self.maximize_button.pack(side="right", padx=(0, 2))

        self.close_button = ctk.CTkButton(self.title_bar, text="X", width=30, fg_color="red", command=self.quit)
        self.close_button.pack(side="right", padx=(0, 2))

        # === Main Content ===
        self.content_frame = ctk.CTkFrame(self, corner_radius=0)
        self.content_frame.pack(expand=True, fill="both")

        # === Editable Input Textbox ===
        self.textbox = ctk.CTkTextbox(self.content_frame, height=5, wrap="word", state="normal")
        self.textbox.pack(side="left", expand=True, fill="both", padx=10, pady=10)

        # === Button to send the message ===
        self.button = ctk.CTkButton(self.content_frame, text="Send Message", command=self.send_message)
        self.button.pack(side="right", padx=10, pady=10)

        # === Display sent messages ===
        self.messages_frame = ctk.CTkFrame(self.content_frame)
        self.messages_frame.pack(side="bottom", fill="both", expand=True, padx=10, pady=10)

        # === Key Binding to restore hidden window ===
        self.bind("<Control-Shift-H>", lambda e: self.restore_window())

    def send_message(self):
        # Get the text entered in the textbox
        entered_text = self.textbox.get("1.0", "end-1c").strip()  # Fetch and strip any trailing newline

        if entered_text:  # Check if text is not empty
            message_label = ctk.CTkLabel(self.messages_frame, text=entered_text, anchor="w", justify="left", font=("Arial", 12))
            message_label.pack(fill="x", padx=10, pady=5)

            # Clear the textbox after sending
            self.textbox.delete("1.0", "end")

    def hide_window(self):
        self.withdraw()
        self.withdrawn = True

    def restore_window(self):
        if self.withdrawn:
            self.deiconify()
            self.withdrawn = False

    def toggle_fullscreen(self):
        if not self.fullscreen:
            x = self.winfo_x() + self.winfo_width() // 2
            y = self.winfo_y() + self.winfo_height() // 2
            for monitor in get_monitors():
                if monitor.x <= x < monitor.x + monitor.width and monitor.y <= y < monitor.y + monitor.height:
                    self.geometry(f"{monitor.width}x{monitor.height}+{monitor.x}+{monitor.y}")
                    break
        else:
            self.geometry("800x500+100+100")
        self.fullscreen = not self.fullscreen

    def get_pos(self, event):
        self._offsetx = event.x
        self._offsety = event.y

    def move_window(self, event):
        x = event.x_root - self._offsetx
        y = event.y_root - self._offsety
        self.geometry(f"+{x}+{y}")

if __name__ == "__main__":
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")
    app = App()
    app.mainloop()
