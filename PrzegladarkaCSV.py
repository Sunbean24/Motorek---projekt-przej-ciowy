import tkinter as tk
from tkinter import filedialog, messagebox
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import os


class CSVViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Zaawansowany Analizator Telemetrii Motorka")
        self.root.geometry("1200x800")

        self.current_folder = ""
        self.file_list = []
        self.current_file_path = None

        # Definicja "Ground Truth" dla checkboxów w interfejsie
        self.data_cols = ['angle', 'velocity', 'acceleration', 'wheel_angle', 'pwm', 'user_free']

        # Słownik aliasów do obsługi starszych plików
        self.aliases = {
            'angle': ['angle', 'angle_rad'],
            'wheel_angle': ['wheel_angle', 'wheel_vel'],
            'pwm': ['pwm', 'fill']
        }

        self.setup_ui()

    def setup_ui(self):
        # --- LEWY PANEL (Lista plików) ---
        left_frame = tk.Frame(self.root, width=250, bg="#f0f0f0")
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        btn_load = tk.Button(left_frame, text="Wybierz folder z CSV", command=self.load_folder, bg="#4CAF50",
                             fg="white", font=("Arial", 10, "bold"))
        btn_load.pack(fill=tk.X, pady=(0, 10))

        scrollbar = tk.Scrollbar(left_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.listbox = tk.Listbox(left_frame, yscrollcommand=scrollbar.set, font=("Arial", 10),
                                  selectbackground="#4CAF50")
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.listbox.yview)

        self.listbox.bind('<<ListboxSelect>>', self.on_file_select)

        # --- ŚRODKOWY PANEL (Wybór danych do wyświetlenia) ---
        middle_frame = tk.Frame(self.root, width=150, bg="#e0e0e0", padx=10, pady=10)
        middle_frame.pack(side=tk.LEFT, fill=tk.Y)

        tk.Label(middle_frame, text="Wybierz wykresy:", font=("Arial", 11, "bold"), bg="#e0e0e0").pack(anchor="w",
                                                                                                       pady=(0, 10))

        self.check_vars = {}
        for col in self.data_cols:
            default_val = True if col in ['angle', 'pwm'] else False
            var = tk.BooleanVar(value=default_val)
            chk = tk.Checkbutton(middle_frame, text=col, variable=var, bg="#e0e0e0",
                                 font=("Arial", 10), command=self.update_plot)
            chk.pack(anchor="w", pady=2)
            self.check_vars[col] = var

        # --- PRAWY PANEL (Wykresy i Toolbar) ---
        self.right_frame = tk.Frame(self.root)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = plt.figure(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)

        toolbar_frame = tk.Frame(self.right_frame)
        toolbar_frame.pack(side=tk.TOP, fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()

        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def load_folder(self):
        folder_selected = filedialog.askdirectory()
        if folder_selected:
            self.current_folder = folder_selected
            self.update_file_list()

    def update_file_list(self):
        self.listbox.delete(0, tk.END)
        self.file_list = [f for f in os.listdir(self.current_folder) if f.endswith('.csv')]
        self.file_list.sort()

        if not self.file_list:
            messagebox.showinfo("Brak plików", "W wybranym folderze nie ma plików .csv")
            return

        for f in self.file_list:
            self.listbox.insert(tk.END, f)

    def on_file_select(self, event):
        selection = self.listbox.curselection()
        if not selection:
            return

        file_name = self.listbox.get(selection[0])
        self.current_file_path = os.path.join(self.current_folder, file_name)

        self.update_plot()

    def find_actual_column(self, df, requested_col):
        """Zwraca faktyczną nazwę kolumny w pliku na podstawie aliasów"""
        if requested_col in df.columns:
            return requested_col

        if requested_col in self.aliases:
            for alias in self.aliases[requested_col]:
                if alias in df.columns:
                    return alias
        return None

    def update_plot(self):
        if not self.current_file_path:
            return

        try:
            df = pd.read_csv(self.current_file_path)

            # Elastyczne szukanie kolumny czasu
            time_col = None
            for c in df.columns:
                if 'time' in c.lower() or 'timestamp' in c.lower():
                    time_col = c
                    break

            if not time_col:
                time_col = df.columns[0]  # Jeśli naprawdę dziwny plik, bierzemy 1. kolumnę

            active_cols = [col for col, var in self.check_vars.items() if var.get()]
            num_plots = len(active_cols)

            self.fig.clear()

            if num_plots == 0:
                self.canvas.draw_idle()
                return

            axs = self.fig.subplots(num_plots, 1, sharex=True)
            if num_plots == 1:
                axs = [axs]

                # Pobieranie i normalizacja czasu
            t = df[time_col].values

            # Przeliczanie ms na sekundy, jeśli to stary format pliku
            if t[0] > 1000000 or 'ms' in time_col.lower():
                t = (t - t[0]) / 1000.0
            else:
                t = t - t[0]

            for i, standard_col in enumerate(active_cols):
                ax = axs[i]

                # Ustalenie jak kolumna faktycznie nazywa się w tym konkretnym pliku
                actual_col = self.find_actual_column(df, standard_col)

                if actual_col:
                    if standard_col == 'user_free':
                        ax.step(t, df[actual_col].values, 'g-', where='post', label=standard_col)
                        ax.set_ylim(-0.1, 1.1)
                        ax.set_yticks([0, 1])
                    else:
                        color = 'b-' if standard_col == 'angle' else ('r-' if standard_col == 'pwm' else 'k-')
                        ax.plot(t, df[actual_col].values, color, label=standard_col)

                    ax.set_ylabel(standard_col)
                    ax.legend(loc="upper right", fontsize="small")
                    ax.grid(True, linestyle='--', alpha=0.7)
                else:
                    # Brak danych mimo aliasów (np. stary plik nie miał acceleration)
                    ax.text(0.5, 0.5, f"Brak danych: {standard_col}", ha='center', va='center', color='red')
                    ax.set_yticks([])

            axs[-1].set_xlabel("Czas względny [s]")

            file_name = os.path.basename(self.current_file_path)
            axs[0].set_title(f"Analiza pliku: {file_name}", fontsize=12, fontweight='bold')

            self.fig.tight_layout()
            self.canvas.draw_idle()

        except Exception as e:
            messagebox.showerror("Błąd odczytu", f"Nie udało się wczytać pliku.\nBłąd: {str(e)}")


if __name__ == "__main__":
    root = tk.Tk()
    app = CSVViewerApp(root)
    root.mainloop()