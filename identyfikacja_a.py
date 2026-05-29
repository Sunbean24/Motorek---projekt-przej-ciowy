import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. KONFIGURACJA
# ==========================================
# Wpisz nazwę swojego pliku CSV z nagrania upadku
PLIK_CSV = "BikeData_20260521_194414.csv"

# Wybierz okno czasowe, w którym motorek swobodnie spadał (w sekundach)
# Ważne: Wybierz fragment, gdy kąt rośnie, ale ZANIM uderzy o podpórki!
CZAS_START = 2.1
CZAS_KONIEC = 2.6

# ==========================================
# 2. WCZYTANIE I OBRÓBKA DANYCH
# ==========================================
try:
    df = pd.read_csv(PLIK_CSV)
except FileNotFoundError:
    print(f"Błąd: Nie znaleziono pliku {PLIK_CSV}!")
    exit()

# Filtrowanie danych do wybranego okna czasowego
df_fall = df[(df['timestamp(s)'] >= CZAS_START) & (df['timestamp(s)'] <= CZAS_KONIEC)].copy()

if df_fall.empty:
    print("Błąd: Brak danych w wybranym przedziale czasowym. Sprawdź CZAS_START i CZAS_KONIEC.")
    exit()

# Normalizacja czasu (żeby upadek zaczynał się od t = 0)
t = df_fall['timestamp(s)'].values
t = t - t[0]

# Pobranie kąta. Bierzemy wartość bezwzględną (upadek może być na minus lub na plus),
# a logarytm można wyciągnąć tylko z liczby dodatniej.
# Dodajemy minimalną wartość (1e-9), żeby uniknąć błędu log(0) gdy kąt jest idealnie zerowy.
phi = np.abs(df_fall['angle'].values) + 1e-9

# ==========================================
# 3. MATEMATYKA (MNK i LOGARYTM)
# ==========================================
# y = ln(phi)
y = np.log(phi)

# Dopasowanie prostej y = m*t + c za pomocą Metody Najmniejszych Kwadratów (polyfit stopnia 1)
wspolczynniki = np.polyfit(t, y, 1)
m = wspolczynniki[0] # To jest nasze nachylenie, czyli sqrt(a)
c = wspolczynniki[1] # To jest stała ln(C)

# Obliczenie poszukiwanego parametru 'a'
a = m**2

# Generowanie idealnej prostej do nałożenia na wykres
y_fit = m * t + c

# ==========================================
# 4. WYŚWIETLANIE WYNIKÓW W TERMINALU
# ==========================================
print("\n" + "="*50)
print(" WYNIKI IDENTYFIKACJI SWOBODNEGO SPADKU (MNK)")
print("="*50)
print(f" Liczba próbek w oknie  : {len(t)}")
print(f" Nachylenie prostej (m) : {m:.4f}  [= sqrt(a)]")
print(f" Punkt przecięcia (c)   : {c:.4f}  [= ln(C)]")
print("-" * 50)
print(f" WYESTYMOWANY PARAMETR 'a' : {a:.4f}")
print("="*50 + "\n")

# ==========================================
# 5. GENEROWANIE WYKRESÓW DO RAPORTU LATEX
# ==========================================
# Ustawienie ładniejszego stylu wykresów
plt.style.use('bmh')
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# --- Wykres 1: Rzeczywisty kąt w czasie ---
ax1.plot(t, phi, 'b.-', label='Pomiary (moduł kąta)')
ax1.set_title("Oryginalny przebieg upadku")
ax1.set_xlabel("Czas [s]")
ax1.set_ylabel(r"Kąt $|\phi|$ [rad]")
ax1.legend()

# --- Wykres 2: Zlogarytmowany kąt i dopasowana prosta ---
ax2.plot(t, y, 'ko', label=r'Pomiary: $\ln(|\phi|)$')
ax2.plot(t, y_fit, 'r-', linewidth=2, label=f'Prosta MNK: y = {m:.2f}t + {c:.2f}')
ax2.set_title("Linearyzacja logarytmiczna i MNK")
ax2.set_xlabel("Czas [s]")
ax2.set_ylabel(r"$\ln(|\phi|)$")
ax2.legend()

plt.tight_layout()

# Zapis do wektorowego PDF - idealnego do includowania w LaTeX
nazwa_wykresu = "wykres_identyfikacja_a.pdf"
plt.savefig(nazwa_wykresu, format='pdf', bbox_inches='tight')
print(f"Pomyślnie wygenerowano plik z wykresem: {nazwa_wykresu}")

plt.show()