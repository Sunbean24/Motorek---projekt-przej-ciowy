import pandas as pd
import numpy as np
import glob
import os

# --- KONFIGURACJA ---
DANE_PATTERN = "BikeData_*.csv"  # Wzorzec szukania plików
DT = 0.01  # Sztywne 10ms taktowania z Arduino

# Szukanie wszystkich plików
pliki = glob.glob(DANE_PATTERN)
if not pliki:
    print(f"Błąd: Nie znaleziono plików dopasowanych do wzorca: {DANE_PATTERN}")
    print("Upewnij się, że uruchamiasz skrypt w folderze z plikami CSV pomiarów.")
    exit()

print(f"Znaleziono {len(pliki)} plików do przeanalizowania.\n")
print(f"{'Nazwa pliku CSV':<30} | {'Kor. prędkości':<15} | {'Kor. przysp.':<15} | {'Kor. grawitacji':<15}")
print("-" * 85)

vel_corrs = []
acc_corrs = []
gravity_corrs = []
odrzucone = 0

for sciezka_pliku in sorted(pliki):
    try:
        df = pd.read_csv(sciezka_pliku)
        
        # Ignoruj puste pliki lub zbyt krótkie serie (poniżej 5 próbek)
        if len(df) < 5:
            odrzucone += 1
            continue
            
        kat = df['angle'].values
        predkosc_mierzona = df['velocity'].values
        przyspieszenie_mierzone = df['acceleration'].values

        # 1. Analiza prędkości: d(Angle)/dt vs Żyroskop
        predkosc_wyliczona = np.diff(kat) / DT
        predkosc_mierzona_skrocona = predkosc_mierzona[:-1]
        korelacja_predkosci = np.corrcoef(predkosc_wyliczona, predkosc_mierzona_skrocona)[0, 1]

        # 2. Analiza przyspieszenia: d(Velocity)/dt vs Akcelerometr
        przyspieszenie_wyliczone = np.diff(predkosc_mierzona) / DT
        przyspieszenie_mierzone_skrocone = przyspieszenie_mierzone[:-1]
        korelacja_przyspieszenia = np.corrcoef(przyspieszenie_wyliczone, przyspieszenie_mierzone_skrocone)[0, 1]

        # 3. Analiza grawitacji: Akcelerometr vs składowa sin(Angle)
        korelacja_acc_vs_sin_kat = np.corrcoef(przyspieszenie_mierzone, np.sin(kat))[0, 1]

        # Zapisz do statystyk (pomijając ewentualne wartości NaN spowodowane brakiem ruchu)
        if not np.isnan(korelacja_predkosci): vel_corrs.append(korelacja_predkosci)
        if not np.isnan(korelacja_przyspieszenia): acc_corrs.append(korelacja_przyspieszenia)
        if not np.isnan(korelacja_acc_vs_sin_kat): gravity_corrs.append(korelacja_acc_vs_sin_kat)

        nazwa = os.path.basename(sciezka_pliku)
        print(f"{nazwa:<30} | {korelacja_predkosci:>15.4f} | {korelacja_przyspieszenia:>15.4f} | {korelacja_acc_vs_sin_kat:>15.4f}")
        
    except Exception as e:
        print(f"Błąd przetwarzania pliku {os.path.basename(sciezka_pliku)}: {e}")

# --- GLOBALNE PODSUMOWANIE STATYSTYCZNE ---
print("\n" + "="*60)
print("               GLOBALNY RAPORT INTEGRALNOŚCI FIZYKI")
print("="*60)

if odrzucone > 0:
    print(f"Info: Zignorowano {odrzucone} pustych/uszkodzonych plików CSV.\n")

if vel_corrs:
    srednia_vel = np.mean(vel_corrs)
    print(f"1. Średnia korelacja d(Angle)/dt vs Żyroskop: {srednia_vel:.4f}")
    if srednia_vel > 0.6:
        print("   [OK]   Znak prędkości w Arduino jest poprawny we wszystkich plikach.")
    elif srednia_vel < -0.6:
        print("   [BŁĄD] Znak prędkości jest odwrócony! Musisz dopisać minus przy gyro.y() w Arduino.")
    else:
        print("   [!]    Niejednorodne wyniki prędkości. Upewnij się, że motocykl ruszał się podczas testów.")

if gravity_corrs:
    srednia_grav = np.mean(gravity_corrs)
    print(f"\n2. Średnia korelacja Akcelerometr vs sin(Angle): {srednia_grav:.4f}")
    if srednia_grav > 0.6:
        print("   [OK]   Akcelerometr prawidłowo współgra z grawitacją.")
    elif srednia_grav < -0.6:
        print("   [BŁĄD] Znak akcelerometru jest odwrotny! Zmień znak przy accel.y() w Arduino.")
    else:
        print("   [!]    Niejednorodne wyniki akcelerometru. Sprawdź, czy oś Y czujnika nie jest zamieniona z inną osią.")

if acc_corrs:
    srednia_acc = np.mean(acc_corrs)
    print(f"\n3. Średnia korelacja d(Velocity)/dt vs Akcelerometr: {srednia_acc:.4f}")
    print("   (Wartość informacyjna o poziomie szumu i opóźnieniu różniczkowania dynamicznego)")
print("="*60)
