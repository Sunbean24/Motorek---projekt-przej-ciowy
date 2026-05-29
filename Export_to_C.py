import torch
from stable_baselines3 import PPO

# 1. Wczytaj wyuczonego agenta (wymuszamy CPU, bo to tylko odczyt wag)
print("Wczytywanie modelu...")
model = PPO.load("bike_ppo_robust", device="cpu")

# 2. Dobieramy się do surowych wag sieci "Aktora" (Policy)
weights = model.policy.state_dict()

# Wagi dla warstwy 1, 2 i wyjściowej
W0 = weights['mlp_extractor.policy_net.0.weight'].numpy()
b0 = weights['mlp_extractor.policy_net.0.bias'].numpy()
W1 = weights['mlp_extractor.policy_net.2.weight'].numpy()
b1 = weights['mlp_extractor.policy_net.2.bias'].numpy()
W2 = weights['action_net.weight'].numpy()
b2 = weights['action_net.bias'].numpy()


# Funkcja pomocnicza do formatowania macierzy w C++
def print_matrix(name, mat):
    if len(mat.shape) == 1:
        res = f"const float {name}[{mat.shape[0]}] = {{"
        res += ", ".join([f"{x:.6f}" for x in mat])
        res += "};\n"
        return res
    else:
        res = f"const float {name}[{mat.shape[0]}][{mat.shape[1]}] = {{\n"
        for row in mat:
            res += "  {" + ", ".join([f"{x:.6f}" for x in row]) + "},\n"
        res += "};\n"
        return res


# 3. Zrzucamy to do pliku nagłówkowego C++
print("Generowanie pliku AI_Controller.h...")
with open("AI_Controller.h", "w") as f:
    f.write("// --- WAGI SIECI NEURONOWEJ (AI PPO) ---\n\n")
    f.write(print_matrix("W0", W0))
    f.write(print_matrix("b0", b0))
    f.write(print_matrix("W1", W1))
    f.write(print_matrix("b1", b1))
    f.write(print_matrix("W2", W2))
    f.write(print_matrix("b2", b2))

    # Doklejamy od razu logikę sieci neuronowej w czystym C
    cpp_code = """
#include <math.h>

// Szybka funkcja aktywacji dla Arduino
void apply_tanh(float* arr, int size) {
    for(int i=0; i<size; i++) {
        arr[i] = tanhf(arr[i]);
    }
}

// Główna funkcja "myśląca"
float compute_ai_action(float angle, float omega_p, float omega_w) {
    static float last_action = 0.0f; // Pamięta poprzednią decyzję (wirtualny stan)

    float input[4] = {angle, omega_p, omega_w, last_action};
    float layer1[64] = {0};
    float layer2[64] = {0};
    float output = 0;

    // --- Warstwa Ukryta 1 ---
    for(int i=0; i<64; i++) {
        layer1[i] = b0[i];
        for(int j=0; j<4; j++) {
            layer1[i] += W0[i][j] * input[j];
        }
    }
    apply_tanh(layer1, 64);

    // --- Warstwa Ukryta 2 ---
    for(int i=0; i<64; i++) {
        layer2[i] = b1[i];
        for(int j=0; j<64; j++) {
            layer2[i] += W1[i][j] * layer1[j];
        }
    }
    apply_tanh(layer2, 64);

    // --- Warstwa Wyjściowa ---
    output = b2[0];
    for(int j=0; j<64; j++) {
        output += W2[0][j] * layer2[j];
    }

    // AI myśli w skali -1.0 do 1.0. Obcinamy ewentualne odchylenia.
    if(output > 1.0f) output = 1.0f;
    if(output < -1.0f) output = -1.0f;

    last_action = output; // Zapisz akcję na następny obrót pętli

    // Konwersja na twardy PWM do silnika (-255 do 255)
    return output * 255.0f;
}
"""
    f.write(cpp_code)

print("SUKCES! Gotowy plik AI_Controller.h czeka na Arduino.")