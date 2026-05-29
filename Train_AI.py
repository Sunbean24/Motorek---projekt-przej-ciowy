import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import math
import collections


# ==========================================
# 1. CYFROWY BLIŹNIAK Z RANDOMIZACJĄ (DOMAIN RANDOMIZATION)
# ==========================================
class RobustBikeEnv(gym.Env):
    def __init__(self):
        super(RobustBikeEnv, self).__init__()

        # Baza: Macierz SINDy od Janka
        self.base_C_matrix = np.array([
            [1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            [3.1650, 0.0000, 1.9677, -340.7180, -18.6057, 2.2722, 4.2342, 4.2484],
            [-2.8034, 0.0000, -0.3921, 1469.3823, 308.7707, -65.5415, -102.0640, -102.3568]
        ])

        self.dt = 0.01  # 10 ms (jak w Arduino)
        self.max_angle = 10.0 * (math.pi / 180.0)  # Podpórki przy 10 stopniach

        # Akcja: Sygnał sterujący PWM zmapowany na -1.0 do 1.0 (Dla stabilności sieci)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # Obserwacja: [Kąt, Prędkość ramy, Prędkość koła, Poprzednia Akcja (dla płynności)]
        high_obs = np.array([self.max_angle * 2, 10.0, 100.0, 1.0], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_obs, high=high_obs, dtype=np.float32)

        # Symulacja sprzętowego opóźnienia silnika (2 próbki = 20ms opóźnienia PWM)
        self.action_delay_buffer = collections.deque(maxlen=2)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 1. Losowy start
        start_angle = np.random.uniform(-0.05, 0.05)  # Losowy start do ~3 stopni
        self.state = np.array([start_angle, 0.0, 0.0], dtype=np.float32)

        # 2. Randomizacja Dziedziny! (Psujemy macierz o losowe +/- 5% co każdy epizod)
        noise_matrix = np.random.uniform(0.95, 1.05, self.base_C_matrix.shape)
        self.current_C_matrix = self.base_C_matrix * noise_matrix

        # Czyszczenie bufora opóźnień
        self.action_delay_buffer.extend([0.0, 0.0])
        self.last_action = 0.0

        obs = np.array([start_angle, 0.0, 0.0, 0.0], dtype=np.float32)
        return obs, {}

    def step(self, action):
        # AI podaje akcję od -1 do 1. Skalujemy na Napięcie silnika (-12 do 12V)
        control_u = float(action[0]) * 12.0

        # Symulacja opóźnienia sprzętowego
        self.action_delay_buffer.append(control_u)
        delayed_u = self.action_delay_buffer[0]  # Silnik reaguje z opóźnieniem!

        th, om_p, om_w = self.state

        # SINDy features
        features = np.array([
            om_p,
            om_w,
            delayed_u,
            math.sin(th),
            math.tanh(0.5 * delayed_u),
            math.tanh(delayed_u),
            math.tanh(2.0 * delayed_u),
            math.tanh(5.0 * delayed_u)
        ])

        # Wyliczenie fizyki
        x_dot = np.dot(self.current_C_matrix, features)

        # Dorzucenie losowego wiatru / szumu pomiarowego (Noise)
        x_dot[0] += np.random.normal(0, 0.01)  # Szum dla kąta
        x_dot[1] += np.random.normal(0, 0.05)  # Szum dla żyroskopu

        # Aktualizacja stanu (Całkowanie Eulera)
        th_new = th + x_dot[0] * self.dt
        om_p_new = om_p + x_dot[1] * self.dt
        om_w_new = om_w + x_dot[2] * self.dt

        self.state = np.array([th_new, om_p_new, om_w_new], dtype=np.float32)

        # Obserwacja dla sieci (kąt, gyro, koło, ostatnia akcja)
        obs = np.array([th_new, om_p_new, om_w_new, float(action[0])], dtype=np.float32)

        # --- FUNKCJA NAGRODY (MÓZG AI) ---
        # 1. Nagroda za trzymanie pionu (max 1.0 w idealnym pionie)
        reward = 1.0 - (abs(th_new) / self.max_angle)

        # 2. Kara za gwałtowne ruchy silnika (wymuszamy płynność i oszczędność prądu!)
        action_diff = abs(float(action[0]) - self.last_action)
        reward -= 0.1 * action_diff

        self.last_action = float(action[0])

        # --- WARUNEK UPADKU ---
        terminated = bool(abs(th_new) > self.max_angle)
        if terminated:
            reward = -100.0  # Bolesna kara za przewrócenie

        return obs, float(reward), terminated, False, {}


# ==========================================
# 2. PROCES TRENINGU AI
# ==========================================
if __name__ == "__main__":
    print("Inicjalizacja Wirtualnego Środowiska...")
    env = RobustBikeEnv()

    # Konfiguracja sieci neuronowej: MlpPolicy (Multi-Layer Perceptron)
    # Ustawiamy 2 warstwy ukryte po 64 neurony. Wystarczy do sterowania i nie zadławi Arduino.
    policy_kwargs = dict(net_arch=dict(pi=[64, 64], vf=[64, 64]))

    model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1)

    # Zapisuj model co 100 000 kroków (jakbyś musiał przerwać)
    checkpoint_callback = CheckpointCallback(save_freq=100000, save_path='./logs/', name_prefix='rl_model')

    print("Rozpoczynam morderczy trening PPO (1 000 000 kroków) - to potrwa kilkanaście minut...")
    model.learn(total_timesteps=1000000, callback=checkpoint_callback)

    print("Trening zakończony! Zapisywanie finalnego 'mózgu'...")
    model.save("bike_ppo_robust")
    print("Gotowe. Zapisano plik 'bike_ppo_robust.zip'.")