# Просмотр BPSK: что отправили и что приняли.
# Запуск из папки build:
#   python3 bpsk_view.py                          # tx_sent.pcm -> rx_recv.pcm (800 МГц)
#   python3 bpsk_view.py rx_sent.pcm tx_recv.pcm  # обратное направление (840 МГц)
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

SENT_FILE = sys.argv[1] if len(sys.argv) > 1 else "recorded_data/QPSK/tx_sent.pcm"
RECV_FILE = sys.argv[2] if len(sys.argv) > 2 else "recorded_data/QPSK/rx_recv.pcm"
SAMPLE_RATE = 1e6
SHOW = 50000   # сколько отсчетов показать во времени (1.5 мс)
BLOCK = 1920  # размер буфера в C-коде
SPS = 10      # отсчетов на символ (как в main.c)
EST_LEN = 50000  # сколько отсчетов брать для оценки сдвига частоты


def read_iq(name):
    if not os.path.exists(name):
        print(f"Ошибка: файл '{name}' не найден")
        sys.exit(1)
    raw = np.fromfile(name, dtype=np.int16)
    raw = raw[: len(raw) // 2 * 2].astype(np.float32)
    return raw[0::2], raw[1::2]

def gardner_ted(x, y, k, Ts, tau_hat):
    prev = int((k - 1) * Ts + tau_hat)
    mid  = int((k - 0.5) * Ts + tau_hat)
    curr = int(k * Ts + tau_hat)

    return x[mid] * (x[prev] - x[curr]) + y[mid] * (y[prev] - y[curr])

tx_i, tx_q = read_iq(SENT_FILE)
rx_i, rx_q = read_iq(RECV_FILE)


rx = rx_i + 1j * rx_q

# Самый сильный участок приема (в начале другая плата могла еще не передавать)
n_blocks = max(len(rx) // BLOCK, 1)
power = [np.mean(np.abs(rx[k * BLOCK:(k + 1) * BLOCK]) ** 2) for k in range(n_blocks)]
start = int(np.argmax(power)) * BLOCK
start = min(start, max(len(rx) - SHOW, 0))
seg = slice(start, start + SHOW)

h = [1.0] * SPS
len_y = len(rx) + len(h) - 1
y = [0.0 + 0.0j] * len_y

for n in range(len_y):
    acc = 0.0 + 0.0j
    for k in range(len(h)):
        if 0 <= n - k < len(rx):
            acc += rx[n - k] * h[k]
    y[n] = acc

y = np.asarray(y)

# --- ГРАФИК 1: Исходный входной I/Q ---
plt.figure(figsize=(6, 6))
plt.scatter(rx_i[seg], rx_q[seg], color="blue", alpha=0.6, label="I/Q")
plt.title("I/Q Созвездие (до фильтра)")
plt.xlabel("I")
plt.ylabel("Q")
plt.axhline(0, color="black", linewidth=0.8, linestyle="--")
plt.axvline(0, color="black", linewidth=0.8, linestyle="--")
plt.grid(True)
plt.legend(loc="upper right")
plt.tight_layout()

# --- ГРАФИК 2: После согласованного фильтра (все отсчеты) ---
plt.figure(figsize=(6, 6))
plt.scatter(y[seg].real, y[seg].imag, color="blue", alpha=0.6, label="Filtered I/Q")
plt.title("I/Q Созвездие (после свертки)")
plt.xlabel("I")
plt.ylabel("Q")
plt.axhline(0, color="black", linewidth=0.8, linestyle="--")
plt.axvline(0, color="black", linewidth=0.8, linestyle="--")
plt.grid(True)
plt.legend(loc="upper right")
plt.tight_layout()

# --- 10 ГРАФИКОВ
y_eval = y[start : start + SHOW]

fig, axes = plt.subplots(2, 5, figsize=(18, 7), sharex=True, sharey=True)
axes = axes.flatten()

for phase in range(SPS):
    z_phase = y_eval[phase::SPS]
    
    ax = axes[phase]
    ax.scatter(z_phase.real, z_phase.imag, color="crimson", alpha=0.6, s=15)
    ax.set_title(f"Фаза: {phase}")
    ax.set_xlabel("I")
    ax.set_ylabel("Q")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax.axvline(0, color="black", linewidth=0.8, linestyle="--")
    ax.grid(True)


plt.suptitle("Созвездия )", fontsize=14)
plt.tight_layout()




e_mean = [0.0] * SPS
x = y_eval.real
y_iq = y_eval.imag

# Сколько символов берем для усреднения (например, 2000)
num_symbols = 2000

e = []

# Перебираем 10 фаз стробирования (tau_hat от 0 до 9)
for phase in range(SPS):
    errors = []
    for k in range(1, num_symbols):
        err = gardner_ted(x, y_iq, k,SPS, phase)
        errors.append(err)
    
    # Средняя ошибка для текущей фазы
    e.append(np.mean(errors))

# --- График S-кривой ---
plt.figure(figsize=(7, 5))
plt.plot(range(SPS), e, marker="o", color="blue")
plt.title("Ошибка Гарднера от фазы стробирования")
plt.xlabel("Фаза (сдвиг отсчета)")
plt.ylabel("Средняя ошибка e")
plt.xticks(range(SPS))
plt.axhline(0, color="black", linestyle="--")
plt.grid(True)
plt.tight_layout()

plt.show()