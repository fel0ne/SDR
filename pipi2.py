import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


pcm_filename = "build/tx_recv.pcm"  
sample_rate = 1e6 

if not os.path.exists(pcm_filename):
    print(f"Ошибка: Файл '{pcm_filename}' не найден!")
    exit(1)

raw_data = np.fromfile(pcm_filename, dtype=np.int16)
if len(raw_data) == 0:
    print("Ошибка: Файл пустой!")
    exit(1)

rx_i = raw_data[0::2].astype(np.float32)
rx_q = raw_data[1::2].astype(np.float32)

rx_i -= np.mean(rx_i)
rx_q -= np.mean(rx_q)

rx_complex = rx_i + 1j * rx_q
num_samples = len(rx_complex)
print(f"Загружено сэмплов: {num_samples} ({(num_samples/sample_rate):.2f} секунд записи)")

nperseg = 8192
freqs, psd = signal.welch(rx_complex, fs=sample_rate, nperseg=nperseg, return_onesided=False)

freqs = np.fft.fftshift(freqs) / 1e3
psd = np.fft.fftshift(psd)
spectrum_db = 10 * np.log10(psd + 1e-12)

center_idx = len(freqs) // 2
search_spectrum = spectrum_db.copy()
search_spectrum[center_idx - 2 : center_idx + 3] = -9999
peak_idx = np.argmax(search_spectrum)
max_freq_khz = freqs[peak_idx]
max_power_db = spectrum_db[peak_idx]
noise_floor = np.median(spectrum_db)

print(f"Уровень шума (медиана): {noise_floor:.2f} дБ")
print(f"Максимальный сигнал найден на частоте: {max_freq_khz:+.2f} кГц (Мощность: {max_power_db:.2f} дБ)")

window_size = 1000
power_envelope = rx_i[:num_samples // window_size * window_size].reshape(-1, window_size)**2 + \
                 rx_q[:num_samples // window_size * window_size].reshape(-1, window_size)**2
mean_power = np.mean(power_envelope, axis=1)

threshold = np.median(mean_power) * 3
active_blocks = np.where(mean_power > threshold)[0]

if len(active_blocks) > 0:
    signal_start_sample = active_blocks[0] * window_size
    print(f"Сигнал обнаружен примерно на {signal_start_sample / sample_rate:.3f секунде записи}")
else:
    signal_start_sample = 0
    print("Явного всплеска мощности не найдено, показываем начало файла.")

plot_len = min(50000, num_samples - signal_start_sample)
start_idx = signal_start_sample
end_idx = start_idx + plot_len

t_ms = (np.arange(plot_len) / sample_rate) * 1e3 + (start_idx / sample_rate * 1e3)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

ax1.plot(t_ms, rx_i[start_idx:end_idx], label="RX I (Cos)", color="blue", alpha=0.6)
ax1.plot(t_ms, rx_q[start_idx:end_idx], label="RX Q (Sin)", color="red", alpha=0.6)
ax1.set_title(f"Сигнал во времени (показан участок с отметки {start_idx/sample_rate:.2f} сек)")
ax1.set_xlabel("Время от начала записи (миллисекунды)")
ax1.set_ylabel("Амплитуда")
ax1.grid(True)
ax1.legend(loc="upper right")

ax2.plot(freqs, spectrum_db, color="darkred", label="Спектр (Усредненный по всему файлу)")
ax2.axvline(
    x=max_freq_khz,
    color="green",
    linestyle="--",
    linewidth=2,
    label=f"Максимум амплитуды: {max_freq_khz:.1f} кГц",
)
ax2.set_title(
    f"Анализ спектра: Максимальный сигнал обнаружен на частоте {max_freq_khz:.1f} кГц (Мощность: {max_power_db:.1f} дБ)",
    color="darkgreen",
    fontweight="bold",
)
ax2.set_xlabel("Частота / Отстройка (кГц)")
ax2.set_ylabel("Плотность мощности (дБ)")
ax2.grid(True)
ax2.legend(loc="upper right")

plt.tight_layout()
plt.show()