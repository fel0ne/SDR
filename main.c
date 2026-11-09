// Передача BPSK / QPSK сообщения между двумя Adalm Pluto по эфиру, в обе стороны.
// База - пример преподавателя, доработанный под ДЗ (4 шага передатчика с комплексными числами).

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <string.h>
#include <math.h>
#include <signal.h>

// Параметры SDR
#define SAMPLE_RATE  1e6
#define AMPLITUDE    2000 // максимум 2047 (12 бит ЦАП)
#define PRINT_EVERY  250  // печать статуса раз в 250 буферов (~0.5 с)

// Параметры модуляции
#define USE_QPSK     0    // 0 = BPSK, 1 = QPSK (переключатель для сдачи ДЗ)
#define SPS          10   // отсчетов на символ (samples per symbol)
#define GAP_SYMBOLS  32   // пауза из нулей между повторами сообщения (в символах)

// Частотный план и сообщения по умолчанию
#define TX_MODE_TX_FREQ 800e6
#define TX_MODE_RX_FREQ 840e6
#define TX_MODE_MSG     "Hello from TX"
#define RX_MODE_TX_FREQ 840e6
#define RX_MODE_RX_FREQ 800e6
#define RX_MODE_MSG     "Hello from RX"

// Остановка по Ctrl+C
static volatile sig_atomic_t running = 1;
static void on_sigint(int sig)
{
    (void)sig;
    running = 0;
    signal(SIGINT, SIG_DFL);
}

// ================= ДОМАШНЕЕ ЗАДАНИЕ: 4 ШАГА ПЕРЕДАТЧИКА =================

// --- ШАГ 1. Генератор битов ---

// Вариант А: генерация случайных битов 
static void generate_random_bits(int *bits, size_t n_bits)
{
    for (size_t i = 0; i < n_bits; i++)
        bits[i] = rand() % 2; // 0 или 1
}

// Вариант Б: перевод текста в биты (ASCII, старший бит первым)
static size_t message_to_bits(const char *msg, int *bits)
{
    size_t len = strlen(msg);
    for (size_t c = 0; c < len; c++)
        for (int b = 7; b >= 0; b--)
            bits[c * 8 + (7 - b)] = ((unsigned char)msg[c] >> b) & 1;
    return len * 8;
}

// --- ШАГ 2. Модуляция (BPSK и QPSK) с комплексными значениями ---

// BPSK: 1 бит -> 1 комплексный символ (меняется только Re/I, а Im/Q = 0)
// 0 -> -1 + 0i, 1 -> +1 + 0i
static size_t modulate_bpsk(const int *bits, size_t n_bits, double complex *symbols)
{
    for (size_t i = 0; i < n_bits; i++)
    {
        double re = bits[i] ? +1.0 : -1.0;
        symbols[i] = re + 0.0 * I;
    }
    return n_bits; // количество символов равно количеству бит
}

// QPSK: 2 бита -> 1 комплексный символ (первый бит в Re/I, второй бит в Im/Q)
// 00 -> -1 - 1i, 01 -> -1 + 1i, 10 -> +1 - 1i, 11 -> +1 + 1i
static size_t modulate_qpsk(const int *bits, size_t n_bits, double complex *symbols)
{
    size_t n_symbols = n_bits / 2;
    for (size_t i = 0; i < n_symbols; i++)
    {
        double re = bits[2 * i]     ? +1.0 : -1.0;
        double im = bits[2 * i + 1] ? +1.0 : -1.0;
        symbols[i] = re + im * I;
    }
    return n_symbols; // символов в 2 раза меньше, чем бит
}

// --- ШАГ 3. Upsampling (добавление нулей между символами) ---

// После каждого комплексного символа вставляем sps - 1 комплексных нулей
static void upsample(const double complex *symbols, size_t n_symbols, int sps, double complex *out)
{
    for (size_t i = 0; i < n_symbols * sps; i++)
        out[i] = 0.0 + 0.0 * I;

    for (size_t i = 0; i < n_symbols; i++)
        out[i * sps] = symbols[i];
}

// --- ШАГ 4. Свертка (получение прямоугольных импульсов) ---

// Свертка комплексного сигнала x с импульсной характеристикой фильтра h:
// y[n] = sum_k x[k] * h[n - k]
static void convolve(const double complex *x, size_t len_x, const double *h, size_t len_h, double complex *y)
{
    size_t len_y = len_x + len_h - 1;
    for (size_t n = 0; n < len_y; n++)
    {
        double complex acc = 0.0 + 0.0 * I;
        for (size_t k = 0; k < len_x; k++)
        {
            if (n >= k && n - k < len_h)
                acc += x[k] * h[n - k];
        }
        y[n] = acc;
    }
}

// Полная сборка сигнала: возвращает массив int16_t, где лежат пары [I0, Q0, I1, Q1, ...]
// В *out_samples записывается количество комплексных отсчетов (пар I/Q)
static int16_t *build_tx_signal(const char *msg, size_t *out_samples)
{
    size_t max_bits = strlen(msg) * 8;
    int            *bits    = malloc(max_bits * sizeof(int));
    double complex *symbols = malloc(max_bits * sizeof(double complex));

    // 1. Генерируем биты из текста
    size_t n_bits = message_to_bits(msg, bits);
    // Если нужно проверить просто случайные биты, можно раскомментировать:
    // generate_random_bits(bits, n_bits);

    // 2. Модуляция (выбираем BPSK или QPSK)
    size_t n_symbols = 0;
    if (USE_QPSK)
        n_symbols = modulate_qpsk(bits, n_bits, symbols);
    else
        n_symbols = modulate_bpsk(bits, n_bits, symbols);

    // Выделяем память под upsampling и свертку
    size_t up_len = n_symbols * SPS;
    size_t y_len  = up_len + SPS - 1;
    double complex *up = malloc(up_len * sizeof(double complex));
    double complex *y  = malloc(y_len * sizeof(double complex));

    // 3. Upsampling (вставляем нули)
    upsample(symbols, n_symbols, SPS, up);

    // 4. Свертка с прямоугольным импульсом из SPS единиц
    double h[SPS];
    for (int i = 0; i < SPS; i++)
        h[i] = 1.0;

    convolve(up, up_len, h, SPS, y);

    // Вывод отладочной информации в консоль
    printf("Модуляция: %s\n", USE_QPSK ? "QPSK" : "BPSK");
    printf("Сообщение: \"%s\" (%zu байт, %zu бит, %zu символов)\nБиты: ",
           msg, strlen(msg), n_bits, n_symbols);
    for (size_t i = 0; i < n_bits; i++)
        printf("%d%s", bits[i], (i % 8 == 7) ? " " : "");
    printf("\n");

    // Перевод комплексных отсчетов в формат ЦАП (чередование I, Q, I, Q...) + пауза в конце
    size_t gap = GAP_SYMBOLS * SPS;
    size_t total_samples = y_len + gap;
    int16_t *signal_out = calloc(2 * total_samples, sizeof(int16_t));

    for (size_t i = 0; i < y_len; i++)
    {
        signal_out[2 * i]     = (int16_t)(AMPLITUDE * creal(y[i])); // Канал I (Re)
        signal_out[2 * i + 1] = (int16_t)(AMPLITUDE * cimag(y[i])); // Канал Q (Im)
    }

    printf("Длина сигнала: %zu комплексных отсчетов (%.2f мс), из них пауза %zu\n",
           total_samples, total_samples / SAMPLE_RATE * 1e3, gap);

    free(bits);
    free(symbols);
    free(up);
    free(y);

    *out_samples = total_samples;
    return signal_out;
}

// =======================================================================

int main(int argc, char *argv[])
{
    if (argc < 3 || (strcmp(argv[1], "tx") != 0 && strcmp(argv[1], "rx") != 0))
    {
        printf("Запуск:\n  %s tx <uri> [текст]   - TX 800 / RX 840 МГц\n  %s rx <uri> [текст]   - TX 840 / RX 800 МГц\n", argv[0], argv[0]);
        printf("Пример: %s tx usb:1.7.5 \"Hello\"\n", argv[0]);
        return EXIT_FAILURE;
    }

    int is_tx = (strcmp(argv[1], "tx") == 0);
    const char *mode = argv[1];
    const char *uri = argv[2];
    const char *msg = (argc >= 4) ? argv[3] : (is_tx ? TX_MODE_MSG : RX_MODE_MSG);
    if (strlen(msg) == 0)
    {
        printf("Пустое сообщение\n");
        return EXIT_FAILURE;
    }

    double tx_freq = is_tx ? TX_MODE_TX_FREQ : RX_MODE_TX_FREQ;
    double rx_freq = is_tx ? TX_MODE_RX_FREQ : RX_MODE_RX_FREQ;
    printf("Режим: %s, устройство: %s\n", mode, uri);
    printf("TX %.0f МГц, RX %.0f МГц, %s, %d отсчетов на символ\n",
           tx_freq / 1e6, rx_freq / 1e6, USE_QPSK ? "QPSK" : "BPSK", SPS);
    signal(SIGINT, on_sigint);

    // Готовим комплексный сигнал (пары I и Q) заранее
    size_t sig_len = 0; // длина в комплексных отсчетах
    int16_t *tx_signal = build_tx_signal(msg, &sig_len);
    size_t sig_pos = 0;
    size_t msg_len = sig_len - GAP_SYMBOLS * SPS;

    // Создание сущности устройства
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", uri);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL)
    {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        free(tx_signal);
        return EXIT_FAILURE;
    }

    // Настраиваем устройства RX/TX
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, SAMPLE_RATE) != 0)
        printf("setSampleRate rx fail: %s\n", SoapySDRDevice_lastError());

    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, rx_freq, NULL) != 0)
        printf("setFrequency rx fail: %s\n", SoapySDRDevice_lastError());

    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, SAMPLE_RATE) != 0)
        printf("setSampleRate tx fail: %s\n", SoapySDRDevice_lastError());

    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, tx_freq, NULL) != 0)
        printf("setFrequency tx fail: %s\n", SoapySDRDevice_lastError());

    printf("SoapySDRDevice_getFrequency tx: %lf\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0));
    printf("SoapySDRDevice_getFrequency rx: %lf\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_RX, 0));

    // Настройка каналов и стримов
    size_t channels[] = {0};
    size_t channel_count = sizeof(channels) / sizeof(channels[0]);
    SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (rxStream == NULL)
    {
        printf("setupStream rx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        free(tx_signal);
        return EXIT_FAILURE;
    }

    if (SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, channels[0], 30.0) != 0)
        printf("setGain rx fail: %s\n", SoapySDRDevice_lastError());

    if (SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, channels[0], -40.0) != 0)
        printf("setGain tx fail: %s\n", SoapySDRDevice_lastError());

    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL)
    {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        free(tx_signal);
        return EXIT_FAILURE;
    }

    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("MTU - TX: %lu, RX: %lu\n", tx_mtu, rx_mtu);

    size_t payload = tx_mtu - 6;
    int one_per_buffer = (msg_len + GAP_SYMBOLS * SPS <= payload);
    if (one_per_buffer)
        printf("Сообщение (%zu отсчетов) передается в начале каждого буфера, остаток буфера - пауза\n", msg_len);
    else
        printf("ВНИМАНИЕ: сообщение длиннее буфера (%zu > %zu отсчетов) - идет по кругу через стыки буферов\n",
               msg_len + GAP_SYMBOLS * SPS, payload);

    int16_t tx_buff[2 * tx_mtu];
    int16_t rx_buffer[2 * rx_mtu];
    int16_t sent_buff[2 * tx_mtu];

    // Заполняем фиксированные флаги заголовка для метки времени
    for (size_t i = 0; i < 2; i++)
    {
        tx_buff[0 + i] = 0xffff;
        tx_buff[10 + i] = 0xffff;
    }

    // Активация потоков
    SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);

    printf("Start test...\n");
    const long timeoutUs = 400000;

    // Очистка приемного буфера при старте
    for (size_t buffers_read = 0; buffers_read < 128 && running;)
    {
        void *buffs[] = {rx_buffer};
        int flags;
        long long timeNs;
        int sr = SoapySDRDevice_readStream(sdr, rxStream, buffs, rx_mtu, &flags, &timeNs, timeoutUs);
        if (sr < 0)
            continue;
        buffers_read++;
    }

    long long last_time = 0;

    char sent_name[64], recv_name[64];
    snprintf(sent_name, sizeof(sent_name), "%s_sent.pcm", mode);
    snprintf(recv_name, sizeof(recv_name), "%s_recv.pcm", mode);
    FILE *sent_file = fopen(sent_name, "wb");
    FILE *recv_file = fopen(recv_name, "wb");

    int peak = 0;
    double complex rot = 0;

    // Основной цикл приема и передачи
    for (size_t buffers_read = 0; running; buffers_read++)
    {
        void *rx_buffs[] = {rx_buffer};
        int flags;
        long long timeNs;

        int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);
        if (sr < 0)
            continue;

        // Запись принятого сигнала в файл
        if (recv_file != NULL)
            fwrite(rx_buffer, 2 * sizeof(int16_t), sr, recv_file);

        for (int i = 0; i < 2 * sr; i++)
            if (abs(rx_buffer[i]) > peak)
                peak = abs(rx_buffer[i]);

        for (int i = 1; i < sr; i++)
        {
            double complex cur  = rx_buffer[2 * i]       + I * rx_buffer[2 * i + 1];
            double complex prev = rx_buffer[2 * (i - 1)] + I * rx_buffer[2 * (i - 1) + 1];
            rot += cur * conj(prev);
        }

        if (buffers_read % PRINT_EVERY == 0)
        {
            double f_khz = carg(rot) * SAMPLE_RATE / (2.0 * M_PI) / 1e3;
            printf("Buffer: %lu | TimeDiff: %lli | прием: пик %4d из 2047, сдвиг частоты ~%+.2f кГц%s\n",
                   buffers_read, timeNs - last_time, peak, f_khz,
                   peak < 20 ? "  - только шум, другая плата не передает?" : "");
            peak = 0;
            rot = 0;
        }
        last_time = timeNs;

        // ===== ПЕРЕДАЧА =====
        long long tx_time = timeNs + (4 * 1000 * 1000); // +4 мс в будущее

        for (size_t i = 0; i < 8; i++)
        {
            uint8_t tx_time_byte = (tx_time >> (i * 8)) & 0xff;
            tx_buff[2 + i] = tx_time_byte << 4;
        }

        // Заполняем буфер парами I и Q (первые 12 элементов - служебный заголовок)
        for (size_t i = 12, j = 0; i < 2 * tx_mtu; i += 2, j++)
        {
            int16_t v_i = 0;
            int16_t v_q = 0;

            if (one_per_buffer)
            {
                if (j < msg_len)
                {
                    v_i = tx_signal[2 * j];
                    v_q = tx_signal[2 * j + 1];
                }
            }
            else
            {
                v_i = tx_signal[2 * sig_pos];
                v_q = tx_signal[2 * sig_pos + 1];
                sig_pos = (sig_pos + 1) % sig_len;
            }

            tx_buff[i]     = (int16_t)(v_i * 16); // I, сдвиг на 4 бита (под 12-битный ЦАП)
            tx_buff[i + 1] = (int16_t)(v_q * 16); // Q, сдвиг на 4 бита (в BPSK тут будет 0, в QPSK +-2000)
        }

        // Сохраняем отправленные сэмплы (без заголовка) в файл для проверки
        if (sent_file != NULL)
        {
            size_t cnt = 0;
            for (size_t i = 12; i < 2 * tx_mtu; i++)
                sent_buff[cnt++] = tx_buff[i] / 16;
            fwrite(sent_buff, sizeof(int16_t), cnt, sent_file);
        }

        void *tx_buffs[] = {tx_buff};
        flags = SOAPY_SDR_HAS_TIME;
        int st = SoapySDRDevice_writeStream(sdr, txStream, (const void * const*)tx_buffs, tx_mtu, &flags, tx_time, timeoutUs);
        if ((size_t)st != tx_mtu)
        {
            printf("TX Failed: %i\n", st);
        }
    }

    if (sent_file != NULL) fclose(sent_file);
    if (recv_file != NULL) fclose(recv_file);

    printf("\nОстановка... Записаны %s и %s\n", sent_name, recv_name);

    SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_closeStream(sdr, txStream);
    SoapySDRDevice_unmake(sdr);

    free(tx_signal);
    printf("test complete!\n");

    return EXIT_SUCCESS;
}