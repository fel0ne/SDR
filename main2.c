// Передача синуса между двумя Adalm Pluto по эфиру, в обе стороны.
// База - пример преподавателя, изменения помечены "// ИЗМЕНЕНО" / "// ДОБАВЛЕНО".
//
// Аргумент задает не "кто передает", а частотный план платы:
//   ./main tx usb:1.7.5  ->  TX 800 МГц, RX 840 МГц, шлет синус 10 кГц
//   ./main rx usb:1.9.5  ->  TX 840 МГц, RX 800 МГц, шлет синус 20 кГц
// Обе платы и передают, и принимают: плата "tx" слышит плату "rx" на 840,
// плата "rx" слышит плату "tx" на 800. Свой передатчик на своей частоте приема не звучит,
// поэтому все, что принято, - от ДРУГОЙ платы. Разные частоты тона (10 / 20 кГц)
// позволяют это проверить: "tx" должна принимать ~20 кГц, "rx" - ~10 кГц.
//
// Адрес платы - при запуске (lsusb: Bus 001 Device 007 -> usb:1.7.5, или iio_info -s).
// Можно в двух терминалах на одном компьютере или по одному на разных компьютерах.
// Файлы: <режим>_sent.pcm - отправленное, <режим>_recv.pcm - принятое.
// Пишется ВСЁ с момента старта до Ctrl+C (~7.7 МБ/с на каждый файл), при новом запуске файлы перезаписываются.
// Остановка - Ctrl+C.
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h> //printf
#include <stdlib.h> //free
#include <stdint.h>
#include <complex.h>
#include <string.h> // ДОБАВЛЕНО: strcmp
#include <math.h>   // ДОБАВЛЕНО: sin, cos
#include <signal.h> // ДОБАВЛЕНО: Ctrl+C

// ДОБАВЛЕНО: параметры
#define SAMPLE_RATE  1e6
#define AMPLITUDE    2000 // максимум 2047 (12 бит)
#define PRINT_EVERY  250  // печать статуса раз в 250 буферов (~0.5 с)

// ДОБАВЛЕНО: частотный план по схеме
#define TX_MODE_TX_FREQ 800e6
#define TX_MODE_RX_FREQ 840e6
#define TX_MODE_TONE    10e3  // плата "tx" шлет 10 кГц
#define RX_MODE_TX_FREQ 840e6
#define RX_MODE_RX_FREQ 800e6
#define RX_MODE_TONE    20e3  // плата "rx" шлет 20 кГц

// ДОБАВЛЕНО: остановка по Ctrl+C
static volatile sig_atomic_t running = 1;
static void on_sigint(int sig)
{
    (void)sig;
    running = 0;
    signal(SIGINT, SIG_DFL); // второй Ctrl+C - выход сразу
}

int main(int argc, char *argv[]) // ИЗМЕНЕНО: принимаем аргументы
{
    // ДОБАВЛЕНО: частотный план (tx/rx) и адрес платы - из командной строки
    if (argc < 3 || (strcmp(argv[1], "tx") != 0 && strcmp(argv[1], "rx") != 0))
    {
        printf("Запуск:\n  %s tx <uri>   - TX 800 / RX 840 МГц\n  %s rx <uri>   - TX 840 / RX 800 МГц\n", argv[0], argv[0]);
        printf("Пример: %s tx usb:1.7.5\n", argv[0]);
        printf("Адрес: usb:<Bus>.<Device>.5 из lsusb, или из iio_info -s\n");
        return EXIT_FAILURE;
    }
    int is_tx = (strcmp(argv[1], "tx") == 0);
    const char *mode = argv[1];
    const char *uri = argv[2];
    double tx_freq   = is_tx ? TX_MODE_TX_FREQ : RX_MODE_TX_FREQ;
    double rx_freq   = is_tx ? TX_MODE_RX_FREQ : RX_MODE_RX_FREQ;
    double my_tone   = is_tx ? TX_MODE_TONE : RX_MODE_TONE;   // что шлем мы
    double peer_tone = is_tx ? RX_MODE_TONE : TX_MODE_TONE;   // что ждем от другой платы
    printf("Режим: %s, устройство: %s\n", mode, uri);
    printf("TX %.0f МГц (шлем %.0f кГц), RX %.0f МГц (ждем %.0f кГц от другой платы)\n",
           tx_freq / 1e6, my_tone / 1e3, rx_freq / 1e6, peer_tone / 1e3);
    signal(SIGINT, on_sigint);

    // Создание сущности устройства
    // args can be user defined or from the enumeration result
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", uri); // ИЗМЕНЕНО: адрес из командной строки
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL)
    {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    // Настраиваем устройства RX\TX
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, 1e6) != 0)
    {
        printf("setSampleRate rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, rx_freq, NULL) != 0) // ИЗМЕНЕНО: частота по плану
    {
        printf("setFrequency rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, 1e6) != 0)
    {
        printf("setSampleRate tx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, tx_freq, NULL) != 0) // ИЗМЕНЕНО: частота по плану
    {
        printf("setFrequency tx fail: %s\n", SoapySDRDevice_lastError());
    }
    printf("SoapySDRDevice_getFrequency tx: %lf\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0));
    printf("SoapySDRDevice_getFrequency rx: %lf\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_RX, 0)); // ДОБАВЛЕНО

    // Настройка каналов и стримов
    size_t channels[] = {0}; // ИЗМЕНЕНО: было {0, 1}, у Pluto один канал (см. TODO в оригинале)

    size_t channel_count = sizeof(channels) / sizeof(channels[0]);
    SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (rxStream == NULL)
    {
        printf("setupStream rx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }
    // ИЗМЕНЕНО: setGain принимает номер канала, а не массив
    if(SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, channels[0], 30.0) !=0 ){
        printf("setGain rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if(SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, channels[0], -40.0) !=0 ){
        printf("setGain tx fail: %s\n", SoapySDRDevice_lastError());
    }
    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL)
    {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    // Получение MTU (Maximum Transmission Unit), в нашем случае - размер буферов.
    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("MTU - TX: %lu, RX: %lu\n", tx_mtu, rx_mtu);


    int16_t tx_buff[2*tx_mtu];
    int16_t rx_buffer[2*rx_mtu];
    int16_t sent_buff[2*tx_mtu]; // ДОБАВЛЕНО: отправленные сэмплы для записи в файл (12 бит, без заголовка)

    //заполнение tx_buff значениями сэмплов первые 16 бит - I, вторые 16 бит - Q.
    // ИЗМЕНЕНО: вместо константы 2000 - синус, заполняется в основном цикле

    //prepare fixed bytes in transmit buffer
    //we transmit a pattern of FFFF FFFF [TS_0]00 [TS_1]00 [TS_2]00 [TS_3]00 [TS_4]00 [TS_5]00 [TS_6]00 [TS_7]00 FFFF FFFF
    //that is a flag (FFFF FFFF) followed by the 64 bit timestamp, split into 8 bytes and packed into the lsb of each of the DAC words.
    //DAC samples are left aligned 12-bits, so each byte is left shifted into place
    for(size_t i = 0; i < 2; i++)
    {
        tx_buff[0 + i] = 0xffff;
        // 8 x timestamp words
        tx_buff[10 + i] = 0xffff;
    }

    //activate streams
    SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0); //start streaming
    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0); //start streaming

    //here goes
    printf("Start test...\n");
    const long  timeoutUs = 400000; // arbitrarily chosen (взяли из srsRAN)

    // При первом запуске отчищаем rx_buff от возможного мусора.
    for (size_t buffers_read = 0; buffers_read < 128 && running; /* in loop */)
    {
        void *buffs[] = {rx_buffer}; //array of buffers
        int flags; //flags set by receive operation
        long long timeNs; //timestamp for receive buffer
        // Read samples
        int sr = SoapySDRDevice_readStream(sdr, rxStream, buffs, rx_mtu, &flags, &timeNs, timeoutUs);
        if (sr < 0)
        {
            // Skip read on error (likely timeout)
            continue;
        }
        // Increment number of buffers read
        buffers_read++;
    }

    long long last_time = 0;

    // ИЗМЕНЕНО: два файла, имена с режимом, чтобы два процесса в одной папке не затирали друг друга
    char sent_name[64], recv_name[64];
    snprintf(sent_name, sizeof(sent_name), "%s_sent.pcm", mode);
    snprintf(recv_name, sizeof(recv_name), "%s_recv.pcm", mode);
    FILE *sent_file = fopen(sent_name, "wb");
    FILE *recv_file = fopen(recv_name, "wb");

    size_t n = 0;           // ДОБАВЛЕНО: сквозной номер сэмпла синуса
    int peak = 0;           // ДОБАВЛЕНО: пик амплитуды приема
    double complex rot = 0; // ДОБАВЛЕНО: накопитель для оценки частоты принятого тона

    // ИЗМЕНЕНО: бесконечный цикл до Ctrl+C (было 10 итераций)
    for (size_t buffers_read = 0; running; buffers_read++)
    {
        void *rx_buffs[] = {rx_buffer};
        int flags;        // flags set by receive operation
        long long timeNs; //timestamp for receive buffer

        int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);
        if (sr < 0)
        {
            // Skip read on error (likely timeout)
            continue;
        }

        // ===== ДОБАВЛЕНО: прием - то, что пришло от ДРУГОЙ платы =====
        if (recv_file != NULL)
            fwrite(rx_buffer, 2 * sizeof(int16_t), sr, recv_file);

        for (int i = 0; i < 2 * sr; i++)
            if (abs(rx_buffer[i]) > peak)
                peak = abs(rx_buffer[i]);

        // Оценка частоты принятого тона по повороту фазы между соседними сэмплами
        for (int i = 1; i < sr; i++)
        {
            double complex cur  = rx_buffer[2 * i]       + I * rx_buffer[2 * i + 1];
            double complex prev = rx_buffer[2 * (i - 1)] + I * rx_buffer[2 * (i - 1) + 1];
            rot += cur * conj(prev);
        }

        if (buffers_read % PRINT_EVERY == 0)
        {
            double f_khz = carg(rot) * SAMPLE_RATE / (2.0 * M_PI) / 1e3;
            printf("Buffer: %lu - TimeDiff: %lli | прием: пик %4d из 2047, тон %+.2f кГц (ждем ~%.0f)%s\n",
                   buffers_read, timeNs - last_time, peak, f_khz, peer_tone / 1e3,
                   peak < 100 ? "  - только шум, другая плата не передает?" : "");
            peak = 0;
            rot = 0;
        }
        last_time = timeNs;

        // ===== передача =====
        // Переменная для времени отправки сэмплов относительно текущего приема
        long long tx_time = timeNs + (4 * 1000 * 1000); // на 4 [мс] в будущее

        // Добавляем время, когда нужно передать блок tx_buff, через N -наносекунд
        for(size_t i = 0; i < 8; i++)
        {
            // Extract byte from tx time
            uint8_t tx_time_byte = (tx_time >> (i * 8)) & 0xff;

            // Add byte to buffer
            tx_buff[2 + i] = tx_time_byte << 4;
        }

        // ДОБАВЛЕНО: заполняем буфер синусом (первые 12 слов - заголовок, их не трогаем)
        for (size_t i = 12; i < 2 * tx_mtu; i += 2, n++)
        {
            double phase = 2.0 * M_PI * my_tone * n / SAMPLE_RATE;
            tx_buff[i]     = (int16_t)(AMPLITUDE * cos(phase)) << 4; // I
            tx_buff[i + 1] = (int16_t)(AMPLITUDE * sin(phase)) << 4; // Q
        }

        // ДОБАВЛЕНО: сохраняем отправленное (без заголовка, 12 бит) - каждый буфер, одной записью
        if (sent_file != NULL)
        {
            size_t cnt = 0;
            for (size_t i = 12; i < 2 * tx_mtu; i++)
                sent_buff[cnt++] = tx_buff[i] >> 4;
            fwrite(sent_buff, sizeof(int16_t), cnt, sent_file);
        }

        // Send buffer
        // ИЗМЕНЕНО: отправляем каждый буфер, а не только при buffers_read == 2
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

    //stop streaming
    SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);

    //shutdown the stream
    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_closeStream(sdr, txStream);

    //cleanup device handle
    SoapySDRDevice_unmake(sdr);

    //all done
    printf("test complete!\n");

    return EXIT_SUCCESS;
}