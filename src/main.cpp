// Передача BPSK / QPSK сообщения между двумя Adalm Pluto по эфиру, в обе стороны.
// Многопоточная версия с графическим интерфейсом (ImGui + ImPlot) и управлением в реальном времени.

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#include <complex>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <chrono>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

using Complex = std::complex<double>;

// Параметры по умолчанию
#define SAMPLE_RATE     1e6

#define TX_MODE_TX_FREQ 800.0f // МГц
#define TX_MODE_RX_FREQ 840.0f // МГц
#define TX_MODE_MSG     "Hello from TX"

#define RX_MODE_TX_FREQ 840.0f // МГц
#define RX_MODE_RX_FREQ 800.0f // МГц
#define RX_MODE_MSG     "Hello from RX"

struct SharedState {
    std::atomic<bool> app_running{true};
    std::atomic<bool> want_connected{false}; // команда от GUI: подключиться / отключиться
    std::atomic<bool> sdr_connected{false};  // фактический статус подключения

    std::atomic<bool> tx_enabled{true};
    std::atomic<bool> record_pcm{false};

    // Параметры подключения и сообщение
    std::mutex cfg_mutex;
    std::string device_uri = "ip:192.168.2.1";
    std::string mode_tag   = "tx";
    std::string tx_message = TX_MODE_MSG;

    // Параметры модуляции и TX DSP
    std::atomic<int>  use_qpsk{0};       // 0 = BPSK, 1 = QPSK
    std::atomic<int>  amplitude{2000};   // 0..2047
    std::atomic<int>  sps{10};           // 2..32
    std::atomic<int>  gap_symbols{32};   // 0..128
    std::atomic<bool> rebuild_tx{true};

    // Параметры RF (PlutoSDR)
    std::atomic<float> rx_gain{30.0f};      // 0..70 дБ
    std::atomic<float> tx_gain{-40.0f};     // -89..0 дБ
    std::atomic<float> rx_freq_mhz{840.0f}; // МГц
    std::atomic<float> tx_freq_mhz{800.0f}; // МГц

    // Параметры отображения созвездия
    std::atomic<int>   constellation_pts{1000};
    std::atomic<int>   strobe_phase{5};      // 0 .. sps-1
    std::atomic<bool>  decimate_by_sps{true};
    std::atomic<float> phase_rot_deg{0.0f};  // ручной доворот фазы (-180..180)
    std::atomic<float> plot_scale{2200.0f};  // предел осей I/Q

    // Телеметрия от SDR -> GUI
    std::atomic<int>    rx_peak{0};
    std::atomic<double> cfo_khz{0.0};

    std::mutex status_mutex;
    std::string status_text = "Отключено. Введите URI PlutoSDR и нажмите «Подключиться»";

    // Буфер точек для созвездия и осциллограммы
    std::mutex iq_mutex;
    std::vector<double> iq_i;
    std::vector<double> iq_q;
    std::vector<double> wave_i;
    std::vector<double> wave_q;
};

static SharedState g_state;

static void on_sigint(int sig)
{
    (void)sig;
    g_state.app_running = false;
    g_state.want_connected = false;
    signal(SIGINT, SIG_DFL);
}

static void clear_iq_buffers()
{
    std::lock_guard<std::mutex> lock(g_state.iq_mutex);
    g_state.iq_i.clear();
    g_state.iq_q.clear();
    g_state.wave_i.clear();
    g_state.wave_q.clear();
    g_state.rx_peak = 0;
    g_state.cfo_khz = 0.0;
}

// ================= 4 ШАГА ПЕРЕДАТЧИКА =================

static size_t message_to_bits(const char *msg, std::vector<int> &bits)
{
    size_t len = strlen(msg);
    bits.resize(len * 8);
    for (size_t c = 0; c < len; c++)
        for (int b = 7; b >= 0; b--)
            bits[c * 8 + (7 - b)] = ((unsigned char)msg[c] >> b) & 1;
    return len * 8;
}

static size_t modulate_bpsk(const std::vector<int> &bits, std::vector<Complex> &symbols)
{
    size_t n_bits = bits.size();
    symbols.resize(n_bits);
    for (size_t i = 0; i < n_bits; i++)
    {
        double re = bits[i] ? +1.0 : -1.0;
        symbols[i] = Complex(re, 0.0);
    }
    return n_bits;
}

static size_t modulate_qpsk(const std::vector<int> &bits, std::vector<Complex> &symbols)
{
    size_t n_symbols = bits.size() / 2;
    symbols.resize(n_symbols);
    for (size_t i = 0; i < n_symbols; i++)
    {
        double re = bits[2 * i]     ? +1.0 : -1.0;
        double im = bits[2 * i + 1] ? +1.0 : -1.0;
        symbols[i] = Complex(re, im);
    }
    return n_symbols;
}

static void upsample(const std::vector<Complex> &symbols, int sps, std::vector<Complex> &out)
{
    out.assign(symbols.size() * sps, Complex(0.0, 0.0));
    for (size_t i = 0; i < symbols.size(); i++)
        out[i * sps] = symbols[i];
}

static void convolve(const std::vector<Complex> &x, const std::vector<double> &h, std::vector<Complex> &y)
{
    size_t len_x = x.size();
    size_t len_h = h.size();
    size_t len_y = len_x + len_h - 1;
    y.assign(len_y, Complex(0.0, 0.0));

    for (size_t n = 0; n < len_y; n++)
    {
        Complex acc(0.0, 0.0);
        for (size_t k = 0; k < len_x; k++)
        {
            if (n >= k && n - k < len_h)
                acc += x[k] * h[n - k];
        }
        y[n] = acc;
    }
}

static std::vector<int16_t> build_tx_signal(const std::string &msg, int use_qpsk, int sps, int gap_symbols, int amplitude, size_t &msg_len_out)
{
    if (msg.empty())
    {
        msg_len_out = 0;
        return std::vector<int16_t>(2 * sps, 0);
    }

    std::vector<int> bits;
    message_to_bits(msg.c_str(), bits);

    std::vector<Complex> symbols;
    if (use_qpsk)
        modulate_qpsk(bits, symbols);
    else
        modulate_bpsk(bits, symbols);

    if (symbols.empty())
    {
        msg_len_out = 0;
        return std::vector<int16_t>(2 * sps, 0);
    }

    std::vector<Complex> up;
    upsample(symbols, sps, up);

    std::vector<double> h(sps, 1.0);
    std::vector<Complex> y;
    convolve(up, h, y);

    msg_len_out = y.size();
    size_t gap = (size_t)gap_symbols * sps;
    size_t total_samples = y.size() + gap;

    std::vector<int16_t> signal_out(2 * total_samples, 0);
    for (size_t i = 0; i < y.size(); i++)
    {
        signal_out[2 * i]     = (int16_t)(amplitude * y[i].real());
        signal_out[2 * i + 1] = (int16_t)(amplitude * y[i].imag());
    }
    return signal_out;
}

static void push_rx_samples_to_gui(const int16_t *rx_buffer, size_t num_samples, int sps, int strobe_phase, bool decimate)
{
    int max_pts = g_state.constellation_pts.load();
    float rot_rad = g_state.phase_rot_deg.load() * (float)M_PI / 180.0f;
    double cos_r = cos(rot_rad);
    double sin_r = sin(rot_rad);

    int step = (decimate && sps > 1) ? sps : 1;
    int offset = (decimate && sps > 1) ? (strobe_phase % sps) : 0;

    std::lock_guard<std::mutex> lock(g_state.iq_mutex);

    for (size_t i = offset; i < num_samples; i += step)
    {
        double raw_i = (double)rx_buffer[2 * i];
        double raw_q = (double)rx_buffer[2 * i + 1];

        double rot_i = raw_i * cos_r - raw_q * sin_r;
        double rot_q = raw_i * sin_r + raw_q * cos_r;

        g_state.iq_i.push_back(rot_i);
        g_state.iq_q.push_back(rot_q);
    }

    if ((int)g_state.iq_i.size() > max_pts)
    {
        size_t excess = g_state.iq_i.size() - max_pts;
        g_state.iq_i.erase(g_state.iq_i.begin(), g_state.iq_i.begin() + excess);
        g_state.iq_q.erase(g_state.iq_q.begin(), g_state.iq_q.begin() + excess);
    }

    size_t wave_len = std::min<size_t>(num_samples, 250);
    g_state.wave_i.resize(wave_len);
    g_state.wave_q.resize(wave_len);
    for (size_t i = 0; i < wave_len; i++)
    {
        g_state.wave_i[i] = (double)rx_buffer[2 * i];
        g_state.wave_q[i] = (double)rx_buffer[2 * i + 1];
    }
}

// ================= ФОНОВЫЙ ПОТОК SDR =================

static void sdr_worker()
{
    while (g_state.app_running)
    {
        // Ждём нажатия кнопки «Подключиться»
        if (!g_state.want_connected)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::string uri, mode;
        {
            std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
            uri = g_state.device_uri;
            mode = g_state.mode_tag;
        }

        {
            std::lock_guard<std::mutex> lock(g_state.status_mutex);
            g_state.status_text = "Подключение к " + uri + "...";
        }

        SoapySDRKwargs args = {};
        SoapySDRKwargs_set(&args, "driver", "plutosdr");
        SoapySDRKwargs_set(&args, "uri", uri.c_str());
        SoapySDRKwargs_set(&args, "direct", "1");
        SoapySDRKwargs_set(&args, "timestamp_every", "1920");
        SoapySDRKwargs_set(&args, "loopback", "0");

        SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
        SoapySDRKwargs_clear(&args);

        if (sdr == NULL)
        {
            g_state.sdr_connected = false;
            g_state.want_connected = false;
            clear_iq_buffers();
            {
                std::lock_guard<std::mutex> lock(g_state.status_mutex);
                g_state.status_text = std::string("Ошибка подключения (") + uri + "): " + SoapySDRDevice_lastError();
            }
            continue;
        }

        float applied_rx_freq = g_state.rx_freq_mhz.load();
        float applied_tx_freq = g_state.tx_freq_mhz.load();
        float applied_rx_gain = g_state.rx_gain.load();
        float applied_tx_gain = g_state.tx_gain.load();

        SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, SAMPLE_RATE);
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, applied_rx_freq * 1e6, NULL);
        SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, SAMPLE_RATE);
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, applied_tx_freq * 1e6, NULL);

        size_t channels[] = {0};
        SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, 1, NULL);
        SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, 1, NULL);

        if (!rxStream || !txStream)
        {
            if (rxStream) SoapySDRDevice_closeStream(sdr, rxStream);
            if (txStream) SoapySDRDevice_closeStream(sdr, txStream);
            SoapySDRDevice_unmake(sdr);
            g_state.sdr_connected = false;
            g_state.want_connected = false;
            clear_iq_buffers();
            {
                std::lock_guard<std::mutex> lock(g_state.status_mutex);
                g_state.status_text = std::string("Ошибка создания потоков RX/TX: ") + SoapySDRDevice_lastError();
            }
            continue;
        }

        SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, applied_rx_gain);
        SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, applied_tx_gain);

        size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
        size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);

        std::vector<int16_t> tx_buff(2 * tx_mtu, 0);
        std::vector<int16_t> rx_buffer(2 * rx_mtu, 0);
        std::vector<int16_t> sent_buff(2 * tx_mtu, 0);

        for (size_t i = 0; i < 2; i++)
        {
            tx_buff[0 + i]  = (int16_t)0xffff;
            tx_buff[10 + i] = (int16_t)0xffff;
        }

        SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);
        SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);

        g_state.sdr_connected = true;
        g_state.rebuild_tx = true;
        clear_iq_buffers();
        {
            std::lock_guard<std::mutex> lock(g_state.status_mutex);
            g_state.status_text = "Подключено: " + uri;
        }

        const long timeoutUs = 400000;
        for (size_t i = 0; i < 64 && g_state.app_running && g_state.want_connected;)
        {
            void *buffs[] = {rx_buffer.data()};
            int flags;
            long long timeNs;
            if (SoapySDRDevice_readStream(sdr, rxStream, buffs, rx_mtu, &flags, &timeNs, timeoutUs) >= 0)
                i++;
        }

        FILE *sent_file = nullptr;
        FILE *recv_file = nullptr;
        bool was_recording = false;

        std::vector<int16_t> tx_signal;
        size_t msg_len = 0;
        size_t sig_pos = 0;
        int cur_sps = g_state.sps.load();
        int cur_gap = g_state.gap_symbols.load();

        int peak = 0;
        Complex rot(0.0, 0.0);

        for (size_t buffers_read = 0; g_state.app_running && g_state.want_connected; buffers_read++)
        {
            // 1. Обновляем RF параметры при изменении слайдеров
            float req_rx_gain = g_state.rx_gain.load();
            if (fabsf(req_rx_gain - applied_rx_gain) > 0.05f)
            {
                SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, req_rx_gain);
                applied_rx_gain = req_rx_gain;
            }

            float req_tx_gain = g_state.tx_gain.load();
            if (fabsf(req_tx_gain - applied_tx_gain) > 0.05f)
            {
                SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, req_tx_gain);
                applied_tx_gain = req_tx_gain;
            }

            float req_rx_freq = g_state.rx_freq_mhz.load();
            if (fabsf(req_rx_freq - applied_rx_freq) > 0.001f)
            {
                SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, req_rx_freq * 1e6, NULL);
                applied_rx_freq = req_rx_freq;
            }

            float req_tx_freq = g_state.tx_freq_mhz.load();
            if (fabsf(req_tx_freq - applied_tx_freq) > 0.001f)
            {
                SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, req_tx_freq * 1e6, NULL);
                applied_tx_freq = req_tx_freq;
            }

            // 2. Пересобираем передаваемый сигнал при изменении сообщения или параметров модуляции
            if (g_state.rebuild_tx.exchange(false))
            {
                std::string msg_copy;
                {
                    std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
                    msg_copy = g_state.tx_message;
                }
                cur_sps = g_state.sps.load();
                cur_gap = g_state.gap_symbols.load();
                tx_signal = build_tx_signal(msg_copy, g_state.use_qpsk.load(), cur_sps, cur_gap, g_state.amplitude.load(), msg_len);
                sig_pos = 0;
            }

            // 3. Запись PCM
            bool rec = g_state.record_pcm.load();
            if (rec && !was_recording)
            {
                sent_file = fopen((mode + "_sent.pcm").c_str(), "wb");
                recv_file = fopen((mode + "_recv.pcm").c_str(), "wb");
                was_recording = true;
            }
            else if (!rec && was_recording)
            {
                if (sent_file) { fclose(sent_file); sent_file = nullptr; }
                if (recv_file) { fclose(recv_file); recv_file = nullptr; }
                was_recording = false;
            }

            // 4. ПРИЁМ
            void *rx_buffs[] = {rx_buffer.data()};
            int flags;
            long long timeNs;
            int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);
            if (sr < 0)
                continue;

            if (recv_file)
                fwrite(rx_buffer.data(), 2 * sizeof(int16_t), sr, recv_file);

            for (int i = 0; i < 2 * sr; i++)
                if (abs(rx_buffer[i]) > peak)
                    peak = abs(rx_buffer[i]);

            for (int i = 1; i < sr; i++)
            {
                Complex cur(rx_buffer[2 * i], rx_buffer[2 * i + 1]);
                Complex prev(rx_buffer[2 * (i - 1)], rx_buffer[2 * (i - 1) + 1]);
                rot += cur * conj(prev);
            }

            if (buffers_read % 2 == 0)
            {
                push_rx_samples_to_gui(rx_buffer.data(), sr, cur_sps, g_state.strobe_phase.load(), g_state.decimate_by_sps.load());
            }

            if (buffers_read % 50 == 0)
            {
                g_state.rx_peak = peak;
                g_state.cfo_khz = std::arg(rot) * SAMPLE_RATE / (2.0 * M_PI) / 1e3;
                peak = 0;
                rot = Complex(0.0, 0.0);
            }

            // 5. ПЕРЕДАЧА
            long long tx_time = timeNs + (4 * 1000 * 1000);
            for (size_t i = 0; i < 8; i++)
            {
                uint8_t tx_time_byte = (tx_time >> (i * 8)) & 0xff;
                tx_buff[2 + i] = tx_time_byte << 4;
            }

            size_t payload = tx_mtu - 6;
            size_t sig_len = tx_signal.size() / 2;
            bool one_per_buffer = (msg_len + (size_t)cur_gap * cur_sps <= payload);
            bool tx_on = g_state.tx_enabled.load();

            for (size_t i = 12, j = 0; i < 2 * tx_mtu; i += 2, j++)
            {
                int16_t v_i = 0;
                int16_t v_q = 0;

                if (tx_on && sig_len > 0)
                {
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
                }

                tx_buff[i]     = (int16_t)(v_i * 16);
                tx_buff[i + 1] = (int16_t)(v_q * 16);
            }

            if (sent_file)
            {
                size_t cnt = 0;
                for (size_t i = 12; i < 2 * tx_mtu; i++)
                    sent_buff[cnt++] = tx_buff[i] / 16;
                fwrite(sent_buff.data(), sizeof(int16_t), cnt, sent_file);
            }

            void *tx_buffs[] = {tx_buff.data()};
            flags = SOAPY_SDR_HAS_TIME;
            SoapySDRDevice_writeStream(sdr, txStream, (const void *const *)tx_buffs, tx_mtu, &flags, tx_time, timeoutUs);
        }

        if (sent_file) fclose(sent_file);
        if (recv_file) fclose(recv_file);

        SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
        SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
        SoapySDRDevice_closeStream(sdr, rxStream);
        SoapySDRDevice_closeStream(sdr, txStream);
        SoapySDRDevice_unmake(sdr);

        g_state.sdr_connected = false;
        clear_iq_buffers();
        {
            std::lock_guard<std::mutex> lock(g_state.status_mutex);
            g_state.status_text = "Отключено от " + uri;
        }
    }
}

// ================= СТИЛЬ ИНТЕРФЕЙСА =================

static void ApplyModernDarkTheme()
{
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding    = 8.0f;
    style.ChildRounding     = 8.0f;
    style.FrameRounding     = 6.0f;
    style.GrabRounding      = 6.0f;
    style.PopupRounding     = 6.0f;
    style.ScrollbarRounding = 6.0f;
    style.WindowPadding     = ImVec2(14, 14);
    style.FramePadding      = ImVec2(10, 6);
    style.ItemSpacing       = ImVec2(10, 8);
    style.GrabMinSize       = 14.0f;

    ImVec4 *colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = ImVec4(0.08f, 0.09f, 0.11f, 1.00f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.11f, 0.13f, 0.16f, 1.00f);
    colors[ImGuiCol_Border]               = ImVec4(0.20f, 0.24f, 0.30f, 0.60f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.15f, 0.18f, 0.23f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.20f, 0.25f, 0.32f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.24f, 0.30f, 0.39f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.08f, 0.09f, 0.11f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.10f, 0.12f, 0.15f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.00f, 0.72f, 0.85f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.15f, 0.85f, 0.98f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.16f, 0.22f, 0.29f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.00f, 0.60f, 0.72f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.00f, 0.75f, 0.88f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.00f, 0.82f, 0.95f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.16f, 0.22f, 0.29f, 1.00f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.00f, 0.55f, 0.68f, 0.80f);
    colors[ImGuiCol_Separator]            = ImVec4(0.20f, 0.24f, 0.30f, 1.00f);
}

// ================= MAIN & GUI =================

int main(int argc, char *argv[])
{
    std::string init_mode = "tx";
    std::string init_uri  = "ip:192.168.2.1";
    std::string init_msg  = TX_MODE_MSG;
    bool auto_connect     = false;

    if (argc >= 2)
    {
        init_mode = argv[1];
        bool is_tx = (init_mode == "tx");
        g_state.tx_freq_mhz = is_tx ? TX_MODE_TX_FREQ : RX_MODE_TX_FREQ;
        g_state.rx_freq_mhz = is_tx ? TX_MODE_RX_FREQ : RX_MODE_RX_FREQ;
        init_msg = is_tx ? TX_MODE_MSG : RX_MODE_MSG;
    }
    if (argc >= 3)
    {
        init_uri = argv[2];
        auto_connect = true;
    }
    if (argc >= 4)
    {
        init_msg = argv[3];
    }

    {
        std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
        g_state.mode_tag   = init_mode;
        g_state.device_uri = init_uri;
        g_state.tx_message = init_msg;
    }
    g_state.want_connected = auto_connect;

    signal(SIGINT, on_sigint);

    if (!glfwInit())
        return -1;

    GLFWwindow *window = glfwCreateWindow(1320, 820, "PlutoSDR BPSK / QPSK Трансивер", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    const ImWchar *cyrillic_ranges = io.Fonts->GetGlyphRangesCyrillic();
    ImFont *font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16.0f, nullptr, cyrillic_ranges);
    if (!font)
        io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", 16.0f, nullptr, cyrillic_ranges);

    ApplyModernDarkTheme();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    std::thread sdr_thread(sdr_worker);

    char uri_input[128];
    strncpy(uri_input, init_uri.c_str(), sizeof(uri_input) - 1);
    uri_input[sizeof(uri_input) - 1] = '\0';

    char msg_input[256];
    strncpy(msg_input, init_msg.c_str(), sizeof(msg_input) - 1);
    msg_input[sizeof(msg_input) - 1] = '\0';

    std::vector<double> local_i, local_q;
    std::vector<double> local_wi, local_wq;

    while (!glfwWindowShouldClose(window) && g_state.app_running)
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int win_w, win_h;
        glfwGetWindowSize(window, &win_w, &win_h);

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2((float)win_w, (float)win_h));
        ImGui::Begin("MainDock", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        // --- ВЕРХНЯЯ ПАНЕЛЬ СТАТУСА ---
        {
            std::string st;
            {
                std::lock_guard<std::mutex> lock(g_state.status_mutex);
                st = g_state.status_text;
            }
            bool conn = g_state.sdr_connected.load();
            ImVec4 led_col = conn ? ImVec4(0.15f, 0.90f, 0.45f, 1.0f) : ImVec4(0.90f, 0.25f, 0.25f, 1.0f);

            ImGui::TextColored(led_col, "●");
            ImGui::SameLine();
            ImGui::Text("%s", st.c_str());

            ImGui::SameLine(win_w - 430.0f);
            int peak = g_state.rx_peak.load();
            double cfo = g_state.cfo_khz.load();
            ImGui::Text("Пик RX: %4d / 2047  |  CFO: %+.2f кГц", peak, cfo);
        }

        ImGui::Separator();

        // --- ЛЕВАЯ ПАНЕЛЬ УПРАВЛЕНИЯ ---
        float left_width = 390.0f;
        ImGui::BeginChild("ControlsPanel", ImVec2(left_width, 0), true);

        // 1. ПОДКЛЮЧЕНИЕ И СООБЩЕНИЕ (самое главное сверху)
        ImGui::TextColored(ImVec4(0.0f, 0.82f, 0.95f, 1.0f), "ПОДКЛЮЧЕНИЕ PLUTOSDR И СООБЩЕНИЕ");
        ImGui::Spacing();

        bool is_connected = g_state.sdr_connected.load();
        bool want_conn    = g_state.want_connected.load();

        ImGui::Text("Адрес PlutoSDR (URI):");
        ImGui::SetNextItemWidth(-1);
        if (is_connected) ImGui::BeginDisabled();
        if (ImGui::InputText("##uri_input", uri_input, sizeof(uri_input)))
        {
            std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
            g_state.device_uri = uri_input;
        }
        if (is_connected) ImGui::EndDisabled();

        // Быстрые пресеты ролей (TX / RX)
        if (ImGui::Button("Пресет TX (800/840 МГц)", ImVec2((left_width - 34) * 0.5f, 0)))
        {
            g_state.tx_freq_mhz = TX_MODE_TX_FREQ;
            g_state.rx_freq_mhz = TX_MODE_RX_FREQ;
            strncpy(msg_input, TX_MODE_MSG, sizeof(msg_input) - 1);
            {
                std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
                g_state.mode_tag = "tx";
                g_state.tx_message = msg_input;
            }
            g_state.rebuild_tx = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Пресет RX (840/800 МГц)", ImVec2(-1, 0)))
        {
            g_state.tx_freq_mhz = RX_MODE_TX_FREQ;
            g_state.rx_freq_mhz = RX_MODE_RX_FREQ;
            strncpy(msg_input, RX_MODE_MSG, sizeof(msg_input) - 1);
            {
                std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
                g_state.mode_tag = "rx";
                g_state.tx_message = msg_input;
            }
            g_state.rebuild_tx = true;
        }

        // Кнопка Подключиться / Отключиться
        if (!want_conn)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.55f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.15f, 0.70f, 0.38f, 1.0f));
            if (ImGui::Button("Подключиться к PlutoSDR", ImVec2(-1, 34)))
            {
                std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
                g_state.device_uri = uri_input;
                g_state.want_connected = true;
            }
            ImGui::PopStyleColor(2);
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.18f, 0.18f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.22f, 0.22f, 1.0f));
            if (ImGui::Button("Отключиться", ImVec2(-1, 34)))
            {
                g_state.want_connected = false;
            }
            ImGui::PopStyleColor(2);
        }

        ImGui::Spacing();
        ImGui::Text("Сообщение для передачи:");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##txmsg", msg_input, sizeof(msg_input)))
        {
            std::lock_guard<std::mutex> lock(g_state.cfg_mutex);
            g_state.tx_message = msg_input;
            g_state.rebuild_tx = true;
        }

        bool tx_en = g_state.tx_enabled.load();
        if (ImGui::Checkbox("Передача TX активна", &tx_en))
            g_state.tx_enabled = tx_en;

        ImGui::SameLine();
        bool rec_pcm = g_state.record_pcm.load();
        if (ImGui::Checkbox("Писать .pcm", &rec_pcm))
            g_state.record_pcm = rec_pcm;

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // 2. РАДИОТРАКТ
        ImGui::TextColored(ImVec4(0.0f, 0.82f, 0.95f, 1.0f), "РАДИОТРАКТ (RF)");
        ImGui::Spacing();

        float rx_gain = g_state.rx_gain.load();
        if (ImGui::SliderFloat("Усиление RX (дБ)", &rx_gain, 0.0f, 70.0f, "%.1f дБ"))
            g_state.rx_gain = rx_gain;

        float tx_gain = g_state.tx_gain.load();
        if (ImGui::SliderFloat("Усиление TX (дБ)", &tx_gain, -89.0f, 0.0f, "%.1f дБ"))
            g_state.tx_gain = tx_gain;

        float rx_freq = g_state.rx_freq_mhz.load();
        if (ImGui::SliderFloat("Частота RX (МГц)", &rx_freq, 70.0f, 3000.0f, "%.2f МГц"))
            g_state.rx_freq_mhz = rx_freq;

        float tx_freq = g_state.tx_freq_mhz.load();
        if (ImGui::SliderFloat("Частота TX (МГц)", &tx_freq, 70.0f, 3000.0f, "%.2f МГц"))
            g_state.tx_freq_mhz = tx_freq;

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // 3. МОДУЛЯЦИЯ
        ImGui::TextColored(ImVec4(0.0f, 0.82f, 0.95f, 1.0f), "МОДУЛЯЦИЯ");
        ImGui::Spacing();

        int mod_type = g_state.use_qpsk.load();
        if (ImGui::RadioButton("BPSK", &mod_type, 0))
        {
            g_state.use_qpsk = 0;
            g_state.rebuild_tx = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("QPSK", &mod_type, 1))
        {
            g_state.use_qpsk = 1;
            g_state.rebuild_tx = true;
        }

        int amp = g_state.amplitude.load();
        if (ImGui::SliderInt("Амплитуда ЦАП", &amp, 0, 2047))
        {
            g_state.amplitude = amp;
            g_state.rebuild_tx = true;
        }

        int sps = g_state.sps.load();
        if (ImGui::SliderInt("Отсчетов на символ (SPS)", &sps, 2, 32))
        {
            g_state.sps = sps;
            if (g_state.strobe_phase >= sps)
                g_state.strobe_phase = sps - 1;
            g_state.rebuild_tx = true;
        }

        int gap = g_state.gap_symbols.load();
        if (ImGui::SliderInt("Пауза (символов)", &gap, 0, 128))
        {
            g_state.gap_symbols = gap;
            g_state.rebuild_tx = true;
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // 4. СОЗВЕЗДИЕ
        ImGui::TextColored(ImVec4(0.0f, 0.82f, 0.95f, 1.0f), "НАСТРОЙКИ СОЗВЕЗДИЯ");
        ImGui::Spacing();

        int pts = g_state.constellation_pts.load();
        if (ImGui::SliderInt("Точек на графике", &pts, 100, 4000))
            g_state.constellation_pts = pts;

        bool decimate = g_state.decimate_by_sps.load();
        if (ImGui::Checkbox("Прореживание по SPS (строб)", &decimate))
            g_state.decimate_by_sps = decimate;

        int strobe = g_state.strobe_phase.load();
        int max_strobe = std::max(0, g_state.sps.load() - 1);
        if (ImGui::SliderInt("Фаза строба", &strobe, 0, max_strobe))
            g_state.strobe_phase = strobe;

        float phase_rot = g_state.phase_rot_deg.load();
        if (ImGui::SliderFloat("Поворот фазы (°)", &phase_rot, -180.0f, 180.0f, "%.1f°"))
            g_state.phase_rot_deg = phase_rot;

        float scale = g_state.plot_scale.load();
        if (ImGui::SliderFloat("Масштаб осей I/Q", &scale, 100.0f, 2200.0f, "%.0f"))
            g_state.plot_scale = scale;

        if (ImGui::Button("Очистить график", ImVec2(-1, 0)))
            clear_iq_buffers();

        ImGui::EndChild();

        ImGui::SameLine();

        // --- ПРАВАЯ ПАНЕЛЬ: СОЗВЕЗДИЕ И ОСЦИЛЛОГРАММА ---
        ImGui::BeginChild("PlotsPanel", ImVec2(0, 0), true);

        {
            std::lock_guard<std::mutex> lock(g_state.iq_mutex);
            local_i = g_state.iq_i;
            local_q = g_state.iq_q;
            local_wi = g_state.wave_i;
            local_wq = g_state.wave_q;
        }

        float avail_h = ImGui::GetContentRegionAvail().y;
        float const_h = avail_h * 0.70f;
        float wave_h  = avail_h - const_h - 10.0f;

        double lim = (double)g_state.plot_scale.load();
        if (ImPlot::BeginPlot("Сигнальное созвездие (I / Q)", ImVec2(-1, const_h), ImPlotFlags_Equal))
        {
            ImPlot::SetupAxes("Синфазная составляющая I (Re)", "Квадратурная составляющая Q (Im)");
            ImPlot::SetupAxisLimits(ImAxis_X1, -lim, lim, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -lim, lim, ImGuiCond_Always);

            double zero_line[2] = {-lim * 2.0, lim * 2.0};
            double zeros[2]     = {0.0, 0.0};
            ImPlot::PlotLine("##xaxis", zero_line, zeros, 2);
            ImPlot::PlotLine("##yaxis", zeros, zero_line, 2);

            if (!local_i.empty())
            {
                ImPlot::PlotScatter("Принятые символы", local_i.data(), local_q.data(), (int)local_i.size());
            }

            ImPlot::EndPlot();
        }

        if (ImPlot::BeginPlot("Принятый сигнал во времени (первые 250 отсчётов)", ImVec2(-1, wave_h)))
        {
            ImPlot::SetupAxes("Отсчёт (n)", "Амплитуда");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, 250, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -lim, lim, ImGuiCond_Always);

            if (!local_wi.empty())
            {
                ImPlot::PlotLine("I (Re)", local_wi.data(), (int)local_wi.size());
                ImPlot::PlotLine("Q (Im)", local_wq.data(), (int)local_wq.size());
            }

            ImPlot::EndPlot();
        }

        ImGui::EndChild();
        ImGui::End();

        ImGui::Render();
        glViewport(0, 0, win_w, win_h);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    g_state.app_running = false;
    g_state.want_connected = false;
    if (sdr_thread.joinable())
        sdr_thread.join();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}