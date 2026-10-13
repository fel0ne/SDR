#include "third_party/SoapySDR/include/SoapySDR/Device.h"
#include "third_party/SoapySDR/include/SoapySDR/Formats.h"

#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h> 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{

    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    if (1) {
        SoapySDRKwargs_set(&args, "uri", "usb:");
    } else {
        SoapySDRKwargs_set(&args, "uri", "ip:192.168.2.1");
    }
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

    double sample_rate = 1e6;
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate) != 0)
    {
        printf("setSampleRate rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, 800e6, NULL) != 0)
    {
        printf("setFrequency rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, sample_rate) != 0)
    {
        printf("setSampleRate tx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, 800e6, NULL) != 0)
    {
        printf("setFrequency tx fail: %s\n", SoapySDRDevice_lastError());
    }
    printf("SoapySDRDevice_getFrequency tx: %lf\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0));
    
    size_t channels[] = {0}; 
    size_t channel_count = sizeof(channels) / sizeof(channels[0]);

    SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (rxStream == NULL)
    {
        printf("setupStream rx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }
    if (SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, channels, 30.0) != 0) {
        printf("setGain rx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, channels, -40.0) != 0) {
        printf("setGain tx fail: %s\n", SoapySDRDevice_lastError());
    }
    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL)
    {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("MTU - TX: %lu, RX: %lu\n", tx_mtu, rx_mtu);

    int16_t tx_buff[2 * tx_mtu];
    int16_t rx_buffer[2 * rx_mtu];


    double tone_freq = 10e3;
    double amplitude = 20000.0;
    for (size_t i = 0; i < tx_mtu; i++)
    {
        double t = (double)i / sample_rate;
        
        tx_buff[2 * i]     = (int16_t)(amplitude * cos(2.0 * M_PI * tone_freq * t));

        tx_buff[2 * i +1] = (int16_t)(amplitude * sin(2.0 * M_PI * tone_freq * t));
    }


    SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);

    printf("Start test...\n");
    const long timeoutUs = 400000;
    long long last_time = 0;

    FILE *file = fopen("txdata.pcm", "w");
    
    size_t iteration_count = 10;
    for (size_t buffers_read = 0; buffers_read < iteration_count; buffers_read++)
    {
        void *rx_buffs[] = {rx_buffer};
        int flags;
        long long timeNs;

        int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);
        if (sr < 0)
        {
            continue;
        }
  
        printf("Buffer: %lu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n", 
               buffers_read, sr, flags, timeNs, timeNs - last_time);
        fwrite(rx_buffer, 2 * rx_mtu * sizeof(int16_t), 1, file);
        last_time = timeNs;

        long long tx_time = timeNs + (4 * 1000 * 1000);

        void *tx_buffs[] = {tx_buff};
        if (buffers_read == 2) {
            printf("buffers_read: %zu\n", buffers_read);
            flags = SOAPY_SDR_HAS_TIME;
            

            int st = SoapySDRDevice_writeStream(sdr, txStream, (const void * const*)tx_buffs, tx_mtu, &flags, tx_time, timeoutUs);
            if ((size_t)st != tx_mtu)
            {
                printf("TX Failed: %i\n", st);
            }
        }
    }
    fclose(file);

    SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);

    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_closeStream(sdr, txStream);

    SoapySDRDevice_unmake(sdr);

    printf("test complete!\n");

    return EXIT_SUCCESS;
}