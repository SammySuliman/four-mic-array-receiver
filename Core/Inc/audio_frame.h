#ifndef AUDIO_FRAME_H
#define AUDIO_FRAME_H

#include <stdint.h>

#define AUDIO_FRAME_MAGIC_0       0xAA
#define AUDIO_FRAME_MAGIC_1       0x55

#define AUDIO_FRAME_SAMPLES       256
#define AUDIO_CHANNELS            2
#define AUDIO_BYTES_PER_SAMPLE    2

#define AUDIO_PAYLOAD_BYTES \
    (AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)

#define AUDIO_FRAME_HEADER_BYTES  6
#define AUDIO_FRAME_CRC_BYTES     2

#define AUDIO_FRAME_TOTAL_BYTES \
    (AUDIO_FRAME_HEADER_BYTES + AUDIO_PAYLOAD_BYTES + AUDIO_FRAME_CRC_BYTES)

typedef struct __attribute__((packed))
{
    uint8_t magic0;
    uint8_t magic1;
    uint16_t frame_id;
    uint16_t payload_size;
    int16_t samples[AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS];
    uint16_t checksum;
} AudioFrame_t;

static inline uint16_t audio_checksum_u8(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return (uint16_t)(sum & 0xFFFF);
}

#endif