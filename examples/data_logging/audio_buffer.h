#include <vector>
#include "Arduino.h"
#include "PDM.h"
#include "SD.h"

#define AB_BUFFER_SIZE 10  // number of vectors within the buffer
#define AD_START_SIZE 1024 // initial size of each vector within buffer
/*
    attempt to make a lock free buffer for reading/writing audio
    The head and tail indexes should not overlap, ensuring no conflicts assuming
    dequeue is called within the main loop, while enqueue is called within the
    audio interrupt
    If head and tail overlap, buffer is empty
*/

/*
    Stores 16bit audio
    len: amount of valid 16bit data
        avoid using data.size() as data is not cleared for efficiency
*/
typedef struct
{
    size_t len; // number of valid 16bit audio samples in data
    std::vector<int16_t> data;
} audio_data16_t;
/*
    See above
*/
typedef struct
{
    size_t len; // number of valid 32bit audio samples in data
    std::vector<int32_t> data;
} audio_data32_t;
/*
 * Create container for 32bit audio samples
 * Data vector is created with initial capacity of AD_START_SIZE
*/
audio_data32_t *audio_data32_create()
{
    audio_data32_t *ad = (audio_data32_t *)malloc(sizeof(*ad));
    ad->len = 0;
    ad->data = std::vector<int32_t>(AD_START_SIZE);
    return ad;
}
/*
    Converts a segment of 32bit audio to 16bit audio
*/
void audio_data_32_to_16(audio_data32_t *src, audio_data16_t *dst)
{
    for (size_t i = 0; i < src->len; i++)
    {
        dst->data[i] = src->data[i] >> 16;
    }
    dst->len = src->len;
}

typedef struct
{
    size_t head;
    size_t tail;
    audio_data32_t data[AB_BUFFER_SIZE];
} audio_buffer_t;
/*
 * Initialize audio_buffer_t
 * Set initial position of head and tail to 0 (empty)
 * Allocate internal data buffer vectors
*/
audio_buffer_t *audio_buffer_init(audio_buffer_t *buf)
{
    buf->head = 0;
    buf->tail = 0;
    for (size_t i = 0; i < AB_BUFFER_SIZE; i++)
    {
        audio_data32_t *data = audio_data32_create();
        assert(data);
        buf->data[i] = *data;
    }
    return buf;
}
/*
 * Increment a value *n* wrapping to 0 if *mod* is reached
*/
size_t wrapping_increment(size_t n, size_t mod) {
    return (n + 1) % mod;
}
/*
    Get a pointer to the current head (newest data, enqueue target-1)
*/
audio_data32_t *audio_buffer_head(audio_buffer_t *buf)
{
    return &(buf->data[buf->head]);
}
/*
    Get a pointer to the current tail (dequeue target)
*/
audio_data32_t *audio_buffer_tail(audio_buffer_t *buf)
{
    return &(buf->data[buf->tail]);
}
/*
 Remove oldest data from queue, write to file
 ensure file is open on function call, file remains open to maximize bandwidth
 return number of bytes written, 0 if buffer is empty
*/
size_t audio_buffer_dequeue(audio_buffer_t *buf, SDFile* file)
{
    if (buf->head == buf->tail) {
        return 0;
    }
    audio_data32_t *data32 = audio_buffer_tail(buf);
    audio_data16_t data16;
    data16.data.reserve(data32->len);
    audio_data_32_to_16(data32, &data16);
    buf->tail = wrapping_increment(buf->tail, AB_BUFFER_SIZE);
    return file->write(
        (uint8_t *)data16.data.data(),
        data16.len * sizeof(data16.data[0]));
}
/*
 Read available data into queue
 Data is stored at head + 1
 Will overwrite old data if queue is full
 returns number of bytes read
*/
size_t audio_buffer_enqueue(audio_buffer_t *buf)
{
    buf->head = wrapping_increment(buf->head, AB_BUFFER_SIZE);
    if (buf->head == buf->tail)
    {
        // buffer full, start overwriting oldest data
        buf->tail = wrapping_increment(buf->tail, AB_BUFFER_SIZE);
    }
    audio_data32_t *target = audio_buffer_head(buf);
    target->len = Mic.available();
    return Mic.read(target->data, target->len); // read all available data
}

bool audio_buffer_full(audio_buffer_t* buf) {
    size_t next_index = wrapping_increment(buf->head, AB_BUFFER_SIZE);
    return next_index == buf->tail;
}
bool audio_buffer_empty(audio_buffer_t* buf) {
    return buf->head == buf->tail;
}
