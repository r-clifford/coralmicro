#include <cassert>
#include <cstdint>
#include <cstring>
#include <memory>

/*
 * Struct for housing a generic 3 float sensor with timestamp
 */
struct __attribute__((__packed__)) GenSensor {
  uint64_t stamp;
  float x;
  float y;
  float z;
};

typedef struct {
  size_t pos;
  size_t capacity;
  size_t watermark;
  struct GenSensor *data;
} io_buffer_t;
/*
 * Allocate and initialize to empty
 */
void iob_create(io_buffer_t *buf, size_t capacity, size_t watermark) {
  assert(capacity >= watermark);
  buf->data = (struct GenSensor *)malloc(capacity * sizeof(*buf->data));
  buf->pos = 0;
  buf->watermark = watermark;
  buf->capacity = capacity;
}
/*
 * Write *len* sensors from *src* array into *buf*
 * Truncates data if out of space
 */
size_t iob_write(io_buffer_t *buf, struct GenSensor *src, size_t len) {
  size_t remaining = buf->capacity - buf->pos;
  if (remaining < len) {  // truncate
    len = remaining;
  }

  memcpy((uint8_t *)&(buf->data[buf->pos]), (uint8_t *)src, len * sizeof(*src));
  buf->pos += len;
  return len;
}
/*
 * Read all data from *buf* into *dst* (sensor array)
 * Ensure *dst* has capacity for buf->pos * sizeof(struct GenSensor)
 * !currently unused!
 */
size_t iob_read(io_buffer_t *buf, struct GenSensor *dst) {
  size_t available = buf->pos;

  memcpy(dst, buf->data, available * sizeof(*(buf->data)));

  return available;
}
bool iob_past_watermark(io_buffer_t *buf) { return buf->watermark < buf->pos; }
void iob_reset(io_buffer_t *buf) { buf->pos = 0; }
