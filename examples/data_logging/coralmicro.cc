#include <Adafruit_BNO08x.h>

#include <cstdint>
#include <cstdio>

#include "Arduino.h"
#include "PDM.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "audio_buffer.h"
#include "io_buffer.h"
#include "pins_arduino.h"

// SemaphoreHandle_t sd_mutex = xSemaphoreCreateMutex();
const uint32_t BAUD_RATE = 115200;

uint64_t startTime;

char AUDIO_FNAME[13] = "/AUDIO";
char ACCEL_FNAME[13] = "/ACCEL";
char GYRO_FNAME[13] = "/GYRO";
char MAG_FNAME[13] = "/MAG";
SDFile audioFile;
volatile uint32_t stopWrite = 0;
volatile uint32_t iter_while_full = 0;
audio_buffer_t audio_buffer;

const uint32_t sample_rate = 16000;  // 48k causes stuttering, lower imu sample rate
const size_t sample_size_ms = 1000;
const size_t drop_first_samples_ms = 1000;

const size_t filenameMax = 13;  // 8.3 file name, name(8)+dot(1)+ext(3)+\0

size_t total = 0;  // total number of bytes written to file

// imu
// struct __attribute__((__packed__)) GenSensor {
// // struct GenSensor {
//   uint64_t stamp;
//   float x;
//   float y;
//   float z;
// };
#define BNO08X_RESET A4  // 2
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorId_t reportTypes[3] = {SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED,
                                 SH2_MAGNETIC_FIELD_CALIBRATED};
long reportIntervalUs = 10000;  // 100Hz
// long reportIntervalUs = 1000000;  // 1Hz
sh2_Accelerometer_t Accelerometer;
sh2_Gyroscope_t Gyroscope;
sh2_MagneticField_t Magnetometer;
sh2_SensorValue_t sensorValue;
struct GenSensor outputSensor;
// For imu debug
#define printSensor(sensor) \
  do {                      \
    Serial.print(sensor.x); \
    Serial.print("\t");     \
    Serial.print(sensor.y); \
    Serial.print("\t");     \
    Serial.print(sensor.z); \
    Serial.print("\t");     \
  } while (0)
/*
 * Enable IMU report type at a given interval
 */
void setReports(sh2_SensorId_t reportType, long reportInterval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, reportInterval)) {
    Serial.print("Could not enable: ");
    Serial.print(reportType);
    Serial.print("@");
    Serial.println(reportInterval);
    while (1) delay(1000);
  }
}
/*
 * Find the next index for a new file with filename: name
 * If no index available within [0,99], return -1;
 * Store last tested file in result
 */
int firstAvailableFileIndex(const char* name, char result[13]) {
  assert(strlen(name) <= 12);  // ensure the name + index fits within result
  for (size_t i = 0; i < 100; i++) {
    sprintf(result, "%s%02d", name, i);
    if (!SD.exists(result)) {
      return i;
    }
  }
  return -1;
}
/*
 * name: filename for file to create
 * deleteExisting: if true, delete if exists, else leave unchanged
 * increment: if true, create new file with incremented file name
 * returns true on success, false on failure
 * ensure SD.begin()
 */
bool createFile(char* name, bool deleteExisting, bool increment) {
  assert(!(deleteExisting &&
           increment));  // delete existing and increment are mutually ex
  char newName[13];
  if (deleteExisting) {
    SD.remove(name);
  } else if (increment) {
    firstAvailableFileIndex(name, newName);
  } else {
    // append
  }
  strncpy(name, newName, 13);  // verify
  SDFile f = SD.open(name, FILE_WRITE);
  if (!f) {
    while (1) {
    Serial.print("Failed to create: ");
    Serial.println(newName);    
    delay(1000);  
    }

    return false;
  } else {
    Serial.print("Created: ");
    Serial.println(newName);
  }
  f.close();
  return true;
}
const size_t buffer_capacity = 200;
const size_t watermark = 125;
io_buffer_t accel_buf;
io_buffer_t gyro_buf;
io_buffer_t mag_buf;
/*
 * Append given sensor to buffer
 */
void bufferSensor(io_buffer_t* buffer, struct GenSensor* sensor) {
  iob_write(buffer, sensor, 1);
}
/*
 * Write all data in buffer to filename
 * Format: timestamp,x,y,z
 */
size_t flushBuffer(io_buffer_t* buffer, const char* filename) {
  SDFile f = SD.open(filename, FILE_WRITE);
  size_t result = 0;
  for (size_t i = 0; i < buffer->pos; i++) {
    struct GenSensor* sensor = &buffer->data[i];

    // arduino sprintf doesn't support float?
    result += f.print(sensor->stamp);
    result += f.print(",");
    result += f.print(sensor->x, 10);
    result += f.print(",");
    result += f.print(sensor->y, 10);
    result += f.print(",");
    result += f.println(sensor->z, 10);
  }
  f.close();
  buffer->pos = 0;
  return result;
}
/*
 * Parse sensorValue
 * Add timestamp to the sensor output struct
 * Append to buffer
 * if buffer size > watermark, flush *all* buffers to file - this potentially
 * reduces number of interruptions to audio stream, testing req
 */
void handleSensorEvent(sh2_SensorValue_t* sensorIn,
                       struct GenSensor* sensorOut) {
  auto a = sensorIn->un.accelerometer;
  auto g = sensorIn->un.gyroscope;
  auto m = sensorIn->un.magneticField;
  const char* filename;
  io_buffer_t* buffer;
  switch (sensorIn->sensorId) {
    case SH2_ACCELEROMETER:
      sensorOut->x = a.x;
      sensorOut->y = a.y;
      sensorOut->z = a.z;
      filename = ACCEL_FNAME;
      buffer = &accel_buf;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      sensorOut->x = g.x;
      sensorOut->y = g.y;
      sensorOut->z = g.z;
      filename = GYRO_FNAME;
      buffer = &gyro_buf;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      sensorOut->x = m.x;
      sensorOut->y = m.y;
      sensorOut->z = m.z;
      filename = MAG_FNAME;
      buffer = &mag_buf;
      break;
    default:
      Serial.print("Invalid sensor id: ");
      Serial.println(sensorIn->sensorId);
      while (1) delay(1000);
  }
  sensorOut->stamp = millis();

  bufferSensor(buffer, sensorOut);

  if (iob_past_watermark(buffer)) {
    // xSemaphoreTake(sd_mutex, portMAX_DELAY); // no audio write
    noInterrupts();
    audioFile.close();  // ensure file close not interrupted by audio callback
    interrupts();
    size_t flushed = flushBuffer(buffer, filename);

    Serial.print("Flushed: ");
    Serial.println(flushed);
    noInterrupts();
    audioFile = SD.open(
        AUDIO_FNAME, FILE_WRITE);  // ensure file is opened without interruption
    interrupts();
    // xSemaphoreGive(sd_mutex);
    total += flushed;
  }
}
/*
 * Callback to handle audio buffering when available
 * See audio_buffer_enqueue
 */
void micCallback() { 
  if (stopWrite == 0) {
    audio_buffer_enqueue(&audio_buffer); 
  }
}
/*
 * Callback to increment the number of attempts to stop file writing
 */
void callback() { stopWrite += 1; }
void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  // setup imu buffers
  iob_create(&accel_buf, buffer_capacity, watermark);
  iob_create(&gyro_buf, buffer_capacity, watermark);
  iob_create(&mag_buf, buffer_capacity, watermark);
  audio_buffer_init(&audio_buffer);

  while (!Serial) delay(100);  // wait for serial to become available
  delay(3000);
  Serial.println("Serial started");

  if (!SD.begin()) {
    Serial.println("SD init failed");
  }

  bool fileFail = true;
  // create data files, do not delete, create new files with incremented index
  if (!createFile(AUDIO_FNAME, false, true)) {
    Serial.print("Failed to create file: "); Serial.println(AUDIO_FNAME);
  } else if (!createFile(ACCEL_FNAME, false, true)) {
    Serial.print("Failed to create file: "); Serial.println(ACCEL_FNAME);
  } else if (!createFile(GYRO_FNAME, false, true)) {
    Serial.print("Failed to create file: "); Serial.println(GYRO_FNAME);
  } else if (!createFile(MAG_FNAME, false, true)) {
    Serial.print("Failed to create file: "); Serial.println(MAG_FNAME);
  } else {
    fileFail = false;
    Serial.println("Files created");
  }
  if (fileFail) {

    while (1) delay(1000);
  }

  // setup button interrupt to stop sd card operations before shutdown
  pinMode(PIN_BTN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN), callback, LOW);

  Serial.println("Start");
  delay(1000);
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed IMU startup");
    while (1) delay(1000);
  }
  // enable sensor report types
  for (size_t i = 0; i < sizeof(reportTypes); i++) {
    setReports(reportTypes[i], reportIntervalUs);
  }
  audioFile = SD.open(AUDIO_FNAME, FILE_WRITE);
  if (!audioFile) {
    Serial.print("Failed to open: ");
    Serial.println(AUDIO_FNAME);
    while (1) delay(1000);
  } else {
    Serial.println("Audio file opened");
  }
  Mic.onReceive(micCallback);
  Mic.begin(sample_rate, sample_size_ms, drop_first_samples_ms);
  Serial.println("Microphone started");
  startTime = millis();
}

void loop() {
  // if audio is available in buffer, write buffer to file
  // does not open file, file should be open at startup and after any writes to
  // other files (imu) maximizing audio file open time maximizes overall
  // throughput as calls to open/close massively reduce the speed data may be
  // written audio file is closed on button interrupt to ensure data is flushed
  // before intentional power loss
  if (!audio_buffer_empty(&audio_buffer)) {
    // xSemaphoreTake(sd_mutex, portMAX_DELAY);
    while (!audio_buffer_empty(&audio_buffer)) {
      total += audio_buffer_dequeue(&audio_buffer, &audioFile);
    }
    // xSemaphoreGive(sd_mutex);
  }
  if (audio_buffer_full(&audio_buffer)) {
    iter_while_full++;
    Serial.print("Iter while full: ");
    Serial.println(iter_while_full);
  }
  // if imu is reset, re-enable required reports
  if (bno08x.wasReset()) {
    for (size_t i = 0; i < sizeof(reportTypes); i++) {
      setReports(reportTypes[i], reportIntervalUs);
    }
  }
  // pass received imu event to handler for parsing, buffering, writing
  if (bno08x.getSensorEvent(&sensorValue)) {
    handleSensorEvent(&sensorValue, &outputSensor);
  }

  if (stopWrite > 0) {
    noInterrupts();
    audioFile.close();
    interrupts();
    // Flush any remaining imu data to file
    size_t flushed = flushBuffer(&accel_buf, ACCEL_FNAME);
    flushed += flushBuffer(&gyro_buf, GYRO_FNAME);
    flushed += flushBuffer(&mag_buf, MAG_FNAME);

    // Flush remaining audio data to file
    noInterrupts();
    audioFile = SD.open(AUDIO_FNAME, FILE_WRITE);
    flushed += audio_buffer_dequeue(&audio_buffer, &audioFile);
    total += flushed;
    audioFile.close();
    interrupts();

    Serial.println("Done");
    Serial.print(total);
    Serial.print("B/");
    Serial.print(millis() - startTime);
    Serial.println("ms");
    Serial.print("Iter while full: ");
    Serial.println(iter_while_full);

    while (1) delay(1000);
  }
}
