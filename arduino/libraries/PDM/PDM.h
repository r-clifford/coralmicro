/*
 * Copyright 2022 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PDM_H_
#define PDM_H_

#include <memory>
#include <optional>

#include "Arduino.h"
#include "libs/audio/audio_driver.h"
#include "libs/audio/audio_service.h"

namespace coralmicro {
namespace arduino {

namespace {
inline constexpr int kNumDmaBuffers = 16;
inline constexpr int kDmaBufferSizeMs = 50;
inline constexpr int kAudioServiceTaskPriority = 5;
inline constexpr int kDropFirstSamplesMs = 150;
}  // namespace

// Exposes the Coral Micro device's native PDM microphone.
//
// You should not initialize this object yourself; instead include `PDM.h` and
// then use the global `Mic` instance.
class PDMClass {
 public:
  // @cond Do not generate docs.
  PDMClass();
  ~PDMClass();
  // @endcond

  // Start recording data with the PDM microphone.
  //
  // @param sample_rate The sample rate to start, only supports 16000 or 48000.
  int begin(int sample_rate = 16000);

  // Sets the current audio callback function. The microphone starts as soon
  // as `begin()` is called, however this function adds an extra callback that
  // gets executed as soon as new data are received. Within the callback, you
  // can call `available()` and `read()` to access the microphone data.
  //
  // @param function The function to call when audio data is received.  The
  // function should not have any arguments and not have a return value.
  void onReceive(void (*function)(void));

  // Removes the current callback, effectively turning off the microphone.
  //
  void end();

  // Gets the amount of available data in the audio buffer.
  // Data is stored in the buffer as `uint32_t`s, and the sizes
  // in this function refer to the amount of values in the buffer.
  //
  // @returns The amount of data values stored in the underlying buffer
  // that are ready to be read.
  int available();

  // Reads data from the audio buffer.
  // Data is stored in the buffer as `uint32_t`s, and the sizes
  // in this function refer to the amount of values in the buffer.
  //
  // @param buffer The buffer that will receive the copied audio data.
  // @param size The amount of audio data values to copy.
  // @returns The amount of audio data values that were copied.
  int read(std::vector<int32_t>& buffer, size_t size);

  // @cond Do not generate docs.
  // Not Implemented
  void setGain(int gain);
  void setBufferSize(int bufferSize);
  // @endcond

 private:
  void Append(const int32_t* samples, size_t num_samples);

  coralmicro::AudioDriverBuffers</*NumDmaBuffers=*/kNumDmaBuffers,
                                 /*CombinedDmaBufferSize=*/16 * 1024>
      audio_buffers_;
  AudioDriver driver_{audio_buffers_};
  std::unique_ptr<AudioDriverConfig> config_{nullptr};
  std::unique_ptr<AudioService> audio_service_{nullptr};
  std::unique_ptr<LatestSamples> latest_samples_{nullptr};
  std::optional<int> current_audio_cb_id_;
  void (*on_receive_)(void) = nullptr;
};

}  // namespace arduino
}  // namespace coralmicro

// This is the global `PDMClass` instance you should use instead of
// creating your own instance of `PDMClass`.
extern coralmicro::arduino::PDMClass Mic;

#endif  // PDM_H_
