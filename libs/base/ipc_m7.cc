// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "libs/base/ipc_m7.h"

#include <cstdio>
#include <cstring>
#include <memory>

#include "libs/base/console_m7.h"
#include "libs/base/message_buffer.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/message_buffer.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/multicore/mcmgr/src/mcmgr.h"

#define CORE1_BOOT_ADDRESS 0x20200000
unsigned int m4_binary_start __attribute__((weak)) = 0xdeadbeef;
unsigned int m4_binary_end __attribute__((weak)) = 0xdeadbeef;
unsigned int m4_binary_size __attribute__((weak)) = 0xdeadbeef;

namespace coral::micro {

uint8_t IPCM7::tx_queue_storage_[IPCM7::kMessageBufferSize +
                                 sizeof(ipc::MessageBuffer)]
    __attribute__((section(".noinit.$rpmsg_sh_mem")));
uint8_t IPCM7::rx_queue_storage_[IPCM7::kMessageBufferSize +
                                 sizeof(ipc::MessageBuffer)]
    __attribute__((section(".noinit.$rpmsg_sh_mem")));

void IPCM7::StaticRemoteAppEventHandler(uint16_t eventData, void* context) {
    GetSingleton()->RemoteAppEventHandler(eventData, context);
}

void IPCM7::RemoteAppEventHandler(uint16_t eventData, void* context) {
    BaseType_t reschedule =
        xTaskResumeFromISR(tx_task_) | xTaskResumeFromISR(rx_task_);
    portYIELD_FROM_ISR(reschedule);
}

void IPCM7::HandleSystemMessage(const ipc::SystemMessage& message) {
    switch (message.type) {
        default:
            printf("Unhandled system message type: %d\r\n",
                   static_cast<int>(message.type));
    }
}

void IPCM7::TxTaskFn() {
    // Send the rx_queue_ address to the M4.
    size_t tx_bytes;
    tx_bytes = xMessageBufferSend(tx_queue_->message_buffer, &rx_queue_,
                                  sizeof(rx_queue_), portMAX_DELAY);
    if (tx_bytes == 0) {
        printf("Failed to send s2p buffer address\r\n");
    }

    IPC::TxTaskFn();
}

bool IPCM7::M4IsAlive(uint32_t millis) {
    constexpr int kSleepMs = 100;
    uint32_t time_left = millis;
    do {
        eTaskState task_state = eTaskGetState(tx_task_);
        if (task_state == eReady || task_state == eBlocked) {
            return true;
        }
        if (time_left < kSleepMs) {
            time_left = 0;
        } else {
            time_left -= kSleepMs;
        }
        vTaskDelay(pdMS_TO_TICKS(kSleepMs));
    } while (time_left);
    return false;
}

bool IPCM7::HasM4Application() { return (m4_binary_start != 0xdeadbeef); }

void IPCM7::Init() {
    if (!HasM4Application()) {
        return;
    }

    tx_queue_ = reinterpret_cast<ipc::MessageBuffer*>(tx_queue_storage_);
    tx_queue_->message_buffer = xMessageBufferCreateStatic(
        kMessageBufferSize, tx_queue_->message_buffer_storage,
        &tx_queue_->static_message_buffer);
    if (!tx_queue_->message_buffer) {
        return;
    }

    rx_queue_ = reinterpret_cast<ipc::MessageBuffer*>(rx_queue_storage_);
    rx_queue_->message_buffer = xMessageBufferCreateStatic(
        kMessageBufferSize, rx_queue_->message_buffer_storage,
        &rx_queue_->static_message_buffer);
    if (!rx_queue_->message_buffer) {
        return;
    }

    // Load the remote core's memory space with the program binary.
    uint32_t m4_start = (uint32_t)&m4_binary_start;
    uint32_t m4_size = (uint32_t)&m4_binary_size;
    memcpy((void*)CORE1_BOOT_ADDRESS, (void*)m4_start, m4_size);

    // Register callbacks for communication with the other core.
    MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent,
                        IPCM7::StaticRemoteAppEventHandler, NULL);

    IPC::Init();
}

void IPCM7::StartM4() {
    if (!HasM4Application()) {
        return;
    }

    // Start up the remote core.
    // Provide the address of the P->S message queue so that the remote core can
    // receive messages from this core.
    MCMGR_StartCore(kMCMGR_Core1, (void*)CORE1_BOOT_ADDRESS,
                    reinterpret_cast<uint32_t>(tx_queue_),
                    kMCMGR_Start_Asynchronous);
}

}  // namespace coral::micro
