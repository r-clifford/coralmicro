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

#ifndef LIBS_BASE_RESET_H_
#define LIBS_BASE_RESET_H_

#include <cstdint>

namespace coral::micro {

struct ResetStats {
    uint32_t reset_reason;
    uint32_t watchdog_resets;
    uint32_t lockup_resets;
};

void ResetToBootloader();
void ResetToFlash();
void StoreResetReason();
ResetStats GetResetStats();

}  // namespace coral::micro

#endif  // LIBS_BASE_RESET_H_
