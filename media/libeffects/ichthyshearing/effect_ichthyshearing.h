/*
 * Copyright (C) 2026 IchthysOS
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_EFFECT_ICHTHYSHEARING_H_
#define ANDROID_EFFECT_ICHTHYSHEARING_H_

#include <hardware/audio_effect.h>

#if __cplusplus
extern "C" {
#endif

/*
 * IchthysOS hearing compensation: an independent-per-ear graphic equaliser meant to sit on the
 * global output mix (session 0) and undo the user's measured hearing asymmetry. The Java side
 * (IchthysHearingController) owns the measurement and the prescription; this library only
 * applies the resulting curve.
 */

// b7e5f001-6a1f-4c2e-9a3b-1c7d5e8f0a21 — effect *type*.
#define ICHTHYS_HEARING_TYPE \
    { 0xb7e5f001, 0x6a1f, 0x4c2e, 0x9a3b, { 0x1c, 0x7d, 0x5e, 0x8f, 0x0a, 0x21 } }
// b7e5f002-6a1f-4c2e-9a3b-1c7d5e8f0a21 — this implementation.
#define ICHTHYS_HEARING_UUID \
    { 0xb7e5f002, 0x6a1f, 0x4c2e, 0x9a3b, { 0x1c, 0x7d, 0x5e, 0x8f, 0x0a, 0x21 } }

// Number of EQ bands per ear. Keep in sync with IchthysHearingController.BAND_FREQS.
#define ICHTHYS_HEARING_NUM_BANDS 8

typedef enum {
    // get: int32 — always ICHTHYS_HEARING_NUM_BANDS.
    ICHTHYS_HEARING_PARAM_NUM_BANDS = 0,
    // get: int32[ICHTHYS_HEARING_NUM_BANDS] — band centre frequencies in Hz.
    ICHTHYS_HEARING_PARAM_BAND_FREQS = 1,
    // set/get: int32[2 * ICHTHYS_HEARING_NUM_BANDS] — per-band gain in millibels, left ear
    // first then right ear. Clamped to [-2400, +1200] mB.
    ICHTHYS_HEARING_PARAM_BAND_GAINS = 2,
    // set/get: int32 — broadband make-up gain in millibels applied after the EQ, [0, +1200].
    ICHTHYS_HEARING_PARAM_MAKEUP_GAIN = 3,
} t_ichthys_hearing_params;

#define ICHTHYS_HEARING_MIN_BAND_GAIN_MB (-2400)
#define ICHTHYS_HEARING_MAX_BAND_GAIN_MB (1200)
#define ICHTHYS_HEARING_MAX_MAKEUP_MB (1200)

#if __cplusplus
}  // extern "C"
#endif

#endif  // ANDROID_EFFECT_ICHTHYSHEARING_H_
