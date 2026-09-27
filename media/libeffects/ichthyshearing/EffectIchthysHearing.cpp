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

// IchthysOS hearing compensation effect: an 8-band graphic EQ with fully independent left and
// right curves, plus a stereo-linked peak limiter to keep the make-up gain from clipping.
//
// It is designed to be attached to the global output mix (session 0) so every stream on the
// device is corrected, which is why the two ears are separate: the whole point is to undo an
// asymmetric hearing loss, and a normal Equalizer effect gains all channels together.
//
// Band 0 is a low shelf and band 7 a high shelf, so the correction keeps going below 125 Hz and
// above 8 kHz instead of falling back to flat at the edges of the measured range. The six bands
// in between are RBJ peaking filters at a deliberately low Q: a hearing-loss curve is smooth, and
// overlapping wide bands reconstruct it far better than narrow surgical ones.

#define LOG_TAG "EffectIchthysHearing"
//#define LOG_NDEBUG 0

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <new>

#include <log/log.h>

#include "effect_ichthyshearing.h"

static constexpr audio_format_t kProcessFormat = AUDIO_FORMAT_PCM_FLOAT;
static constexpr int kNumBands = ICHTHYS_HEARING_NUM_BANDS;

// Band centre frequencies. The audiometric set (125 Hz .. 8 kHz) — anything higher is beyond
// what the test can measure reliably through headphones and beyond what most lossy codecs pass.
static constexpr float kBandFreqs[kNumBands] = {125.f, 250.f, 500.f, 1000.f,
                                                2000.f, 4000.f, 6000.f, 8000.f};
// ~1.3 octaves wide. Wide enough that adjacent bands sum to a smooth curve rather than a comb.
static constexpr float kPeakingQ = 1.0f;

// Peak limiter: catches the transients that make-up gain would otherwise push past full scale.
static constexpr float kLimiterThreshold = 0.891f;  // -1 dBFS
static constexpr float kLimiterAttackMs = 1.5f;
static constexpr float kLimiterReleaseMs = 120.f;

// Below this a biquad's state is inaudible, and zeroing it keeps denormals out of the inner loop
// on cores where flush-to-zero is not set for us.
static constexpr float kDenormalFloor = 1e-25f;

enum ichthys_state_e {
    ICHTHYS_STATE_UNINITIALIZED,
    ICHTHYS_STATE_INITIALIZED,
    ICHTHYS_STATE_ACTIVE,
};

// Which of the three coefficient sets a given output channel is filtered with.
enum coeff_set_e {
    COEFF_LEFT = 0,
    COEFF_RIGHT = 1,
    // Used for mono output and for anything past the front L/R pair (centre, surrounds), where
    // "which ear" is not a meaningful question. Holds the mean of the two ears' gains.
    COEFF_MONO = 2,
    COEFF_SET_COUNT = 3,
};

struct Biquad {
    // Normalised (a0 == 1) transposed-direct-form-II coefficients.
    float b0, b1, b2, a1, a2;
};

struct BiquadState {
    float z1, z2;
};

struct IchthysHearingContext {
    const struct effect_interface_s* mItfe;
    effect_config_t mConfig;
    uint8_t mState;

    // Prescription, as handed down by IchthysHearingController.
    int32_t mBandGainMb[2][kNumBands];
    int32_t mMakeupGainMb;

    Biquad mCoeffs[COEFF_SET_COUNT][kNumBands];
    BiquadState mStates[FCC_LIMIT][kNumBands];
    float mMakeupLinear;
    float mLimiterGain;
    float mLimiterAttack;
    float mLimiterRelease;
};

//
//--- Filter design (RBJ audio EQ cookbook)
//

static void designUnity(Biquad* f) {
    f->b0 = 1.f;
    f->b1 = 0.f;
    f->b2 = 0.f;
    f->a1 = 0.f;
    f->a2 = 0.f;
}

static void normalise(Biquad* f, float b0, float b1, float b2, float a0, float a1, float a2) {
    const float inv = 1.f / a0;
    f->b0 = b0 * inv;
    f->b1 = b1 * inv;
    f->b2 = b2 * inv;
    f->a1 = a1 * inv;
    f->a2 = a2 * inv;
}

static void designPeaking(Biquad* f, float freq, float fs, float gainDb, float q) {
    const float A = powf(10.f, gainDb / 40.f);
    const float w0 = 2.f * (float)M_PI * freq / fs;
    const float cosw0 = cosf(w0);
    const float alpha = sinf(w0) / (2.f * q);
    normalise(f, 1.f + alpha * A, -2.f * cosw0, 1.f - alpha * A, 1.f + alpha / A, -2.f * cosw0,
              1.f - alpha / A);
}

// Shelf slope S = 1 (the steepest slope that stays monotonic), so alpha reduces to
// sin(w0)/2 * sqrt(2).
static void designShelf(Biquad* f, float freq, float fs, float gainDb, bool high) {
    const float A = powf(10.f, gainDb / 40.f);
    const float w0 = 2.f * (float)M_PI * freq / fs;
    const float cosw0 = cosf(w0);
    const float alpha = sinf(w0) * 0.5f * (float)M_SQRT2;
    const float twoSqrtAAlpha = 2.f * sqrtf(A) * alpha;
    const float Ap1 = A + 1.f;
    const float Am1 = A - 1.f;
    if (high) {
        normalise(f, A * (Ap1 + Am1 * cosw0 + twoSqrtAAlpha), -2.f * A * (Am1 + Ap1 * cosw0),
                  A * (Ap1 + Am1 * cosw0 - twoSqrtAAlpha), Ap1 - Am1 * cosw0 + twoSqrtAAlpha,
                  2.f * (Am1 - Ap1 * cosw0), Ap1 - Am1 * cosw0 - twoSqrtAAlpha);
    } else {
        normalise(f, A * (Ap1 - Am1 * cosw0 + twoSqrtAAlpha), 2.f * A * (Am1 - Ap1 * cosw0),
                  A * (Ap1 - Am1 * cosw0 - twoSqrtAAlpha), Ap1 + Am1 * cosw0 + twoSqrtAAlpha,
                  -2.f * (Am1 + Ap1 * cosw0), Ap1 + Am1 * cosw0 - twoSqrtAAlpha);
    }
}

static void designSet(Biquad* set, const float* gainsDb, float fs) {
    for (int b = 0; b < kNumBands; b++) {
        const float freq = kBandFreqs[b];
        // A band at or above Nyquist has nothing to shape, and the cookbook formulas degenerate
        // there. Leave it flat rather than emitting an unstable filter.
        if (freq >= fs * 0.45f || fabsf(gainsDb[b]) < 0.01f) {
            designUnity(&set[b]);
        } else if (b == 0) {
            designShelf(&set[b], freq, fs, gainsDb[b], false /*high*/);
        } else if (b == kNumBands - 1) {
            designShelf(&set[b], freq, fs, gainsDb[b], true /*high*/);
        } else {
            designPeaking(&set[b], freq, fs, gainsDb[b], kPeakingQ);
        }
    }
}

//
//--- Local functions
//

static void IH_updateCoefficients(IchthysHearingContext* pContext) {
    const float fs = (float)pContext->mConfig.inputCfg.samplingRate;
    if (fs <= 0.f) {
        return;
    }
    float gains[COEFF_SET_COUNT][kNumBands];
    for (int b = 0; b < kNumBands; b++) {
        gains[COEFF_LEFT][b] = pContext->mBandGainMb[0][b] / 100.f;
        gains[COEFF_RIGHT][b] = pContext->mBandGainMb[1][b] / 100.f;
        gains[COEFF_MONO][b] = 0.5f * (gains[COEFF_LEFT][b] + gains[COEFF_RIGHT][b]);
    }
    for (int s = 0; s < COEFF_SET_COUNT; s++) {
        designSet(pContext->mCoeffs[s], gains[s], fs);
    }
    pContext->mMakeupLinear = powf(10.f, pContext->mMakeupGainMb / 2000.f);
    pContext->mLimiterAttack = 1.f - expf(-1000.f / (kLimiterAttackMs * fs));
    pContext->mLimiterRelease = 1.f - expf(-1000.f / (kLimiterReleaseMs * fs));
}

static void IH_reset(IchthysHearingContext* pContext) {
    memset(pContext->mStates, 0, sizeof(pContext->mStates));
    pContext->mLimiterGain = 1.f;
}

static int IH_setConfig(IchthysHearingContext* pContext, effect_config_t* pConfig) {
    if (pConfig->inputCfg.samplingRate != pConfig->outputCfg.samplingRate) return -EINVAL;
    if (pConfig->inputCfg.channels != pConfig->outputCfg.channels) return -EINVAL;
    if (pConfig->inputCfg.format != pConfig->outputCfg.format) return -EINVAL;
    if (audio_channel_count_from_out_mask(pConfig->inputCfg.channels) > FCC_LIMIT) return -EINVAL;
    if (pConfig->outputCfg.accessMode != EFFECT_BUFFER_ACCESS_WRITE &&
        pConfig->outputCfg.accessMode != EFFECT_BUFFER_ACCESS_ACCUMULATE) {
        return -EINVAL;
    }
    if (pConfig->inputCfg.format != kProcessFormat) return -EINVAL;
    if (pConfig->inputCfg.samplingRate == 0) return -EINVAL;

    pContext->mConfig = *pConfig;
    IH_updateCoefficients(pContext);
    IH_reset(pContext);
    return 0;
}

static void IH_getConfig(IchthysHearingContext* pContext, effect_config_t* pConfig) {
    *pConfig = pContext->mConfig;
}

static int IH_init(IchthysHearingContext* pContext) {
    pContext->mConfig.inputCfg.accessMode = EFFECT_BUFFER_ACCESS_READ;
    pContext->mConfig.inputCfg.channels = AUDIO_CHANNEL_OUT_STEREO;
    pContext->mConfig.inputCfg.format = kProcessFormat;
    pContext->mConfig.inputCfg.samplingRate = 48000;
    pContext->mConfig.inputCfg.bufferProvider.getBuffer = NULL;
    pContext->mConfig.inputCfg.bufferProvider.releaseBuffer = NULL;
    pContext->mConfig.inputCfg.bufferProvider.cookie = NULL;
    pContext->mConfig.inputCfg.mask = EFFECT_CONFIG_ALL;
    pContext->mConfig.outputCfg.accessMode = EFFECT_BUFFER_ACCESS_ACCUMULATE;
    pContext->mConfig.outputCfg.channels = AUDIO_CHANNEL_OUT_STEREO;
    pContext->mConfig.outputCfg.format = kProcessFormat;
    pContext->mConfig.outputCfg.samplingRate = 48000;
    pContext->mConfig.outputCfg.bufferProvider.getBuffer = NULL;
    pContext->mConfig.outputCfg.bufferProvider.releaseBuffer = NULL;
    pContext->mConfig.outputCfg.bufferProvider.cookie = NULL;
    pContext->mConfig.outputCfg.mask = EFFECT_CONFIG_ALL;

    // Flat until a profile arrives, so an enabled-but-unconfigured effect is a no-op.
    memset(pContext->mBandGainMb, 0, sizeof(pContext->mBandGainMb));
    pContext->mMakeupGainMb = 0;

    return IH_setConfig(pContext, &pContext->mConfig);
}

//
//--- Effect Library Interface Implementation
//

extern "C" {

extern const struct effect_interface_s gIchthysHearingInterface;

const effect_descriptor_t gIchthysHearingDescriptor = {
        ICHTHYS_HEARING_TYPE,
        ICHTHYS_HEARING_UUID,
        EFFECT_CONTROL_API_VERSION,
        // Insert-last: hearing compensation is the final shaping stage, applied to whatever the
        // rest of the chain produced.
        (EFFECT_FLAG_TYPE_INSERT | EFFECT_FLAG_INSERT_LAST),
        0,  // cpuLoad
        1,  // memoryUsage
        "IchthysOS Hearing Compensation",
        "IchthysOS",
};

// The session and io handles are the effect framework's business, not ours: the curve is the
// same wherever this instance is attached. Unnamed rather than __unused, which is bionic-only and
// so unavailable to the host test.
static int IHLib_Create(const effect_uuid_t* uuid, int32_t /*sessionId*/, int32_t /*ioId*/,
                        effect_handle_t* pHandle) {
    if (pHandle == NULL || uuid == NULL) {
        return -EINVAL;
    }
    if (memcmp(uuid, &gIchthysHearingDescriptor.uuid, sizeof(effect_uuid_t)) != 0) {
        return -EINVAL;
    }

    IchthysHearingContext* pContext = new IchthysHearingContext;
    memset(pContext, 0, sizeof(*pContext));
    pContext->mItfe = &gIchthysHearingInterface;
    pContext->mState = ICHTHYS_STATE_UNINITIALIZED;

    const int ret = IH_init(pContext);
    if (ret < 0) {
        ALOGW("IHLib_Create() init failed");
        delete pContext;
        return ret;
    }

    *pHandle = (effect_handle_t)pContext;
    pContext->mState = ICHTHYS_STATE_INITIALIZED;
    ALOGV("IHLib_Create context is %p", pContext);
    return 0;
}

static int IHLib_Release(effect_handle_t handle) {
    IchthysHearingContext* pContext = (IchthysHearingContext*)handle;
    if (pContext == NULL) {
        return -EINVAL;
    }
    pContext->mState = ICHTHYS_STATE_UNINITIALIZED;
    delete pContext;
    return 0;
}

static int IHLib_GetDescriptor(const effect_uuid_t* uuid, effect_descriptor_t* pDescriptor) {
    if (pDescriptor == NULL || uuid == NULL) {
        return -EINVAL;
    }
    if (memcmp(uuid, &gIchthysHearingDescriptor.uuid, sizeof(effect_uuid_t)) == 0) {
        *pDescriptor = gIchthysHearingDescriptor;
        return 0;
    }
    return -EINVAL;
}

//
//--- Effect Control Interface Implementation
//

static inline float runBiquad(const Biquad& f, BiquadState& s, float x) {
    const float y = f.b0 * x + s.z1;
    s.z1 = f.b1 * x - f.a1 * y + s.z2;
    s.z2 = f.b2 * x - f.a2 * y;
    if (fabsf(s.z1) < kDenormalFloor) s.z1 = 0.f;
    if (fabsf(s.z2) < kDenormalFloor) s.z2 = 0.f;
    return y;
}

static int IH_process(effect_handle_t self, audio_buffer_t* inBuffer, audio_buffer_t* outBuffer) {
    IchthysHearingContext* pContext = (IchthysHearingContext*)self;

    if (pContext == NULL) {
        return -EINVAL;
    }
    if (inBuffer == NULL || inBuffer->raw == NULL || outBuffer == NULL || outBuffer->raw == NULL ||
        inBuffer->frameCount != outBuffer->frameCount || inBuffer->frameCount == 0) {
        return -EINVAL;
    }

    const size_t channelCount =
            audio_channel_count_from_out_mask(pContext->mConfig.outputCfg.channels);
    const size_t frameCount = inBuffer->frameCount;
    const bool accumulate =
            pContext->mConfig.outputCfg.accessMode == EFFECT_BUFFER_ACCESS_ACCUMULATE;

    if (pContext->mState != ICHTHYS_STATE_ACTIVE) {
        // Still has to hand the audio on, but must report that it produced nothing of its own so
        // the framework can drop us out of the chain.
        if (inBuffer->raw != outBuffer->raw) {
            const size_t sampleCount = frameCount * channelCount;
            if (accumulate) {
                for (size_t i = 0; i < sampleCount; i++) {
                    outBuffer->f32[i] += inBuffer->f32[i];
                }
            } else {
                memcpy(outBuffer->raw, inBuffer->raw, sampleCount * sizeof(float));
            }
        }
        return -ENODATA;
    }

    const float makeup = pContext->mMakeupLinear;
    float limGain = pContext->mLimiterGain;

    for (size_t i = 0; i < frameCount; i++) {
        float frame[FCC_LIMIT];
        float peak = 0.f;

        for (size_t c = 0; c < channelCount; c++) {
            // Channel 0 is the left ear and channel 1 the right; a mono mix and any channel past
            // the front pair get the average of the two curves.
            int set;
            if (channelCount == 1) {
                set = COEFF_MONO;
            } else if (c == 0) {
                set = COEFF_LEFT;
            } else if (c == 1) {
                set = COEFF_RIGHT;
            } else {
                set = COEFF_MONO;
            }

            float x = inBuffer->f32[i * channelCount + c];
            for (int b = 0; b < kNumBands; b++) {
                x = runBiquad(pContext->mCoeffs[set][b], pContext->mStates[c][b], x);
            }
            x *= makeup;
            frame[c] = x;
            const float mag = fabsf(x);
            if (mag > peak) peak = mag;
        }

        // Stereo-linked limiter: one gain for every channel, so ducking a loud transient never
        // shifts the stereo image — which matters more than usual here, because the whole effect
        // exists to get the left/right balance right.
        const float target = (peak > kLimiterThreshold) ? (kLimiterThreshold / peak) : 1.f;
        const float coef = (target < limGain) ? pContext->mLimiterAttack : pContext->mLimiterRelease;
        limGain += (target - limGain) * coef;

        for (size_t c = 0; c < channelCount; c++) {
            // The limiter has no lookahead, so a fast transient can still poke through before the
            // gain catches up; clamp as a last resort rather than letting it wrap downstream.
            float y = frame[c] * limGain;
            if (y > 1.f) y = 1.f;
            else if (y < -1.f) y = -1.f;
            const size_t idx = i * channelCount + c;
            if (accumulate) {
                outBuffer->f32[idx] += y;
            } else {
                outBuffer->f32[idx] = y;
            }
        }
    }

    pContext->mLimiterGain = limGain;
    return 0;
}

static int IH_command(effect_handle_t self, uint32_t cmdCode, uint32_t cmdSize, void* pCmdData,
                      uint32_t* replySize, void* pReplyData) {
    IchthysHearingContext* pContext = (IchthysHearingContext*)self;

    if (pContext == NULL || pContext->mState == ICHTHYS_STATE_UNINITIALIZED) {
        return -EINVAL;
    }

    switch (cmdCode) {
        case EFFECT_CMD_INIT:
            if (pReplyData == NULL || replySize == NULL || *replySize != sizeof(int)) {
                return -EINVAL;
            }
            *(int*)pReplyData = IH_init(pContext);
            break;

        case EFFECT_CMD_SET_CONFIG:
            if (pCmdData == NULL || cmdSize != sizeof(effect_config_t) || pReplyData == NULL ||
                replySize == NULL || *replySize != sizeof(int)) {
                return -EINVAL;
            }
            *(int*)pReplyData = IH_setConfig(pContext, (effect_config_t*)pCmdData);
            break;

        case EFFECT_CMD_GET_CONFIG:
            if (pReplyData == NULL || replySize == NULL || *replySize != sizeof(effect_config_t)) {
                return -EINVAL;
            }
            IH_getConfig(pContext, (effect_config_t*)pReplyData);
            break;

        case EFFECT_CMD_RESET:
            IH_reset(pContext);
            break;

        case EFFECT_CMD_ENABLE:
            if (pReplyData == NULL || replySize == NULL || *replySize != sizeof(int)) {
                return -EINVAL;
            }
            if (pContext->mState != ICHTHYS_STATE_INITIALIZED) {
                return -ENOSYS;
            }
            IH_reset(pContext);
            pContext->mState = ICHTHYS_STATE_ACTIVE;
            *(int*)pReplyData = 0;
            break;

        case EFFECT_CMD_DISABLE:
            if (pReplyData == NULL || replySize == NULL || *replySize != sizeof(int)) {
                return -EINVAL;
            }
            if (pContext->mState != ICHTHYS_STATE_ACTIVE) {
                return -ENOSYS;
            }
            pContext->mState = ICHTHYS_STATE_INITIALIZED;
            *(int*)pReplyData = 0;
            break;

        case EFFECT_CMD_GET_PARAM: {
            if (pCmdData == NULL || cmdSize < (uint32_t)(sizeof(effect_param_t) + sizeof(int32_t)) ||
                pReplyData == NULL || replySize == NULL ||
                *replySize < (uint32_t)(sizeof(effect_param_t) + sizeof(int32_t))) {
                return -EINVAL;
            }
            effect_param_t* pCmd = (effect_param_t*)pCmdData;
            if (pCmd->psize != sizeof(int32_t)) {
                return -EINVAL;
            }
            memcpy(pReplyData, pCmdData, sizeof(effect_param_t) + sizeof(int32_t));
            effect_param_t* p = (effect_param_t*)pReplyData;
            p->status = 0;
            int32_t* pValue = (int32_t*)(p->data + sizeof(int32_t));
            const uint32_t headerSize = sizeof(effect_param_t) + sizeof(int32_t);
            const uint32_t room = *replySize - headerSize;

            switch (*(int32_t*)p->data) {
                case ICHTHYS_HEARING_PARAM_NUM_BANDS:
                    if (room < sizeof(int32_t)) {
                        p->status = -EINVAL;
                        break;
                    }
                    *pValue = kNumBands;
                    p->vsize = sizeof(int32_t);
                    break;
                case ICHTHYS_HEARING_PARAM_BAND_FREQS:
                    if (room < sizeof(int32_t) * kNumBands) {
                        p->status = -EINVAL;
                        break;
                    }
                    for (int b = 0; b < kNumBands; b++) {
                        pValue[b] = (int32_t)kBandFreqs[b];
                    }
                    p->vsize = sizeof(int32_t) * kNumBands;
                    break;
                case ICHTHYS_HEARING_PARAM_BAND_GAINS:
                    if (room < sizeof(int32_t) * 2 * kNumBands) {
                        p->status = -EINVAL;
                        break;
                    }
                    for (int e = 0; e < 2; e++) {
                        for (int b = 0; b < kNumBands; b++) {
                            pValue[e * kNumBands + b] = pContext->mBandGainMb[e][b];
                        }
                    }
                    p->vsize = sizeof(int32_t) * 2 * kNumBands;
                    break;
                case ICHTHYS_HEARING_PARAM_MAKEUP_GAIN:
                    if (room < sizeof(int32_t)) {
                        p->status = -EINVAL;
                        break;
                    }
                    *pValue = pContext->mMakeupGainMb;
                    p->vsize = sizeof(int32_t);
                    break;
                default:
                    p->status = -EINVAL;
                    break;
            }
            if (p->status != 0) {
                p->vsize = 0;
            }
            *replySize = headerSize + p->vsize;
        } break;

        case EFFECT_CMD_SET_PARAM: {
            if (pCmdData == NULL || cmdSize < (uint32_t)(sizeof(effect_param_t) + sizeof(int32_t)) ||
                pReplyData == NULL || replySize == NULL || *replySize != sizeof(int32_t)) {
                return -EINVAL;
            }
            effect_param_t* p = (effect_param_t*)pCmdData;
            if (p->psize != sizeof(int32_t) ||
                cmdSize < sizeof(effect_param_t) + p->psize + p->vsize) {
                *(int32_t*)pReplyData = -EINVAL;
                break;
            }
            const int32_t* pValue = (const int32_t*)(p->data + sizeof(int32_t));
            *(int32_t*)pReplyData = 0;

            switch (*(int32_t*)p->data) {
                case ICHTHYS_HEARING_PARAM_BAND_GAINS: {
                    if (p->vsize != sizeof(int32_t) * 2 * kNumBands) {
                        *(int32_t*)pReplyData = -EINVAL;
                        break;
                    }
                    for (int e = 0; e < 2; e++) {
                        for (int b = 0; b < kNumBands; b++) {
                            int32_t mb = pValue[e * kNumBands + b];
                            if (mb < ICHTHYS_HEARING_MIN_BAND_GAIN_MB) {
                                mb = ICHTHYS_HEARING_MIN_BAND_GAIN_MB;
                            } else if (mb > ICHTHYS_HEARING_MAX_BAND_GAIN_MB) {
                                mb = ICHTHYS_HEARING_MAX_BAND_GAIN_MB;
                            }
                            pContext->mBandGainMb[e][b] = mb;
                        }
                    }
                    IH_updateCoefficients(pContext);
                } break;
                case ICHTHYS_HEARING_PARAM_MAKEUP_GAIN: {
                    if (p->vsize != sizeof(int32_t)) {
                        *(int32_t*)pReplyData = -EINVAL;
                        break;
                    }
                    int32_t mb = *pValue;
                    if (mb < 0) {
                        mb = 0;
                    } else if (mb > ICHTHYS_HEARING_MAX_MAKEUP_MB) {
                        mb = ICHTHYS_HEARING_MAX_MAKEUP_MB;
                    }
                    pContext->mMakeupGainMb = mb;
                    IH_updateCoefficients(pContext);
                } break;
                default:
                    *(int32_t*)pReplyData = -EINVAL;
                    break;
            }
        } break;

        case EFFECT_CMD_SET_DEVICE:
        case EFFECT_CMD_SET_VOLUME:
        case EFFECT_CMD_SET_AUDIO_MODE:
            break;

        default:
            ALOGW("IH_command invalid command %d", cmdCode);
            return -EINVAL;
    }

    return 0;
}

static int IH_getDescriptor(effect_handle_t self, effect_descriptor_t* pDescriptor) {
    IchthysHearingContext* pContext = (IchthysHearingContext*)self;
    if (pContext == NULL || pDescriptor == NULL) {
        return -EINVAL;
    }
    *pDescriptor = gIchthysHearingDescriptor;
    return 0;
}

const struct effect_interface_s gIchthysHearingInterface = {
        IH_process,
        IH_command,
        IH_getDescriptor,
        NULL,
};

__attribute__((visibility("default"))) audio_effect_library_t AUDIO_EFFECT_LIBRARY_INFO_SYM = {
        .tag = AUDIO_EFFECT_LIBRARY_TAG,
        .version = EFFECT_LIBRARY_API_VERSION,
        .name = "IchthysOS Hearing Compensation Library",
        .implementor = "IchthysOS",
        .create_effect = IHLib_Create,
        .release_effect = IHLib_Release,
        .get_descriptor = IHLib_GetDescriptor,
};

}  // extern "C"
