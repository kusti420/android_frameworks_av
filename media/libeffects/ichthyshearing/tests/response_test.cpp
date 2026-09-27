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

// Host test for the hearing-compensation effect. Drives the real effect through the real
// effect_handle_t interface and measures what comes out, because the failure mode that matters
// here is a filter that is stable and plausible-sounding but simply has the wrong gain — nothing
// crashes, the audiogram looks right, and the correction is quietly wrong.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include "effect_ichthyshearing.h"

extern "C" audio_effect_library_t AUDIO_EFFECT_LIBRARY_INFO_SYM;

namespace {

constexpr int kSampleRate = 48000;
constexpr int kBands = ICHTHYS_HEARING_NUM_BANDS;
const int kBandFreqs[kBands] = {125, 250, 500, 1000, 2000, 4000, 6000, 8000};

int gFailures = 0;

void expectNear(const char* what, double actual, double expected, double tolerance) {
    const bool ok = fabs(actual - expected) <= tolerance;
    if (!ok) {
        gFailures++;
    }
    printf("%-46s %7.2f  (expected %6.2f +/- %.2f)  %s\n", what, actual, expected, tolerance,
           ok ? "ok" : "FAIL");
}

struct Effect {
    effect_handle_t handle = nullptr;

    bool create() {
        const effect_uuid_t uuid = ICHTHYS_HEARING_UUID;
        if (AUDIO_EFFECT_LIBRARY_INFO_SYM.create_effect(&uuid, 0, 0, &handle) != 0) {
            printf("create_effect failed\n");
            return false;
        }
        return configure() && enable();
    }

    void release() {
        if (handle != nullptr) {
            AUDIO_EFFECT_LIBRARY_INFO_SYM.release_effect(handle);
            handle = nullptr;
        }
    }

    int command(uint32_t code, uint32_t cmdSize, void* cmd, uint32_t* replySize, void* reply) {
        return (*handle)->command(handle, code, cmdSize, cmd, replySize, reply);
    }

    bool configure() {
        effect_config_t config = {};
        config.inputCfg.samplingRate = kSampleRate;
        config.inputCfg.channels = AUDIO_CHANNEL_OUT_STEREO;
        config.inputCfg.format = AUDIO_FORMAT_PCM_FLOAT;
        config.inputCfg.accessMode = EFFECT_BUFFER_ACCESS_READ;
        config.inputCfg.mask = EFFECT_CONFIG_ALL;
        config.outputCfg = config.inputCfg;
        config.outputCfg.accessMode = EFFECT_BUFFER_ACCESS_WRITE;

        int reply = -1;
        uint32_t replySize = sizeof(reply);
        command(EFFECT_CMD_SET_CONFIG, sizeof(config), &config, &replySize, &reply);
        if (reply != 0) {
            printf("SET_CONFIG failed: %d\n", reply);
            return false;
        }
        return true;
    }

    bool enable() {
        int reply = -1;
        uint32_t replySize = sizeof(reply);
        command(EFFECT_CMD_ENABLE, 0, nullptr, &replySize, &reply);
        return reply == 0;
    }

    void reset() { command(EFFECT_CMD_RESET, 0, nullptr, nullptr, nullptr); }

    /** Sends a parameter exactly the way the JNI layer packs one. */
    bool setParam(int32_t param, const int32_t* values, size_t count) {
        std::vector<uint8_t> buf(sizeof(effect_param_t) + sizeof(int32_t) +
                                 count * sizeof(int32_t));
        effect_param_t* p = reinterpret_cast<effect_param_t*>(buf.data());
        p->status = 0;
        p->psize = sizeof(int32_t);
        p->vsize = count * sizeof(int32_t);
        memcpy(p->data, &param, sizeof(int32_t));
        memcpy(p->data + sizeof(int32_t), values, count * sizeof(int32_t));

        int32_t reply = -1;
        uint32_t replySize = sizeof(reply);
        command(EFFECT_CMD_SET_PARAM, buf.size(), buf.data(), &replySize, &reply);
        return reply == 0;
    }

    bool setBandGains(const int32_t gainsMb[2 * kBands]) {
        return setParam(ICHTHYS_HEARING_PARAM_BAND_GAINS, gainsMb, 2 * kBands);
    }

    bool setMakeup(int32_t mb) {
        return setParam(ICHTHYS_HEARING_PARAM_MAKEUP_GAIN, &mb, 1);
    }

    bool getBandGains(int32_t out[2 * kBands]) {
        uint8_t cmd[sizeof(effect_param_t) + sizeof(int32_t)];
        effect_param_t* c = reinterpret_cast<effect_param_t*>(cmd);
        c->status = 0;
        c->psize = sizeof(int32_t);
        c->vsize = 2 * kBands * sizeof(int32_t);
        const int32_t param = ICHTHYS_HEARING_PARAM_BAND_GAINS;
        memcpy(c->data, &param, sizeof(int32_t));

        std::vector<uint8_t> reply(sizeof(effect_param_t) + sizeof(int32_t) +
                                   2 * kBands * sizeof(int32_t));
        uint32_t replySize = reply.size();
        command(EFFECT_CMD_GET_PARAM, sizeof(cmd), cmd, &replySize, reply.data());
        effect_param_t* r = reinterpret_cast<effect_param_t*>(reply.data());
        if (r->status != 0 || r->vsize != 2 * kBands * sizeof(int32_t)) {
            return false;
        }
        memcpy(out, r->data + sizeof(int32_t), 2 * kBands * sizeof(int32_t));
        return true;
    }

    /**
     * Feeds a sine into one channel and returns the steady-state gain in dB. The first 300 ms is
     * discarded so the biquads' transient is not measured as frequency response.
     */
    double measureGainDb(double freq, int channel, double amplitude = 0.1) {
        const int settleFrames = kSampleRate * 3 / 10;
        const int measureFrames = kSampleRate / 2;
        const int total = settleFrames + measureFrames;

        std::vector<float> in(total * 2, 0.f);
        std::vector<float> out(total * 2, 0.f);
        const double w = 2 * M_PI * freq / kSampleRate;
        for (int i = 0; i < total; i++) {
            in[i * 2 + channel] = static_cast<float>(sin(w * i) * amplitude);
        }

        reset();
        audio_buffer_t inBuf = {};
        audio_buffer_t outBuf = {};
        inBuf.frameCount = total;
        inBuf.f32 = in.data();
        outBuf.frameCount = total;
        outBuf.f32 = out.data();
        if ((*handle)->process(handle, &inBuf, &outBuf) != 0) {
            return -999;
        }

        double sumOut = 0;
        size_t counted = 0;
        for (size_t idx = static_cast<size_t>(settleFrames) * 2 + channel; idx < out.size();
             idx += 2) {
            const double s = out[idx];
            sumOut += s * s;
            counted++;
        }
        const double rmsOut = sqrt(sumOut / static_cast<double>(counted));
        const double rmsIn = amplitude / sqrt(2.0);
        return 20 * log10(rmsOut / rmsIn);
    }

    /** Peak absolute sample across both channels for a full-scale input. */
    double measurePeak(double freq, double amplitude) {
        const int total = kSampleRate / 2;
        std::vector<float> in(total * 2);
        std::vector<float> out(total * 2, 0.f);
        const double w = 2 * M_PI * freq / kSampleRate;
        for (int i = 0; i < total; i++) {
            const float s = static_cast<float>(sin(w * i) * amplitude);
            in[i * 2] = s;
            in[i * 2 + 1] = s;
        }
        reset();
        audio_buffer_t inBuf = {};
        audio_buffer_t outBuf = {};
        inBuf.frameCount = total;
        inBuf.f32 = in.data();
        outBuf.frameCount = total;
        outBuf.f32 = out.data();
        (*handle)->process(handle, &inBuf, &outBuf);

        // Skip the limiter's attack: what matters is the level it settles to. Both channels are
        // fed the same signal, so a flat contiguous scan of the tail is the peak across the pair
        // — and a contiguous scan keeps the compiler from widening a strided load past the end
        // of the buffer, which an every-other-sample version of this loop provably did.
        double peak = 0;
        const float* samples = out.data();
        const size_t first = static_cast<size_t>(kSampleRate / 10) * 2;
        for (size_t idx = first; idx < out.size(); idx++) {
            const double magnitude = fabs(static_cast<double>(samples[idx]));
            if (magnitude > peak) {
                peak = magnitude;
            }
        }
        return peak;
    }
};

void zeroGains(int32_t gains[2 * kBands]) {
    memset(gains, 0, 2 * kBands * sizeof(int32_t));
}

}  // namespace

int main() {
    Effect fx;
    if (!fx.create()) {
        return 1;
    }

    int32_t gains[2 * kBands];

    // A flat profile must be transparent, or every user who has not run the test yet pays for it.
    zeroGains(gains);
    fx.setBandGains(gains);
    fx.setMakeup(0);
    printf("\n-- flat profile is transparent --\n");
    for (int b = 0; b < kBands; b++) {
        char name[64];
        snprintf(name, sizeof(name), "flat @ %d Hz", kBandFreqs[b]);
        expectNear(name, fx.measureGainDb(kBandFreqs[b], 0), 0.0, 0.1);
    }

    // Each peaking band must deliver its requested gain at its own centre frequency.
    printf("\n-- peaking bands hit their requested gain at centre --\n");
    for (int b = 1; b < kBands - 1; b++) {
        zeroGains(gains);
        gains[b] = -600;
        fx.setBandGains(gains);
        char name[64];
        snprintf(name, sizeof(name), "-6 dB band @ %d Hz", kBandFreqs[b]);
        expectNear(name, fx.measureGainDb(kBandFreqs[b], 0), -6.0, 0.5);
    }

    // The shelves carry the correction past the ends of the measured range: half the gain at the
    // corner, the whole of it further out. That is the entire reason they are shelves.
    printf("\n-- shelves extend past the measured range --\n");
    zeroGains(gains);
    gains[0] = -600;
    fx.setBandGains(gains);
    expectNear("low shelf @ 125 Hz corner", fx.measureGainDb(125, 0), -3.0, 0.7);
    expectNear("low shelf @ 40 Hz", fx.measureGainDb(40, 0), -6.0, 0.7);

    zeroGains(gains);
    gains[kBands - 1] = -600;
    fx.setBandGains(gains);
    expectNear("high shelf @ 8 kHz corner", fx.measureGainDb(8000, 0), -3.0, 0.7);
    expectNear("high shelf @ 16 kHz", fx.measureGainDb(16000, 0), -6.0, 0.7);

    // The whole point of the effect: the two ears are independent.
    printf("\n-- the ears are independent --\n");
    zeroGains(gains);
    gains[3] = -1200;              // left, 1 kHz
    gains[kBands + 3] = 0;         // right, 1 kHz
    fx.setBandGains(gains);
    expectNear("left  1 kHz cut by 12 dB", fx.measureGainDb(1000, 0), -12.0, 0.5);
    expectNear("right 1 kHz untouched", fx.measureGainDb(1000, 1), 0.0, 0.2);

    gains[3] = 0;
    gains[kBands + 3] = -900;
    fx.setBandGains(gains);
    expectNear("left  1 kHz untouched", fx.measureGainDb(1000, 0), 0.0, 0.2);
    expectNear("right 1 kHz cut by 9 dB", fx.measureGainDb(1000, 1), -9.0, 0.5);

    printf("\n-- make-up gain and limiter --\n");
    zeroGains(gains);
    fx.setBandGains(gains);
    fx.setMakeup(600);
    expectNear("+6 dB make-up on a quiet signal", fx.measureGainDb(1000, 0, 0.05), 6.0, 0.2);

    fx.setMakeup(1200);
    const double peak = fx.measurePeak(1000, 0.9);
    printf("%-46s %7.4f  (must not exceed 1.0)          %s\n", "limiter on a hot signal", peak,
           peak <= 1.0 ? "ok" : "FAIL");
    if (peak > 1.0) {
        gFailures++;
    }
    // A limiter that never lets anything through would also pass the line above.
    expectNear("limiter still passes audio", 20 * log10(peak / 0.9), 0.0, 3.0);

    printf("\n-- parameters round-trip and clamp --\n");
    fx.setMakeup(0);
    zeroGains(gains);
    gains[0] = -9999;   // below the floor
    gains[1] = 9999;    // above the ceiling
    gains[2] = -543;
    fx.setBandGains(gains);
    int32_t readBack[2 * kBands];
    if (!fx.getBandGains(readBack)) {
        printf("GET_PARAM band gains FAILED\n");
        gFailures++;
    } else {
        expectNear("clamped to floor", readBack[0], ICHTHYS_HEARING_MIN_BAND_GAIN_MB, 0);
        expectNear("clamped to ceiling", readBack[1], ICHTHYS_HEARING_MAX_BAND_GAIN_MB, 0);
        expectNear("in-range value preserved", readBack[2], -543, 0);
    }

    fx.release();
    printf("\n%s (%d failure%s)\n", gFailures == 0 ? "PASS" : "FAIL", gFailures,
           gFailures == 1 ? "" : "s");
    return gFailures == 0 ? 0 : 1;
}
