#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <algorithm>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// ============================================================
//  AudioEngine.cpp
//  Real-time audio filtering for haptic feedback sleeve.
//
//  BAND COUNT matches MOTOR COUNT (12) so every mapper downstream
//  can use bands directly without any aggregation math.
//
//  Frequency layout — logarithmically spaced, 50 Hz → 8000 Hz.
//  Log spacing mirrors human perception: we feel low-frequency
//  differences more acutely than high-frequency ones, so lower
//  bands are narrower in Hz but equal in perceptual "size".
//
//  To change band boundaries, edit BAND_DEFS only.
//  BAND_COUNT drives everything else automatically.
//
//  Pipeline (per process() call):
//
//    Raw PCM
//      │
//      ▼
//    Pre-gain  +  Soft-clip (tanh)
//      │
//      ▼
//    DC Blocker  (~10 Hz one-pole HP)
//      │
//      ▼
//    Mains Notch Bank  (60 Hz harmonics, 2 Hz wide each)
//      │
//      ▼
//    Tonal Buzz Gate   (spectral flatness + transient guard)
//      │
//      ▼
//    12 × Band-pass filters  (4th-order Linkwitz-Riley)
//      │
//      ▼
//    FilterOutput  { envelope[12], peak[12] }
//      │
//      ▼
//    Mapper layer  (BandMapper / DirectMapper / DirectionalMapper)
//      │
//      ▼
//    MotorController  →  GPIO
// ============================================================


// ─────────────────────────────────────────────────────────────
//  Motor / band count
//  Change this if you add or remove motors.
//  Everything else scales automatically.
// ─────────────────────────────────────────────────────────────

static constexpr int MOTOR_ROWS    = 3;   // physical rows on the sleeve
static constexpr int MOTORS_PER_ROW = 4;  // motors per row
static constexpr int BAND_COUNT    = MOTOR_ROWS * MOTORS_PER_ROW;  // = 12


// ─────────────────────────────────────────────────────────────
//  DSP constants
//  These rarely need changing — tune INPUT_GAIN for your mic.
// ─────────────────────────────────────────────────────────────

static constexpr int   SAMPLE_RATE  = 44100;  // Hz

// Envelope follower — controls how quickly band energy tracks audio.
// Shorter attack = snappier response to onsets (good for haptics).
// Longer release = smoother decay, avoids jittery motor on/off.
static constexpr float ATTACK_TIME  = 0.002f;  // 2 ms
static constexpr float RELEASE_TIME = 0.050f;  // 50 ms

// Scale microphone input before filtering.
// Increase if motors barely fire; decrease if they always fire.
static constexpr float INPUT_GAIN   = 2.0f;


// ─────────────────────────────────────────────────────────────
//  Mains hum rejection
//  60 Hz for North America / Japan.  Change to 50.0f elsewhere.
//  Notches are placed at every harmonic up to MAX_HARMONIC_HZ.
//  BW_HZ controls notch width — 2 Hz is narrow enough that
//  nearby environmental sounds (jackhammer at 70 Hz) pass through.
// ─────────────────────────────────────────────────────────────

static constexpr float MAINS_HZ         = 60.0f;
static constexpr float MAINS_MAX_HZ     = 360.0f;  // 6th harmonic
static constexpr float MAINS_NOTCH_BW   = 2.0f;    // Hz


// ─────────────────────────────────────────────────────────────
//  Tonal buzz detector
//  Rejects stationary narrow-band noise (HVAC hum, fluorescent
//  lights) while passing irregular broadband sounds (traffic,
//  construction, speech).
//
//  TRANSIENT_RATIO — if short-term RMS > long-term × this value,
//    the signal is a real event (impact, voice onset) → gate open.
//    Lower = gate opens more easily. 2.5 works well in practice.
//
//  FLATNESS_THRESH — spectral flatness below this → signal is
//    tonal (buzz-like) → gate closes. Construction rumble sits
//    around 0.4–0.7; pure mains buzz sits around 0.01–0.05.
//    Raise if legitimate sounds are being gated; lower if buzz
//    is leaking through.
//
//  GATE_FLOOR — attenuation level when buzz is detected.
//    0.0 = full mute, 0.08 ≈ −22 dB (soft gate, not hard mute).
// ─────────────────────────────────────────────────────────────

static constexpr float TRANSIENT_RATIO  = 2.5f;
static constexpr float FLATNESS_THRESH  = 0.15f;
static constexpr float GATE_FLOOR       = 0.08f;
static constexpr float GATE_ATTACK      = 0.005f;  // 5 ms
static constexpr float GATE_RELEASE     = 0.080f;  // 80 ms


// ─────────────────────────────────────────────────────────────
//  Frequency band definitions
//
//  Each row = one band = one motor.
//  Bands are logarithmically spaced between LOW_FREQ and HIGH_FREQ.
//  LOW_HZ / HIGH_HZ are computed at init — edit the two constants
//  below to shift the entire range.
//
//  If you want to manually override individual band boundaries,
//  replace the log-spacing loop in AudioEngine::initFilters()
//  with a hand-written BAND_DEFS array — the rest of the code
//  will pick it up automatically.
// ─────────────────────────────────────────────────────────────

static constexpr float FREQ_LOW  =   50.0f;  // Hz — lowest band edge
static constexpr float FREQ_HIGH = 8000.0f;  // Hz — highest band edge

// Human-readable names — update if you change BAND_COUNT.
// Index must match the band index (0 = lowest frequency).
static constexpr const char* BAND_NAMES[BAND_COUNT] = {
    "Band 00  ( 50– 78 Hz)",   // Motor row 0, col 0
    "Band 01  ( 78–122 Hz)",   // Motor row 0, col 1
    "Band 02  (122–191 Hz)",   // Motor row 0, col 2
    "Band 03  (191–299 Hz)",   // Motor row 0, col 3
    "Band 04  (299–468 Hz)",   // Motor row 1, col 0
    "Band 05  (468–733 Hz)",   // Motor row 1, col 1
    "Band 06  (733–1147 Hz)",  // Motor row 1, col 2
    "Band 07  (1147–1796 Hz)", // Motor row 1, col 3
    "Band 08  (1796–2812 Hz)", // Motor row 2, col 0
    "Band 09  (2812–4402 Hz)", // Motor row 2, col 1
    "Band 10  (4402–6891 Hz)", // Motor row 2, col 2
    "Band 11  (6891–8000 Hz)", // Motor row 2, col 3
};


// ─────────────────────────────────────────────────────────────
//  FilterOutput
//  Plain data struct — no logic.  Passed between AudioEngine
//  and every mapper.  Index [0] = lowest frequency band.
// ─────────────────────────────────────────────────────────────

struct BandResult {
    float envelope = 0.0f;  // Smoothed energy, 0.0–1.0
    float peak     = 0.0f;  // Instantaneous peak, 0.0–1.0
};

struct FilterOutput {
    std::array<BandResult, BAND_COUNT> bands;

    // Convenience accessor so callers can write out[b] instead of out.bands[b]
    const BandResult& operator[](int b) const { return bands[b]; }
          BandResult& operator[](int b)       { return bands[b]; }
};


// ─────────────────────────────────────────────────────────────
//  BiquadFilter
//  Direct Form II Transposed — numerically stable, branch-free
//  inner loop.  Supports LP, HP, BP, and notch types.
//  Reference: Audio EQ Cookbook, Robert Bristow-Johnson.
// ─────────────────────────────────────────────────────────────

class BiquadFilter {
public:
    enum Type { LOW_PASS, HIGH_PASS, BAND_PASS, NOTCH };

    BiquadFilter() { reset(); }

    void configure(Type type, float freqHz, float sampleRate, float Q = 0.707f) {
        const float w0    = 2.0f * static_cast<float>(M_PI) * freqHz / sampleRate;
        const float cosW0 = std::cos(w0);
        const float sinW0 = std::sin(w0);
        const float alpha = sinW0 / (2.0f * Q);

        float b0, b1, b2, a0, a1, a2;

        switch (type) {
            case LOW_PASS:
                b0 = (1.0f - cosW0) / 2.0f;
                b1 =  1.0f - cosW0;
                b2 = (1.0f - cosW0) / 2.0f;
                a0 =  1.0f + alpha;
                a1 = -2.0f * cosW0;
                a2 =  1.0f - alpha;
                break;
            case HIGH_PASS:
                b0 =  (1.0f + cosW0) / 2.0f;
                b1 = -(1.0f + cosW0);
                b2 =  (1.0f + cosW0) / 2.0f;
                a0 =   1.0f + alpha;
                a1 =  -2.0f * cosW0;
                a2 =   1.0f - alpha;
                break;
            case NOTCH:
                b0 =  1.0f;
                b1 = -2.0f * cosW0;
                b2 =  1.0f;
                a0 =  1.0f + alpha;
                a1 = -2.0f * cosW0;
                a2 =  1.0f - alpha;
                break;
            case BAND_PASS:
            default:
                b0 =  sinW0 / 2.0f;
                b1 =  0.0f;
                b2 = -sinW0 / 2.0f;
                a0 =  1.0f + alpha;
                a1 = -2.0f * cosW0;
                a2 =  1.0f - alpha;
                break;
        }

        b0_ = b0/a0;  b1_ = b1/a0;  b2_ = b2/a0;
        a1_ = a1/a0;  a2_ = a2/a0;
        reset();
    }

    inline float process(float x) {
        const float y = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * y + z2_;
        z2_ = b2_ * x - a2_ * y;
        return y;
    }

    void reset() { z1_ = z2_ = 0.0f; }

private:
    float b0_ = 0, b1_ = 0, b2_ = 0;
    float a1_ = 0, a2_ = 0;
    float z1_ = 0, z2_ = 0;
};


// ─────────────────────────────────────────────────────────────
//  DCBlocker
//  One-pole HP at ~10 Hz.  Removes microphone DC offset before
//  any band filtering so DC doesn't saturate every band.
// ─────────────────────────────────────────────────────────────

class DCBlocker {
public:
    inline float process(float x) {
        const float y = x - xPrev_ + R_ * yPrev_;
        xPrev_ = x;
        yPrev_ = y;
        return y;
    }
    void reset() { xPrev_ = yPrev_ = 0.0f; }

private:
    static constexpr float R_ = 0.995f;  // ≈ 10 Hz corner @ 44100 Hz
    float xPrev_ = 0.0f;
    float yPrev_ = 0.0f;
};


// ─────────────────────────────────────────────────────────────
//  MainsNotchBank
//  Chain of narrow notch filters at mains harmonics.
//  All constructed from the MAINS_* constants above.
// ─────────────────────────────────────────────────────────────

class MainsNotchBank {
public:
    void configure(float mainsHz, float maxHz, float bwHz, float sampleRate) {
        notches_.clear();
        for (float f = mainsHz; f <= maxHz + 0.5f; f += mainsHz) {
            const float Q = f / bwHz;  // constant-BW: Q scales with frequency
            BiquadFilter n;
            n.configure(BiquadFilter::NOTCH, f, sampleRate, Q);
            notches_.push_back(n);
        }
    }

    inline float process(float x) {
        for (auto& n : notches_) x = n.process(x);
        return x;
    }

    void reset() { for (auto& n : notches_) n.reset(); }

private:
    std::vector<BiquadFilter> notches_;
};


// ─────────────────────────────────────────────────────────────
//  TonalBuzzDetector
//  Per-buffer two-stage gate:
//    Stage 1 — transient guard (short/long-term RMS ratio)
//    Stage 2 — spectral flatness measure (SFM)
//  See constant comments above for tuning guidance.
// ─────────────────────────────────────────────────────────────

class TonalBuzzDetector {
public:
    void configure(float sampleRate) {
        attackCoeff_  = 1.0f - std::exp(-1.0f / (sampleRate * GATE_ATTACK));
        releaseCoeff_ = 1.0f - std::exp(-1.0f / (sampleRate * GATE_RELEASE));
        ltRmsCoeff_   = 1.0f - std::exp(-1.0f / (sampleRate * 0.3f));
        reset();
    }

    // Returns gate gain in [GATE_FLOOR, 1.0].
    // Pass band envelopes as proxy spectral bins (no FFT needed).
    float analyse(const std::array<float, BAND_COUNT>& envs, float bufferRms) {
        // Stage 1: transient guard
        ltRms_ += ltRmsCoeff_ * (bufferRms - ltRms_);
        const bool isTransient = (ltRms_ > 1e-6f) &&
                                 (bufferRms / ltRms_ > TRANSIENT_RATIO);

        // Stage 2: spectral flatness measure
        float sumLin = 0.0f, sumLog = 0.0f;
        int   nActive = 0;
        for (int b = 0; b < BAND_COUNT; ++b) {
            const float e = envs[b];
            if (e > 1e-7f) { sumLin += e; sumLog += std::log(e); ++nActive; }
        }

        float sfm = 1.0f;
        if (nActive > 1) {
            const float geoMean = std::exp(sumLog / nActive);
            const float ariMean = sumLin / nActive;
            sfm = (ariMean > 1e-7f) ? (geoMean / ariMean) : 1.0f;
        }

        const float target = (!isTransient && sfm < FLATNESS_THRESH)
                                 ? GATE_FLOOR : 1.0f;

        const float coeff = (target > gateGain_) ? attackCoeff_ : releaseCoeff_;
        gateGain_ += coeff * (target - gateGain_);
        return gateGain_;
    }

    void reset() { gateGain_ = 1.0f; ltRms_ = 0.0f; }

private:
    float attackCoeff_  = 0.0f;
    float releaseCoeff_ = 0.0f;
    float ltRmsCoeff_   = 0.0f;
    float gateGain_     = 1.0f;
    float ltRms_        = 0.0f;
};


// ─────────────────────────────────────────────────────────────
//  PerBandFilter
//  4th-order Linkwitz-Riley crossover per band:
//    HP × 2  (low edge)  →  LP × 2  (high edge)
//  Edge bands (lowest / highest) use only one side.
//  LR crossovers sum to unity at boundaries — no gaps or peaks
//  when adjacent bands are both active.
// ─────────────────────────────────────────────────────────────

struct PerBandFilter {
    BiquadFilter lpA, lpB;  // cascaded LP stages (high edge)
    BiquadFilter hpA, hpB;  // cascaded HP stages (low edge)

    bool hasHP = true;
    bool hasLP = true;

    float envelope     = 0.0f;
    float attackCoeff  = 0.0f;
    float releaseCoeff = 0.0f;

    void configureEnvelope(float sr) {
        attackCoeff  = 1.0f - std::exp(-1.0f / (sr * ATTACK_TIME));
        releaseCoeff = 1.0f - std::exp(-1.0f / (sr * RELEASE_TIME));
    }

    // Returns { filtered_sample, envelope }
    inline std::pair<float, float> process(float x) {
        float y = x;
        if (hasHP) { y = hpA.process(y); y = hpB.process(y); }
        if (hasLP) { y = lpA.process(y); y = lpB.process(y); }

        const float absY = std::abs(y);
        if (absY > envelope)
            envelope += attackCoeff  * (absY - envelope);
        else
            envelope += releaseCoeff * (absY - envelope);

        return { y, envelope };
    }

    void reset() {
        lpA.reset(); lpB.reset();
        hpA.reset(); hpB.reset();
        envelope = 0.0f;
    }
};


// ─────────────────────────────────────────────────────────────
//  AudioEngine
//
//  One instance per microphone channel.
//  For directionality, instantiate one per mic and pass both
//  FilterOutputs to DirectionalMapper.
//
//  Call process() once per PortAudio callback buffer.
//  Returns FilterOutput — hand this to your active IMapper.
// ─────────────────────────────────────────────────────────────

class AudioEngine {
public:

    AudioEngine() { initFilters(); }

    // Call before streaming if you need a non-default sample rate or gain.
    void configure(float sampleRate, float inputGain = INPUT_GAIN) {
        sampleRate_ = sampleRate;
        inputGain_  = inputGain;
        initFilters();
    }

    // ── Main entry point ─────────────────────────────────────
    FilterOutput process(const std::vector<float>& inputBuffer) {
        FilterOutput out;

        std::array<float, BAND_COUNT> peakAcc;
        peakAcc.fill(0.0f);

        // Pre-compute buffer RMS for transient guard (pre-gain, pre-filter)
        float sumSq = 0.0f;
        for (float s : inputBuffer) sumSq += s * s;
        const float bufferRms =
            std::sqrt(sumSq / static_cast<float>(std::max(1, (int)inputBuffer.size())));

        for (float raw : inputBuffer) {
            // 1. Gain + soft clip
            const float gained  = raw * inputGain_;
            const float clipped = std::tanh(gained);

            // 2. DC block
            const float clean = dcBlocker_.process(clipped);

            // 3. Mains notch
            const float notched = mainsNotch_.process(clean);

            // 4. Per-band filters
            for (int b = 0; b < BAND_COUNT; ++b) {
                auto [filtered, env] = bands_[b].process(notched);
                (void)env;
                const float absFiltered = std::abs(filtered);
                if (absFiltered > peakAcc[b]) peakAcc[b] = absFiltered;
            }
        }

        // 5. Buzz gate (uses band envelopes as spectral proxy)
        std::array<float, BAND_COUNT> envSnap;
        for (int b = 0; b < BAND_COUNT; ++b) envSnap[b] = bands_[b].envelope;
        const float buzzGate = buzzDetector_.analyse(envSnap, bufferRms);

        // 6. Build output — apply gate and clamp
        for (int b = 0; b < BAND_COUNT; ++b) {
            out[b].envelope = std::clamp(bands_[b].envelope * buzzGate, 0.0f, 1.0f);
            out[b].peak     = std::clamp(peakAcc[b]         * buzzGate, 0.0f, 1.0f);
        }

        return out;
    }

    void reset() {
        dcBlocker_.reset();
        mainsNotch_.reset();
        buzzDetector_.reset();
        for (auto& b : bands_) b.reset();
    }

    float sampleRate() const { return sampleRate_; }

    static const char* bandName(int b) {
        if (b >= 0 && b < BAND_COUNT) return BAND_NAMES[b];
        return "Unknown";
    }


private:

    // ── Band boundary computation ────────────────────────────
    // Logarithmically spaces BAND_COUNT+1 edges between FREQ_LOW
    // and FREQ_HIGH, then builds a PerBandFilter for each slice.
    // To hard-code custom boundaries instead, replace the log-
    // spacing lines with a hand-written edges[] array.

    void initFilters() {
        const float sr = sampleRate_;

        // Compute log-spaced crossover frequencies
        // edges[0] = FREQ_LOW, edges[BAND_COUNT] = FREQ_HIGH
        float edges[BAND_COUNT + 1];
        for (int i = 0; i <= BAND_COUNT; ++i) {
            const float t = static_cast<float>(i) / BAND_COUNT;
            edges[i] = FREQ_LOW * std::pow(FREQ_HIGH / FREQ_LOW, t);
        }

        for (int b = 0; b < BAND_COUNT; ++b) {
            PerBandFilter& band = bands_[b];

            const float lowEdge  = edges[b];      // HP cutoff
            const float highEdge = edges[b + 1];  // LP cutoff

            band.hasHP = (b > 0);              // lowest band: no HP
            band.hasLP = (b < BAND_COUNT - 1); // highest band: no LP

            if (band.hasHP) {
                band.hpA.configure(BiquadFilter::HIGH_PASS, lowEdge,  sr);
                band.hpB.configure(BiquadFilter::HIGH_PASS, lowEdge,  sr);
            }
            if (band.hasLP) {
                band.lpA.configure(BiquadFilter::LOW_PASS,  highEdge, sr);
                band.lpB.configure(BiquadFilter::LOW_PASS,  highEdge, sr);
            }

            band.configureEnvelope(sr);
            band.reset();
        }

        dcBlocker_.reset();
        mainsNotch_.configure(MAINS_HZ, MAINS_MAX_HZ, MAINS_NOTCH_BW, sr);
        buzzDetector_.configure(sr);
    }

    float sampleRate_ = static_cast<float>(SAMPLE_RATE);
    float inputGain_  = INPUT_GAIN;

    DCBlocker         dcBlocker_;
    MainsNotchBank    mainsNotch_;
    TonalBuzzDetector buzzDetector_;
    std::array<PerBandFilter, BAND_COUNT> bands_;
};