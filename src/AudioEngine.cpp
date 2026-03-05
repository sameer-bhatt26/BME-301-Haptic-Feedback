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
//    Mains Notch Bank  ─── removes 50/60 Hz hum + harmonics up to ~360 Hz
//      │                    Notch BW is very narrow (~2 Hz) so broadband
//      │                    construction noise at those freqs passes through.
//      ▼
//    Tonal Buzz Detector  ─── per-buffer analysis:
//      │                       • measures short-term vs long-term energy ratio
//      │                       • if signal is stationary AND tonal (narrow-band
//      │                         spike above broadband floor), apply soft gate
//      │                       • transients always bypass the gate entirely
//      ▼
//    Band filters  (4th-order Linkwitz-Riley crossovers)
//      ├── Sub-bass   (20–80 Hz)
//      ├── Bass       (80–300 Hz)
//      ├── Low-mid    (300–800 Hz)
//      ├── Mid        (800–2500 Hz)
//      ├── High-mid   (2500–6000 Hz)
//      └── Presence   (6000–16000 Hz)
//
//  Each band exposes:
//    • envelope  – smoothed RMS energy  (0.0–1.0, for band/direct mapping)
//    • peak      – instantaneous peak   (0.0–1.0, for transient detection)
//
//  Downstream mapping layers (NOT implemented here) will consume
//  FilterOutput and translate it to PWM values:
//    1. Directionality  – compare per-mic envelopes
//    2. Band mapping    – combine bands into low/high groups per motor
//    3. Direct mapping  – one band → one dedicated motor
//
//  Noise rejection design rationale:
//    Electrical buzz and construction noise overlap in frequency (50–300 Hz),
//    so a band-cut alone would remove useful environmental sound.
//    Instead we exploit two properties unique to mains/electrical interference:
//      (a) Fixed known frequencies  → surgical notch filters
//      (b) Highly stationary & tonal → buzz detector gates only steady,
//          narrow-band energy while letting irregular broadband noise pass.
//    A transient guard ensures hammers, impacts, and footsteps always pass
//    regardless of spectral shape.
//
//  Target latency contribution: < 2 ms (filter math only).
//  Total system budget: < 20 ms  (audio I/O + filter + PWM write).
// ============================================================


// ─────────────────────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────────────────────

static constexpr int   SAMPLE_RATE   = 44100;   // Hz
static constexpr int   MAX_MOTORS    = 10;

// Envelope follower time constants (seconds → per-sample coefficients)
// Shorter attack = faster response to transients (good for haptics)
// Longer release = prevents buzzy flutter on rapid signal changes
static constexpr float ATTACK_TIME   = 0.002f;  // 2 ms
static constexpr float RELEASE_TIME  = 0.050f;  // 50 ms

// Input gain applied before any filtering.
// Tune for your microphone's sensitivity; 1.0 = unity gain.
static constexpr float INPUT_GAIN    = 2.0f;

// ── Mains frequency ──────────────────────────────────────────
// Set to 60.0f for North America / Japan, 50.0f for Europe / Asia / AU.
// All harmonics up to MAINS_MAX_HARMONIC_HZ are notched automatically.
static constexpr float MAINS_FREQ_HZ          = 60.0f;
static constexpr float MAINS_MAX_HARMONIC_HZ  = 360.0f;  // 60×6 or 50×7
// Notch bandwidth: narrower = more musical content preserved around the notch.
// 2 Hz is tight enough to leave construction rumble almost untouched.
static constexpr float MAINS_NOTCH_BW_HZ      = 2.0f;

// ── Tonal buzz detector ──────────────────────────────────────
// How many buffers of history the long-term energy averager holds.
// At 256 samples / 44100 Hz ≈ 5.8 ms per buffer → ~48 buffers ≈ 280 ms.
// Longer = more confident stationarity detection, slightly slower response.
static constexpr int   BUZZ_HISTORY_BUFFERS   = 48;

// If short-term energy > long-term by this ratio, it's a transient → bypass gate.
// 2.5× means "50% louder than recent average" = a hit, footstep, door slam, etc.
static constexpr float TRANSIENT_RATIO        = 2.5f;

// Spectral flatness threshold.  Sfm in [0,1]; 1=white noise, 0=pure sine.
// Below this value the signal is considered "too tonal" to be real-world noise.
// 0.15 passes construction rumble (sfm ~0.4–0.7) while catching buzz (sfm ~0.02).
static constexpr float TONAL_FLATNESS_THRESH  = 0.15f;

// Gate depth when buzz is detected.  0.0 = full mute, 0.1 = –20 dB attenuation.
// A soft gate (not full mute) preserves very loud buzz that might carry real info.
static constexpr float BUZZ_GATE_FLOOR        = 0.08f;

// Smoothing for gate open/close transitions — prevents clicks.
// Attack = fast open (don't miss real sounds), release = slow close.
static constexpr float GATE_ATTACK_TIME       = 0.005f;  // 5 ms
static constexpr float GATE_RELEASE_TIME      = 0.080f;  // 80 ms


// ─────────────────────────────────────────────────────────────
//  Band identifiers
//  Keep this enum in sync with BAND_DEFS below.
// ─────────────────────────────────────────────────────────────

enum Band : int {
    BAND_SUB_BASS  = 0,   //  20 –   80 Hz  — deep rumble / explosions
    BAND_BASS      = 1,   //  80 –  300 Hz  — body of speech / music bass
    BAND_LOW_MID   = 2,   // 300 –  800 Hz  — fundamental vowels / warmth
    BAND_MID       = 3,   // 800 – 2500 Hz  — speech intelligibility core
    BAND_HIGH_MID  = 4,   // 2500– 6000 Hz  — consonants / attack transients
    BAND_PRESENCE  = 5,   // 6000–16000 Hz  — air / sibilance
    BAND_COUNT
};


// ─────────────────────────────────────────────────────────────
//  FilterOutput
//  Everything downstream mapping needs.  Pure data, no methods.
// ─────────────────────────────────────────────────────────────

struct BandResult {
    float envelope = 0.0f;   // Smoothed energy,  0.0–1.0
    float peak     = 0.0f;   // Instantaneous peak, 0.0–1.0
};

struct FilterOutput {
    std::array<BandResult, BAND_COUNT> bands;

    // Convenience accessors
    const BandResult& operator[](Band b)  const { return bands[b]; }
          BandResult& operator[](Band b)        { return bands[b]; }
};


// ─────────────────────────────────────────────────────────────
//  Biquad filter
//
//  Direct Form II Transposed — numerically stable, cache-friendly.
//  Supports: low-pass, high-pass, band-pass (peak-normalised).
//
//  Reference: Audio EQ Cookbook – Robert Bristow-Johnson
// ─────────────────────────────────────────────────────────────

class BiquadFilter {
public:
    enum Type { LOW_PASS, HIGH_PASS, BAND_PASS, NOTCH };

    BiquadFilter() { reset(); }

    void configure(Type type, float cutoffHz, float sampleRate, float Q = 0.707f) {
        const float w0    = 2.0f * M_PI * cutoffHz / sampleRate;
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
                // Bandstop (notch): unity gain everywhere except at cutoffHz.
                // Q = cutoffHz / BW — caller should compute Q from desired BW.
                b0 =  1.0f;
                b1 = -2.0f * cosW0;
                b2 =  1.0f;
                a0 =  1.0f + alpha;
                a1 = -2.0f * cosW0;
                a2 =  1.0f - alpha;
                break;

            case BAND_PASS:
            default:
                // Peak-normalised band-pass (0 dB at centre)
                b0 =  sinW0 / 2.0f;
                b1 =  0.0f;
                b2 = -sinW0 / 2.0f;
                a0 =  1.0f + alpha;
                a1 = -2.0f * cosW0;
                a2 =  1.0f - alpha;
                break;
        }

        // Normalise by a0
        b0_ = b0 / a0;  b1_ = b1 / a0;  b2_ = b2 / a0;
        a1_ = a1 / a0;  a2_ = a2 / a0;

        reset();
    }

    // Process one sample. Call in tight loop — no branches inside.
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
//  DC Blocker
//  Simple one-pole high-pass at ~10 Hz.
//  Removes microphone DC offset that would saturate every band.
// ─────────────────────────────────────────────────────────────

class DCBlocker {
public:
    inline float process(float x) {
        // y[n] = x[n] - x[n-1] + R * y[n-1]
        const float y = x - xPrev_ + R_ * yPrev_;
        xPrev_ = x;
        yPrev_ = y;
        return y;
    }
    void reset() { xPrev_ = yPrev_ = 0.0f; }

private:
    static constexpr float R_ = 0.995f;   // ≈ 10 Hz corner at 44100 Hz
    float xPrev_ = 0.0f;
    float yPrev_ = 0.0f;
};


// ─────────────────────────────────────────────────────────────
//  MainsNotchBank
//
//  Builds a chain of notch biquads at every mains harmonic from
//  MAINS_FREQ_HZ up to MAINS_MAX_HARMONIC_HZ.
//
//  For 60 Hz with 2 Hz BW this removes:  60, 120, 180, 240, 300, 360 Hz
//  For 50 Hz with 2 Hz BW this removes:  50, 100, 150, 200, 250, 300, 350 Hz
//
//  Each notch is only 2 Hz wide, so a 70 Hz jackhammer impact sits 10 Hz
//  away from the 60 Hz notch and passes essentially unaffected.
//
//  Q = centre_freq / bandwidth, computed per harmonic so BW stays
//  constant in Hz (not proportional to frequency).
// ─────────────────────────────────────────────────────────────

class MainsNotchBank {
public:
    void configure(float mainsHz, float maxHarmonicHz,
                   float bwHz, float sampleRate) {
        notches_.clear();
        for (float f = mainsHz; f <= maxHarmonicHz + 0.5f; f += mainsHz) {
            const float Q = f / bwHz;   // constant-BW design
            BiquadFilter n;
            n.configure(BiquadFilter::NOTCH, f, sampleRate, Q);
            notches_.push_back(n);
        }
    }

    inline float process(float x) {
        for (auto& n : notches_)
            x = n.process(x);
        return x;
    }

    void reset() {
        for (auto& n : notches_) n.reset();
    }

private:
    std::vector<BiquadFilter> notches_;
};


// ─────────────────────────────────────────────────────────────
//  TonalBuzzDetector
//
//  Detects steady, narrow-band interference that is NOT a real
//  environmental event and applies a soft amplitude gate to it.
//
//  Two-stage decision per buffer:
//
//  Stage 1 — Transient guard
//    Compare short-term RMS to a slow-moving long-term RMS average.
//    If short/long > TRANSIENT_RATIO, declare a transient: gate = 1.0 (open).
//    This means any sudden impact (hammer, door, footstep) bypasses the gate
//    completely, even if it happens to be somewhat tonal.
//
//  Stage 2 — Spectral flatness measure (SFM)
//    SFM = geometric_mean(|X[k]|) / arithmetic_mean(|X[k]|)
//    Pure sine → SFM ≈ 0.  White noise → SFM ≈ 1.
//    Computed over a lightweight set of LOG-spaced magnitude bins estimated
//    from the per-sample running state (no FFT needed — we use the band
//    envelope outputs as proxy bins, which is fast and sufficient).
//
//    If SFM < TONAL_FLATNESS_THRESH  AND  no transient detected:
//        gate target = BUZZ_GATE_FLOOR  (attenuate)
//    else:
//        gate target = 1.0             (pass through)
//
//  The gate itself is a smoothed gain value that ramps between targets
//  using asymmetric attack/release to avoid clicks.
// ─────────────────────────────────────────────────────────────

class TonalBuzzDetector {
public:
    void configure(float sampleRate,
                   float attackSec  = GATE_ATTACK_TIME,
                   float releaseSec = GATE_RELEASE_TIME) {
        attackCoeff_  = 1.0f - std::exp(-1.0f / (sampleRate * attackSec));
        releaseCoeff_ = 1.0f - std::exp(-1.0f / (sampleRate * releaseSec));
        ltRmsCoeff_   = 1.0f - std::exp(-1.0f / (sampleRate * 0.3f)); // 300 ms LT window
        reset();
    }

    /**
     * @brief   Analyse one buffer and return a gate gain in [BUZZ_GATE_FLOOR, 1.0].
     *
     * @param   bandEnvelopes   The BAND_COUNT per-band envelope values from the
     *                          current buffer (used as proxy spectral bins for SFM).
     * @param   bufferPeakRms   RMS of the raw pre-filter buffer this frame.
     * @return  Gate gain:  1.0 = fully open (pass), BUZZ_GATE_FLOOR = attenuated.
     */
    float analyse(const std::array<float, BAND_COUNT>& bandEnvelopes,
                  float bufferRms) {
        // ── Stage 1: Transient guard ─────────────────────────
        // Update long-term RMS
        ltRms_ += ltRmsCoeff_ * (bufferRms - ltRms_);

        const bool isTransient = (ltRms_ > 1e-6f) &&
                                 (bufferRms / ltRms_ > TRANSIENT_RATIO);

        // ── Stage 2: Spectral flatness measure ───────────────
        // Use band envelopes as proxy magnitude bins.
        // SFM = geometric_mean / arithmetic_mean
        float sumLin  = 0.0f;
        float sumLog  = 0.0f;
        int   nActive = 0;

        for (int b = 0; b < BAND_COUNT; ++b) {
            const float e = bandEnvelopes[b];
            if (e > 1e-7f) {    // skip silent bands (avoid log(0))
                sumLin += e;
                sumLog += std::log(e);
                ++nActive;
            }
        }

        float sfm = 1.0f;   // default: flat (noise-like) — safe fallback
        if (nActive > 1) {
            const float geoMean  = std::exp(sumLog  / nActive);
            const float ariMean  = sumLin / nActive;
            sfm = (ariMean > 1e-7f) ? (geoMean / ariMean) : 1.0f;
        }

        const bool isTonal = (sfm < TONAL_FLATNESS_THRESH);

        // ── Gate target ──────────────────────────────────────
        // Open gate if transient OR signal is spectrally flat (broadband)
        const float targetGain = (!isTransient && isTonal) ? BUZZ_GATE_FLOOR : 1.0f;

        // ── Smooth gate transitions ──────────────────────────
        // Use fast attack (don't miss real sounds) and slow release
        const float coeff = (targetGain > gateGain_) ? attackCoeff_ : releaseCoeff_;
        gateGain_ += coeff * (targetGain - gateGain_);

        return gateGain_;
    }

    void reset() {
        gateGain_ = 1.0f;
        ltRms_    = 0.0f;
    }

private:
    float attackCoeff_  = 0.0f;
    float releaseCoeff_ = 0.0f;
    float ltRmsCoeff_   = 0.0f;
    float gateGain_     = 1.0f;
    float ltRms_        = 0.0f;
};


// ─────────────────────────────────────────────────────────────
//  PerBandFilter
//
//  Each band uses a cascade of two biquads (4th-order Linkwitz-Riley):
//      HP biquad (low edge)  →  LP biquad (high edge)
//
//  This gives a steeper roll-off (~24 dB/oct) while keeping adjacent
//  bands summing to unity at crossover — important for haptic coherence.
//
//  Sub-bass and Presence are half-band (LP or HP only) since they sit
//  at the spectrum edges.
// ─────────────────────────────────────────────────────────────

struct PerBandFilter {
    BiquadFilter lpA, lpB;   // Cascaded LP stages  (or just lpA for HP-only)
    BiquadFilter hpA, hpB;   // Cascaded HP stages  (or just hpA for LP-only)

    bool hasHP = true;
    bool hasLP = true;

    // Envelope follower state
    float envelope  = 0.0f;
    float attackCoeff  = 0.0f;
    float releaseCoeff = 0.0f;

    void configureEnvelope(float attackSec, float releaseSec, float sr) {
        attackCoeff  = 1.0f - std::exp(-1.0f / (sr * attackSec));
        releaseCoeff = 1.0f - std::exp(-1.0f / (sr * releaseSec));
    }

    // Returns (filtered_sample, envelope)
    inline std::pair<float, float> process(float x) {
        float y = x;

        if (hasHP) {
            y = hpA.process(y);
            y = hpB.process(y);
        }
        if (hasLP) {
            y = lpA.process(y);
            y = lpB.process(y);
        }

        // Full-wave rectify, then envelope follow
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
//  For directionality mapping, instantiate one engine per mic and
//  compare their FilterOutputs in the motor-mapping layer.
// ─────────────────────────────────────────────────────────────

class AudioEngine {
public:

    // ── Construction / configuration ────────────────────────

    AudioEngine() {
        initFilters();
    }

    // Call once before streaming if you need a different sample rate.
    void configure(float sampleRate, float inputGain = INPUT_GAIN) {
        sampleRate_ = sampleRate;
        inputGain_  = inputGain;
        initFilters();
    }

    // ── Main entry point ─────────────────────────────────────

    /**
     * @brief  Process one buffer of raw PCM audio.
     *
     * @param  inputBuffer   Floating-point samples, range [-1.0, +1.0].
     *                       For int16 input: divide by 32768.0f before passing.
     * @return FilterOutput  One BandResult per frequency band.
     *                       Downstream mapping layers read this struct.
     *
     * Call this once per audio callback (~5–10 ms at 256–512 samples).
     * Internal per-sample work is O(N × BAND_COUNT × 8 multiplies).
     */
    FilterOutput process(const std::vector<float>& inputBuffer) {
        FilterOutput out;

        // Track peak per band across the whole buffer
        std::array<float, BAND_COUNT> peakAcc;
        peakAcc.fill(0.0f);

        // Compute buffer RMS for the transient guard (pre-filter, pre-gain)
        float sumSq = 0.0f;
        for (float s : inputBuffer) sumSq += s * s;
        const float bufferRms = std::sqrt(sumSq / std::max(1, (int)inputBuffer.size()));

        for (float rawSample : inputBuffer) {

            // 1. Input gain + soft clip (tanh) — avoids hard clipping artefacts
            const float gained  = rawSample * inputGain_;
            const float clipped = std::tanh(gained);   // |output| < 1.0

            // 2. DC block — must come before band filters
            const float dcBlocked = dcBlocker_.process(clipped);

            // 3. Mains notch bank — surgically removes 50/60 Hz harmonics.
            //    Broadband signals at nearby frequencies pass through unchanged.
            const float notched = mainsNotch_.process(dcBlocked);

            // 4. Per-band filtering + envelope update
            //    (Buzz gate is applied after this loop using band envelopes)
            for (int b = 0; b < BAND_COUNT; ++b) {
                auto [filtered, env] = bands_[b].process(notched);
                (void)env;

                const float absFiltered = std::abs(filtered);
                if (absFiltered > peakAcc[b])
                    peakAcc[b] = absFiltered;
            }
        }

        // 5. Collect raw band envelopes for buzz analysis
        std::array<float, BAND_COUNT> envSnapshot;
        for (int b = 0; b < BAND_COUNT; ++b)
            envSnapshot[b] = bands_[b].envelope;

        // 6. Tonal buzz detector — returns a gate gain in [BUZZ_GATE_FLOOR, 1.0].
        //    Gate is 1.0 (fully open) for:
        //      • any transient (construction impact, footstep, voice onset)
        //      • spectrally flat (broadband) signals — traffic, wind, machinery
        //    Gate attenuates toward BUZZ_GATE_FLOOR only for:
        //      • stationary AND narrow-band signals — electrical hum, HVAC tones
        const float buzzGate = buzzDetector_.analyse(envSnapshot, bufferRms);

        // 7. Build output struct — apply buzz gate and clamp to [0, 1]
        for (int b = 0; b < BAND_COUNT; ++b) {
            out.bands[b].envelope = std::clamp(bands_[b].envelope * buzzGate, 0.0f, 1.0f);
            out.bands[b].peak     = std::clamp(peakAcc[b]         * buzzGate, 0.0f, 1.0f);
        }

        return out;
    }

    // ── Utility ──────────────────────────────────────────────

    void reset() {
        dcBlocker_.reset();
        mainsNotch_.reset();
        buzzDetector_.reset();
        for (auto& b : bands_)
            b.reset();
    }

    float sampleRate() const { return sampleRate_; }

    // Human-readable band name (handy for debugging / logging)
    static const char* bandName(Band b) {
        switch (b) {
            case BAND_SUB_BASS:  return "Sub-bass  (20-80 Hz)";
            case BAND_BASS:      return "Bass      (80-300 Hz)";
            case BAND_LOW_MID:   return "Low-mid   (300-800 Hz)";
            case BAND_MID:       return "Mid       (800-2500 Hz)";
            case BAND_HIGH_MID:  return "High-mid  (2500-6000 Hz)";
            case BAND_PRESENCE:  return "Presence  (6000-16000 Hz)";
            default:             return "Unknown";
        }
    }


private:

    // ── Internal types ───────────────────────────────────────

    struct BandDef {
        float lowHz;    // 0 = no high-pass
        float highHz;   // 0 = no low-pass
        float Q;        // Filter Q at each crossover
    };

    // ── Band definitions ─────────────────────────────────────
    //  lowHz == 0  → this is the lowest band; only LP stages applied
    //  highHz == 0 → this is the highest band; only HP stages applied

    static constexpr std::array<BandDef, BAND_COUNT> BAND_DEFS = {{
        { 0.0f,    80.0f,   0.707f },   // SUB_BASS
        { 80.0f,   300.0f,  0.707f },   // BASS
        { 300.0f,  800.0f,  0.707f },   // LOW_MID
        { 800.0f,  2500.0f, 0.707f },   // MID
        { 2500.0f, 6000.0f, 0.707f },   // HIGH_MID
        { 6000.0f, 0.0f,    0.707f },   // PRESENCE
    }};

    // ── State ────────────────────────────────────────────────

    float sampleRate_ = static_cast<float>(SAMPLE_RATE);
    float inputGain_  = INPUT_GAIN;

    DCBlocker  dcBlocker_;
    MainsNotchBank mainsNotch_;
    TonalBuzzDetector buzzDetector_;
    std::array<PerBandFilter, BAND_COUNT> bands_;

    // ── Initialisation ───────────────────────────────────────

    void initFilters() {
        const float sr = sampleRate_;

        for (int b = 0; b < BAND_COUNT; ++b) {
            const auto& def = BAND_DEFS[b];
            PerBandFilter& band = bands_[b];

            band.hasHP = (def.lowHz  > 0.0f);
            band.hasLP = (def.highHz > 0.0f);

            if (band.hasHP) {
                band.hpA.configure(BiquadFilter::HIGH_PASS, def.lowHz,  sr, def.Q);
                band.hpB.configure(BiquadFilter::HIGH_PASS, def.lowHz,  sr, def.Q);
            }
            if (band.hasLP) {
                band.lpA.configure(BiquadFilter::LOW_PASS,  def.highHz, sr, def.Q);
                band.lpB.configure(BiquadFilter::LOW_PASS,  def.highHz, sr, def.Q);
            }

            band.configureEnvelope(ATTACK_TIME, RELEASE_TIME, sr);
            band.reset();
        }

        dcBlocker_.reset();

        mainsNotch_.configure(MAINS_FREQ_HZ, MAINS_MAX_HARMONIC_HZ,
                              MAINS_NOTCH_BW_HZ, sr);
        buzzDetector_.configure(sr);
    }
};


// ─────────────────────────────────────────────────────────────
//  Quick usage example (compiled out in production builds)
// ─────────────────────────────────────────────────────────────
#ifdef AUDIO_ENGINE_DEMO

#include <iostream>

int main() {
    AudioEngine engine;

    // Simulate a 256-sample buffer at 44100 Hz (~5.8 ms)
    std::vector<float> buffer(256);
    for (int i = 0; i < 256; ++i)
        buffer[i] = 0.5f * std::sin(2.0f * M_PI * 440.0f * i / 44100.0f);

    FilterOutput result = engine.process(buffer);

    std::cout << "Band envelopes:\n";
    for (int b = 0; b < BAND_COUNT; ++b) {
        std::cout << "  " << AudioEngine::bandName(static_cast<Band>(b))
                  << "  env=" << result.bands[b].envelope
                  << "  peak=" << result.bands[b].peak << "\n";
    }
    return 0;
}

#endif // AUDIO_ENGINE_DEMO