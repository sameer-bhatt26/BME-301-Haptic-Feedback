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
//  BAND_COUNT matches MOTOR_COUNT (12) so every mapper downstream
//  can use bands directly without aggregation math.
//
//  Frequency layout — logarithmically spaced, 50 Hz to 8000 Hz.
//  To change band boundaries edit FREQ_LOW / FREQ_HIGH.
//  To manually set individual crossovers, replace the log-spacing
//  loop in initFilters() with a hand-written edges[] array.
// ============================================================


// ─────────────────────────────────────────────────────────────
//  Motor / band count
// ─────────────────────────────────────────────────────────────

static constexpr int MOTOR_ROWS     = 3;
static constexpr int MOTORS_PER_ROW = 4;
static constexpr int BAND_COUNT     = MOTOR_ROWS * MOTORS_PER_ROW;  // 12


// ─────────────────────────────────────────────────────────────
//  DSP constants
// ─────────────────────────────────────────────────────────────

static constexpr int   SAMPLE_RATE  = 44100;
static constexpr float ATTACK_TIME  = 0.002f;   // 2 ms  — onset response
static constexpr float RELEASE_TIME = 0.050f;   // 50 ms — decay smoothing
static constexpr float INPUT_GAIN   = 2.0f;     // mic sensitivity scale


// ─────────────────────────────────────────────────────────────
//  Mains hum rejection
//  60 Hz for North America / Japan. Change to 50.0f elsewhere.
// ─────────────────────────────────────────────────────────────

static constexpr float MAINS_HZ       = 60.0f;
static constexpr float MAINS_MAX_HZ   = 360.0f;
static constexpr float MAINS_NOTCH_BW = 2.0f;


// ─────────────────────────────────────────────────────────────
//  Tonal buzz detector tuning
// ─────────────────────────────────────────────────────────────

static constexpr float TRANSIENT_RATIO = 2.5f;
static constexpr float FLATNESS_THRESH = 0.15f;
static constexpr float GATE_FLOOR      = 0.08f;
static constexpr float GATE_ATTACK     = 0.005f;
static constexpr float GATE_RELEASE    = 0.080f;


// ─────────────────────────────────────────────────────────────
//  Frequency band range
// ─────────────────────────────────────────────────────────────

static constexpr float FREQ_LOW  =   50.0f;
static constexpr float FREQ_HIGH = 8000.0f;

static constexpr const char* BAND_NAMES[BAND_COUNT] = {
    "Band 00  ( 50-  78 Hz)",
    "Band 01  ( 78- 122 Hz)",
    "Band 02  (122- 191 Hz)",
    "Band 03  (191- 299 Hz)",
    "Band 04  (299- 468 Hz)",
    "Band 05  (468- 733 Hz)",
    "Band 06  (733-1147 Hz)",
    "Band 07  (1147-1796 Hz)",
    "Band 08  (1796-2812 Hz)",
    "Band 09  (2812-4402 Hz)",
    "Band 10  (4402-6891 Hz)",
    "Band 11  (6891-8000 Hz)",
};


// ─────────────────────────────────────────────────────────────
//  FilterOutput
// ─────────────────────────────────────────────────────────────

struct BandResult {
    float envelope = 0.0f;
    float peak     = 0.0f;
};

struct FilterOutput {
    std::array<BandResult, BAND_COUNT> bands;
    const BandResult& operator[](int b) const { return bands[b]; }
          BandResult& operator[](int b)       { return bands[b]; }
};


// ─────────────────────────────────────────────────────────────
//  BiquadFilter  —  Direct Form II Transposed
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
                b0=(1-cosW0)/2; b1=1-cosW0; b2=(1-cosW0)/2;
                a0=1+alpha; a1=-2*cosW0; a2=1-alpha; break;
            case HIGH_PASS:
                b0=(1+cosW0)/2; b1=-(1+cosW0); b2=(1+cosW0)/2;
                a0=1+alpha; a1=-2*cosW0; a2=1-alpha; break;
            case NOTCH:
                b0=1; b1=-2*cosW0; b2=1;
                a0=1+alpha; a1=-2*cosW0; a2=1-alpha; break;
            case BAND_PASS: default:
                b0=sinW0/2; b1=0; b2=-sinW0/2;
                a0=1+alpha; a1=-2*cosW0; a2=1-alpha; break;
        }
        b0_=b0/a0; b1_=b1/a0; b2_=b2/a0;
        a1_=a1/a0; a2_=a2/a0;
        reset();
    }

    inline float process(float x) {
        const float y = b0_*x + z1_;
        z1_ = b1_*x - a1_*y + z2_;
        z2_ = b2_*x - a2_*y;
        return y;
    }

    void reset() { z1_ = z2_ = 0.0f; }

private:
    float b0_=0,b1_=0,b2_=0,a1_=0,a2_=0,z1_=0,z2_=0;
};


// ─────────────────────────────────────────────────────────────
//  DCBlocker
// ─────────────────────────────────────────────────────────────

class DCBlocker {
public:
    inline float process(float x) {
        const float y = x - xPrev_ + R_ * yPrev_;
        xPrev_ = x; yPrev_ = y;
        return y;
    }
    void reset() { xPrev_ = yPrev_ = 0.0f; }
private:
    static constexpr float R_ = 0.995f;
    float xPrev_ = 0.0f, yPrev_ = 0.0f;
};


// ─────────────────────────────────────────────────────────────
//  MainsNotchBank
// ─────────────────────────────────────────────────────────────

class MainsNotchBank {
public:
    void configure(float mainsHz, float maxHz, float bwHz, float sr) {
        notches_.clear();
        for (float f = mainsHz; f <= maxHz + 0.5f; f += mainsHz) {
            BiquadFilter n;
            n.configure(BiquadFilter::NOTCH, f, sr, f / bwHz);
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
// ─────────────────────────────────────────────────────────────

class TonalBuzzDetector {
public:
    void configure(float sr) {
        attackCoeff_  = 1.0f - std::exp(-1.0f / (sr * GATE_ATTACK));
        releaseCoeff_ = 1.0f - std::exp(-1.0f / (sr * GATE_RELEASE));
        ltRmsCoeff_   = 1.0f - std::exp(-1.0f / (sr * 0.3f));
        reset();
    }
    float analyse(const std::array<float, BAND_COUNT>& envs, float bufRms) {
        ltRms_ += ltRmsCoeff_ * (bufRms - ltRms_);
        const bool isTransient = (ltRms_ > 1e-6f) &&
                                 (bufRms / ltRms_ > TRANSIENT_RATIO);
        float sumLin=0, sumLog=0; int nActive=0;
        for (int b=0; b<BAND_COUNT; ++b) {
            const float e = envs[b];
            if (e > 1e-7f) { sumLin+=e; sumLog+=std::log(e); ++nActive; }
        }
        float sfm = 1.0f;
        if (nActive > 1) {
            const float geo=std::exp(sumLog/nActive), ari=sumLin/nActive;
            sfm = (ari > 1e-7f) ? geo/ari : 1.0f;
        }
        const float target = (!isTransient && sfm < FLATNESS_THRESH)
                                 ? GATE_FLOOR : 1.0f;
        const float coeff  = (target > gateGain_) ? attackCoeff_ : releaseCoeff_;
        gateGain_ += coeff * (target - gateGain_);
        return gateGain_;
    }
    void reset() { gateGain_=1.0f; ltRms_=0.0f; }
private:
    float attackCoeff_=0,releaseCoeff_=0,ltRmsCoeff_=0;
    float gateGain_=1.0f, ltRms_=0.0f;
};


// ─────────────────────────────────────────────────────────────
//  PerBandFilter  —  4th-order Linkwitz-Riley
// ─────────────────────────────────────────────────────────────

struct PerBandFilter {
    BiquadFilter lpA,lpB,hpA,hpB;
    bool  hasHP=true, hasLP=true;
    float envelope=0.0f, attackCoeff=0.0f, releaseCoeff=0.0f;

    void configureEnvelope(float sr) {
        attackCoeff  = 1.0f - std::exp(-1.0f / (sr * ATTACK_TIME));
        releaseCoeff = 1.0f - std::exp(-1.0f / (sr * RELEASE_TIME));
    }
    inline std::pair<float,float> process(float x) {
        float y = x;
        if (hasHP) { y=hpA.process(y); y=hpB.process(y); }
        if (hasLP) { y=lpA.process(y); y=lpB.process(y); }
        const float absY = std::abs(y);
        if (absY > envelope) envelope += attackCoeff  * (absY - envelope);
        else                 envelope += releaseCoeff * (absY - envelope);
        return { y, envelope };
    }
    void reset() {
        lpA.reset(); lpB.reset(); hpA.reset(); hpB.reset();
        envelope = 0.0f;
    }
};


// ─────────────────────────────────────────────────────────────
//  AudioEngine
// ─────────────────────────────────────────────────────────────

class AudioEngine {
public:
    AudioEngine() { initFilters(); }

    void configure(float sampleRate, float inputGain = INPUT_GAIN) {
        sampleRate_ = sampleRate; inputGain_ = inputGain; initFilters();
    }

    FilterOutput process(const std::vector<float>& buf) {
        FilterOutput out;
        std::array<float, BAND_COUNT> peakAcc; peakAcc.fill(0.0f);

        float sumSq = 0.0f;
        for (float s : buf) sumSq += s*s;
        const float bufRms = std::sqrt(
            sumSq / static_cast<float>(std::max(1,(int)buf.size())));

        for (float raw : buf) {
            const float gained  = raw * inputGain_;
            const float clipped = std::tanh(gained);
            const float clean   = dcBlocker_.process(clipped);
            const float notched = mainsNotch_.process(clean);
            for (int b=0; b<BAND_COUNT; ++b) {
                auto [filtered, env] = bands_[b].process(notched);
                (void)env;
                const float a = std::abs(filtered);
                if (a > peakAcc[b]) peakAcc[b] = a;
            }
        }

        std::array<float, BAND_COUNT> envSnap;
        for (int b=0; b<BAND_COUNT; ++b) envSnap[b] = bands_[b].envelope;
        const float gate = buzzDetector_.analyse(envSnap, bufRms);

        for (int b=0; b<BAND_COUNT; ++b) {
            out[b].envelope = std::clamp(bands_[b].envelope * gate, 0.0f, 1.0f);
            out[b].peak     = std::clamp(peakAcc[b]         * gate, 0.0f, 1.0f);
        }
        return out;
    }

    void reset() {
        dcBlocker_.reset(); mainsNotch_.reset(); buzzDetector_.reset();
        for (auto& b : bands_) b.reset();
    }

    float sampleRate() const { return sampleRate_; }

    static const char* bandName(int b) {
        return (b>=0 && b<BAND_COUNT) ? BAND_NAMES[b] : "Unknown";
    }

private:
    void initFilters() {
        const float sr = sampleRate_;
        // Log-spaced crossover edges. To use custom boundaries,
        // replace this loop with a hand-written edges[] array.
        float edges[BAND_COUNT + 1];
        for (int i=0; i<=BAND_COUNT; ++i) {
            const float t = static_cast<float>(i) / BAND_COUNT;
            edges[i] = FREQ_LOW * std::pow(FREQ_HIGH / FREQ_LOW, t);
        }
        for (int b=0; b<BAND_COUNT; ++b) {
            auto& band = bands_[b];
            band.hasHP = (b > 0);
            band.hasLP = (b < BAND_COUNT-1);
            if (band.hasHP) {
                band.hpA.configure(BiquadFilter::HIGH_PASS, edges[b],   sr);
                band.hpB.configure(BiquadFilter::HIGH_PASS, edges[b],   sr);
            }
            if (band.hasLP) {
                band.lpA.configure(BiquadFilter::LOW_PASS,  edges[b+1], sr);
                band.lpB.configure(BiquadFilter::LOW_PASS,  edges[b+1], sr);
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