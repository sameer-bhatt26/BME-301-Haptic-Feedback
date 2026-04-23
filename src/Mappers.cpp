#pragma once

#include "AudioEngine.cpp"
#include <array>
#include <algorithm>
#include <cstring>

// ============================================================
//  Mappers.cpp
//  Translates FilterOutput into per-motor PWM intensity values.
//
//  Two mapping modes are implemented:
//    1. BandMapper   — entire rows buzz together by frequency group
//    2. DirectMapper — one band per motor, proportional intensity
//
//  Both share the IMapper interface so main.cpp swaps modes by
//  changing one pointer — no other code needs to change.
//
//  Motor layout (3 rows x 4 columns = 12 motors):
//
//      Index:  0   1   2   3    <- row 0  low  freq (50-299 Hz)
//              4   5   6   7    <- row 1  mid  freq (299-1796 Hz)
//              8   9  10  11    <- row 2  high freq (1796-8000 Hz)
// ============================================================


// ─────────────────────────────────────────────────────────────
//  MotorCommand  —  output from every mapper
//  intensity[m] in [0.0, 1.0]:
//    0.0 = off,  1.0 = full vibration,  0.5 = half intensity
//  MotorControl.cpp converts these to 12-bit PCA9685 values.
// ─────────────────────────────────────────────────────────────

struct MotorCommand {
    float intensity[BAND_COUNT] = {};
};


// ─────────────────────────────────────────────────────────────
//  Mapping constants
//
//  MOTOR_THRESHOLD   — minimum scaled envelope to activate a motor.
//                      Raise if motors hum in silence; lower if
//                      they don't respond to quiet sounds.
//
//  MAX_MOTORS_ACTIVE — hard cap on simultaneous motors.
//                      Limits peak current from the battery rail.
//
//  OUTPUT_SMOOTHING  — low-pass coefficient on PWM output.
//                      Prevents abrupt motor speed steps.
//                      0.3 gives ~33 ms ramp at 30 Hz update rate.
//                      Range 0.0 (frozen) to 1.0 (no smoothing).
// ─────────────────────────────────────────────────────────────

static constexpr float MOTOR_THRESHOLD   = 0.05f;
static constexpr int   MAX_MOTORS_ACTIVE = 8;
static constexpr float OUTPUT_SMOOTHING  = 0.3f;


// ─────────────────────────────────────────────────────────────
//  IMapper  —  base interface for all mapping modes
// ─────────────────────────────────────────────────────────────

class IMapper {
public:
    virtual ~IMapper() = default;

    /**
     * @param input   FilterOutput from AudioEngine::process()
     * @param volume  User volume 0-100; scales all intensities.
     * @param cmd     Output: per-motor intensity in [0.0, 1.0].
     */
    virtual void map(const FilterOutput& input,
                     int                 volume,
                     MotorCommand&       cmd) = 0;

    virtual const char* name() const = 0;
};


// ─────────────────────────────────────────────────────────────
//  Shared helper — volume scale, threshold gate, cap, smoothing
//  Call this at the end of every mapper's map() implementation.
// ─────────────────────────────────────────────────────────────

static void applyVolumeGateCapAndSmooth(const float   scores[BAND_COUNT],
                                         int           volume,
                                         float         smoothed[BAND_COUNT],
                                         MotorCommand& cmd)
{
    const float volScale = static_cast<float>(volume) / 100.0f;

    // Collect motors above threshold, ranked by energy
    struct Candidate { int idx; float score; };
    Candidate candidates[BAND_COUNT];
    int nCandidates = 0;

    for (int m = 0; m < BAND_COUNT; ++m) {
        const float scaled = scores[m] * volScale;
        if (scaled > MOTOR_THRESHOLD)
            candidates[nCandidates++] = { m, scaled };
    }

    // Sort descending — strongest signals get priority under the cap
    std::sort(candidates, candidates + nCandidates,
              [](const Candidate& a, const Candidate& b) {
                  return a.score > b.score;
              });

    // Build target intensities — only top MAX_MOTORS_ACTIVE fire
    float target[BAND_COUNT] = {};
    const int limit = std::min(nCandidates, MAX_MOTORS_ACTIVE);
    for (int i = 0; i < limit; ++i)
        target[candidates[i].idx] = candidates[i].score;

    // Smooth output to prevent abrupt PWM steps
    for (int m = 0; m < BAND_COUNT; ++m) {
        smoothed[m] += OUTPUT_SMOOTHING * (target[m] - smoothed[m]);
        cmd.intensity[m] = std::clamp(smoothed[m], 0.0f, 1.0f);
    }
}


// ─────────────────────────────────────────────────────────────
//  BandMapper  (function 1)
//
//  All 4 motors in a row vibrate at equal intensity based on
//  the average envelope across that row's frequency bands.
//  Gives a simple "low / mid / high" feel — whole rows buzz
//  together rather than individual motors.
//
//  Row assignments:
//    Row 0  ->  bands 0-3   (50-299 Hz)    motors 0-3
//    Row 1  ->  bands 4-7   (299-1796 Hz)  motors 4-7
//    Row 2  ->  bands 8-11  (1796-8000 Hz) motors 8-11
//
//  To change which bands belong to which row, edit
//  ROW_BAND_START and ROW_BAND_END below only.
// ─────────────────────────────────────────────────────────────

class BandMapper : public IMapper {
public:
    // ── Row band assignments ─────────────────────────────────
    // ROW_BAND_START[r] = first band index for row r (inclusive)
    // ROW_BAND_END[r]   = last  band index for row r (inclusive)
    // Must cover all BAND_COUNT bands, no gaps, no overlaps.
    static constexpr int ROW_BAND_START[MOTOR_ROWS] = { 0, 4, 8  };
    static constexpr int ROW_BAND_END  [MOTOR_ROWS] = { 3, 7, 11 };

    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        float scores[BAND_COUNT] = {};

        for (int row = 0; row < MOTOR_ROWS; ++row) {
            // Average envelope across all bands in this row
            float rowEnergy = 0.0f;
            const int nBands = ROW_BAND_END[row] - ROW_BAND_START[row] + 1;
            for (int b = ROW_BAND_START[row]; b <= ROW_BAND_END[row]; ++b)
                rowEnergy += input[b].envelope;
            rowEnergy /= static_cast<float>(nBands);

            // All 4 motors in this row get the same intensity
            const int motorStart = row * MOTORS_PER_ROW;
            for (int m = motorStart; m < motorStart + MOTORS_PER_ROW; ++m)
                scores[m] = rowEnergy;
        }

        applyVolumeGateCapAndSmooth(scores, volume, smoothed_, cmd);
    }

    const char* name() const override { return "Mode 1: Band (rows)"; }

private:
    float smoothed_[BAND_COUNT] = {};
};


// ─────────────────────────────────────────────────────────────
//  DirectMapper  (function 2)
//
//  One band drives one motor in frequency order.
//  Motor intensity is proportional to that band's envelope.
//  Gives the most granular spatial representation of the audio
//  spectrum — lower sleeve = lower frequencies, higher = higher.
//
//  Result:
//    Row 0: motors 0-3  <- bands 0-3  (50-299 Hz)
//    Row 1: motors 4-7  <- bands 4-7  (299-1796 Hz)
//    Row 2: motors 8-11 <- bands 8-11 (1796-8000 Hz)
// ─────────────────────────────────────────────────────────────

class DirectMapper : public IMapper {
public:
    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        float scores[BAND_COUNT];
        for (int m = 0; m < BAND_COUNT; ++m)
            scores[m] = input[m].envelope;
        applyVolumeGateCapAndSmooth(scores, volume, smoothed_, cmd);
    }

    const char* name() const override { return "Mode 2: Direct (1:1)"; }

private:
    float smoothed_[BAND_COUNT] = {};
};