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
//  Physical motor layout (palm down, top of arm, wrist at top):
//
//      Wrist →  10 |  5 |  4 | 11    row 2  high freq (1796-8000 Hz)
//                8 |  2 |  3 |  9    row 1  mid  freq (299-1796 Hz)
//      Elbow →   6 |  1 |  0 |  7    row 0  low  freq (50-299 Hz)
//
//  Motor index -> PCA9685 channel mapping lives in MotorControl.cpp.
//  Band->row assignments live in BandMapper below.
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
//  MOTOR_THRESHOLD     — minimum scaled envelope to activate a motor.
//                        Raise if motors hum in silence; lower if
//                        they don't respond to quiet sounds.
//
//  MAX_CURRENT_MA      — hard current budget in milliamps.
//                        Motors are admitted in descending energy
//                        order until adding the next one would
//                        exceed this budget. Prevents regulator
//                        (LD1117) from being overloaded.
//                        LD1117 max: ~800 mA. Pi + PCA9685 draw
//                        ~150-200 mA at rest, so 500 mA leaves
//                        comfortable headroom.
//
//  MOTOR_CURRENT_MA    — current drawn by one motor at 100% duty.
//                        At partial duty, actual draw scales
//                        proportionally (PWM average current).
//                        Measure your specific ERM; 50 mA is typical.
//
//  OUTPUT_SMOOTHING    — low-pass coefficient on PWM output.
//                        Prevents abrupt motor speed steps.
//                        0.3 gives ~33 ms ramp at 30 Hz update rate.
//                        Range 0.0 (frozen) to 1.0 (no smoothing).
// ─────────────────────────────────────────────────────────────

static constexpr float MOTOR_THRESHOLD  = 0.05f;
static constexpr float MAX_CURRENT_MA   = 500.0f;  // safe ceiling for LD1117
static constexpr float MOTOR_CURRENT_MA = 50.0f;   // per motor at 100% duty
static constexpr float OUTPUT_SMOOTHING = 0.6f;


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
//  Shared helper — volume scale, threshold gate, current budget,
//  deterministic sort, and output smoothing.
//  Call this at the end of every mapper's map() implementation.
//
//  Current budget logic:
//    Motors are sorted strongest-first. They are admitted one by
//    one until adding the next motor's estimated draw (intensity
//    * MOTOR_CURRENT_MA) would exceed MAX_CURRENT_MA. This means:
//      - At low amplitudes more motors can fire simultaneously.
//      - At full amplitude fewer motors fire, protecting the LDO.
//      - Ties in score are broken by motor index (deterministic),
//        so the same motors always win — no random flickering.
// ─────────────────────────────────────────────────────────────

static void applyVolumeGateCurrentAndSmooth(const float   scores[BAND_COUNT],
                                             int           volume,
                                             float         smoothed[BAND_COUNT],
                                             MotorCommand& cmd)
{
    const float volScale = 0.2f + (static_cast<float>(volume) / 100.0f) * 0.8f;

    // Collect motors above threshold
    struct Candidate { int idx; float score; };
    Candidate candidates[BAND_COUNT];
    int nCandidates = 0;

    for (int m = 0; m < BAND_COUNT; ++m) {
        const float scaled = scores[m] * volScale;
        if (scaled > MOTOR_THRESHOLD)
            candidates[nCandidates++] = { m, scaled };
    }

    // Sort descending by score — break ties by motor index so the
    // same motors always win (no frame-to-frame flickering in BandMapper
    // where all motors in a row share the same score).
    std::sort(candidates, candidates + nCandidates,
              [](const Candidate& a, const Candidate& b) {
                  return a.score != b.score ? a.score > b.score
                                           : a.idx   < b.idx;
              });

    // Admit motors in priority order until current budget is exhausted.
    // Actual draw per motor = intensity * MOTOR_CURRENT_MA (PWM average).
    float target[BAND_COUNT] = {};
    float budgetUsed = 0.0f;

    for (int i = 0; i < nCandidates; ++i) {
        const float motorCurrent = candidates[i].score * MOTOR_CURRENT_MA;
        if (budgetUsed + motorCurrent > MAX_CURRENT_MA) break;
        target[candidates[i].idx] = candidates[i].score;
        budgetUsed += motorCurrent;
    }

    // Smooth output to prevent abrupt PWM steps
    for (int m = 0; m < BAND_COUNT; ++m) {
        smoothed[m] += OUTPUT_SMOOTHING * (target[m] - smoothed[m]);
        cmd.intensity[m] = std::clamp(smoothed[m], 0.0f, 1.0f);
    }
}


// ─────────────────────────────────────────────────────────────
//  BandMapper  (mode 1)
//
//  All 4 motors in a physical row vibrate at equal intensity
//  based on the average envelope across that row's frequency
//  bands. Gives a simple "low / mid / high" feel.
//
//  Physical row -> motor index mapping (from arm diagram):
//    Row 0  elbow/low  freq  ->  motors 6, 1, 0, 7
//    Row 1  mid        freq  ->  motors 8, 2, 3, 9
//    Row 2  wrist/high freq  ->  motors 10, 5, 4, 11
//
//  Band -> row assignments:
//    Row 0  ->  bands 0-3   (50-299 Hz)
//    Row 1  ->  bands 4-7   (299-1796 Hz)
//    Row 2  ->  bands 8-11  (1796-8000 Hz)
//
//  To change which bands map to which row, edit ROW_BANDS only.
// ─────────────────────────────────────────────────────────────

class BandMapper : public IMapper {
public:
    // Physical row -> motor indices (from arm diagram)
    // Row 0 = elbow (low), Row 1 = mid, Row 2 = wrist (high)
    static constexpr int ROW_MOTORS[MOTOR_ROWS][MOTORS_PER_ROW] = {
        { 6, 1, 0, 7  },   // row 0 — low  freq — elbow
        { 8, 2, 3, 9  },   // row 1 — mid  freq
        { 10, 5, 4, 11 },  // row 2 — high freq — wrist
    };

    // Band range per row (inclusive)
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

            // All 4 motors in this physical row get the same intensity
            for (int col = 0; col < MOTORS_PER_ROW; ++col)
                scores[ROW_MOTORS[row][col]] = rowEnergy;
        }

        applyVolumeGateCurrentAndSmooth(scores, volume, smoothed_, cmd);
    }

    const char* name() const override { return "Mode 1: Band (rows)"; }

private:
    float smoothed_[BAND_COUNT] = {};
};


// ─────────────────────────────────────────────────────────────
//  DirectMapper  (mode 2)
//
//  One band drives one motor in frequency order.
//  Motor intensity is proportional to that band's envelope.
//  Gives the most granular spatial representation of the audio
//  spectrum — elbow = lower frequencies, wrist = higher.
//
//  Band -> motor index is 1:1 in index order:
//    Bands  0-3  -> motors  0-3   (low freq)
//    Bands  4-7  -> motors  4-7   (mid freq)
//    Bands  8-11 -> motors  8-11  (high freq)
// ─────────────────────────────────────────────────────────────

class DirectMapper : public IMapper {
public:
    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        float scores[BAND_COUNT];
        for (int m = 0; m < BAND_COUNT; ++m)
            scores[m] = input[m].envelope;
        applyVolumeGateCurrentAndSmooth(scores, volume, smoothed_, cmd);
    }

    const char* name() const override { return "Mode 2: Direct (1:1)"; }

private:
    float smoothed_[BAND_COUNT] = {};
};