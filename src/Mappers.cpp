#pragma once

#include "AudioEngine.cpp"
#include <array>
#include <algorithm>
#include <cstring>

// ============================================================
//  Mappers.cpp
//  Translates FilterOutput into per-motor PWM intensity values.
//
//  Now that the PCA9685 supports PWM, MotorCommand carries a
//  float intensity per motor (0.0 = off, 1.0 = full vibration)
//  instead of a simple bool.  This lets mappers express how
//  strongly each motor should vibrate, not just whether it fires.
//
//  All mappers share the IMapper interface so main.cpp can swap
//  modes at runtime by changing one pointer:
//
//      IMapper* active = &directMapper;
//      active->map(filterOutput, volume, cmd);
//
//  Motor layout — 3 rows x 4 columns = 12 motors:
//
//      Index:  0   1   2   3    <- row 0  low freq  (50-299 Hz)
//              4   5   6   7    <- row 1  mid freq  (299-1796 Hz)
//              8   9  10  11    <- row 2  high freq (1796-8000 Hz)
// ============================================================


// ─────────────────────────────────────────────────────────────
//  MotorCommand  —  output from every mapper
//
//  intensity[m] is in [0.0, 1.0]:
//    0.0  = motor fully off
//    1.0  = motor at full vibration
//    0.5  = motor at half intensity
//
//  MotorControl.cpp converts these to 12-bit PCA9685 values.
// ─────────────────────────────────────────────────────────────

struct MotorCommand {
    float intensity[BAND_COUNT] = {};
};


// ─────────────────────────────────────────────────────────────
//  Mapping constants
//
//  MOTOR_THRESHOLD — minimum scaled envelope to activate a motor.
//    Below this, intensity is forced to 0 (motor stays still).
//    Prevents micro-vibrations from background noise.
//    Range 0.0-1.0.  Tune upward if motors hum in silence.
//
//  MAX_MOTORS_ACTIVE — hard cap on simultaneously active motors.
//    Limits peak current draw from the battery rail.
//    With 12 x ERM at ~100 mA each, 8 simultaneous = 800 mA.
//
//  OUTPUT_SMOOTHING_COEFF — low-pass on the PWM output intensity.
//    Prevents abrupt step changes in motor speed (audible clicking,
//    mechanical stress).  Higher = faster response, more stepping.
//    Range 0.0-1.0.  ~0.3 gives a ~33ms ramp at 30 Hz update rate.
// ─────────────────────────────────────────────────────────────

static constexpr float MOTOR_THRESHOLD      = 0.05f;
static constexpr int   MAX_MOTORS_ACTIVE    = 8;
static constexpr float OUTPUT_SMOOTHING     = 0.3f;  // per 33 ms frame


// ─────────────────────────────────────────────────────────────
//  IMapper  —  base interface
// ─────────────────────────────────────────────────────────────

class IMapper {
public:
    virtual ~IMapper() = default;

    /**
     * @brief  Convert FilterOutput into per-motor PWM intensities.
     *
     * @param  input    FilterOutput from AudioEngine::process()
     * @param  volume   User volume, 0-100.  Scales all intensities
     *                  proportionally before threshold comparison.
     * @param  cmd      Output: per-motor intensity in [0.0, 1.0].
     */
    virtual void map(const FilterOutput& input,
                     int                 volume,
                     MotorCommand&       cmd) = 0;

    virtual const char* name() const = 0;
};


// ─────────────────────────────────────────────────────────────
//  Shared helpers
// ─────────────────────────────────────────────────────────────

// Applies volume scaling, threshold gate, MAX_MOTORS_ACTIVE cap,
// and output smoothing to a raw scores[] array.
// scores[m] should be in [0, 1] before calling.
// prevIntensity[m] is updated in-place for the smoothing filter.
static void applyVolumeGateCapAndSmooth(const float  scores[BAND_COUNT],
                                         int          volume,
                                         float        prevIntensity[BAND_COUNT],
                                         MotorCommand& cmd)
{
    const float volScale = static_cast<float>(volume) / 100.0f;

    // Collect motors above threshold, sorted by energy
    struct Candidate { int idx; float score; };
    Candidate candidates[BAND_COUNT];
    int nCandidates = 0;

    for (int m = 0; m < BAND_COUNT; ++m) {
        const float scaled = scores[m] * volScale;
        if (scaled > MOTOR_THRESHOLD)
            candidates[nCandidates++] = { m, scaled };
    }

    // Sort descending — strongest signal gets priority under the cap
    std::sort(candidates, candidates + nCandidates,
              [](const Candidate& a, const Candidate& b) {
                  return a.score > b.score;
              });

    // Build target intensity array — only top MAX_MOTORS_ACTIVE fire
    float target[BAND_COUNT] = {};
    const int limit = std::min(nCandidates, MAX_MOTORS_ACTIVE);
    for (int i = 0; i < limit; ++i)
        target[candidates[i].idx] = candidates[i].score;

    // Smooth output — prevents abrupt PWM steps
    for (int m = 0; m < BAND_COUNT; ++m) {
        prevIntensity[m] += OUTPUT_SMOOTHING * (target[m] - prevIntensity[m]);
        cmd.intensity[m] = std::clamp(prevIntensity[m], 0.0f, 1.0f);
    }
}


// ─────────────────────────────────────────────────────────────
//  DirectMapper
//
//  One band -> one motor, 1-to-1 in frequency order.
//  Band 0 (lowest) drives motor 0; band 11 (highest) drives motor 11.
//  Motor intensity is proportional to band envelope energy.
//
//  Motor layout result:
//    Row 0:  motors 0-3   <- bands 0-3   (50-299 Hz)
//    Row 1:  motors 4-7   <- bands 4-7   (299-1796 Hz)
//    Row 2:  motors 8-11  <- bands 8-11  (1796-8000 Hz)
// ─────────────────────────────────────────────────────────────

class DirectMapper : public IMapper {
public:
    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        float scores[BAND_COUNT];
        for (int m = 0; m < BAND_COUNT; ++m)
            scores[m] = input[m].envelope;
        applyVolumeGateCapAndSmooth(scores, volume, smoothed_, cmd);
    }
    const char* name() const override { return "Direct (1 band : 1 motor)"; }

private:
    float smoothed_[BAND_COUNT] = {};
};


// ─────────────────────────────────────────────────────────────
//  BandMapper
//
//  Groups all 4 motors in a row and drives them at equal intensity
//  based on the average envelope across that row's bands.
//  Gives a simpler "low / mid / high" sensation — entire rows
//  buzz together rather than individual motors.
//
//  Row band assignments:
//    Row 0  ->  bands 0-3   (low)
//    Row 1  ->  bands 4-7   (mid)
//    Row 2  ->  bands 8-11  (high)
//
//  To change which bands belong to which row, edit ROW_BAND_START
//  and ROW_BAND_END.  They must cover all BAND_COUNT bands with
//  no overlap or gaps.
// ─────────────────────────────────────────────────────────────

class BandMapper : public IMapper {
public:
    // First and last band index (inclusive) for each motor row
    static constexpr int ROW_BAND_START[MOTOR_ROWS] = { 0, 4, 8  };
    static constexpr int ROW_BAND_END  [MOTOR_ROWS] = { 3, 7, 11 };

    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        float scores[BAND_COUNT] = {};

        for (int row = 0; row < MOTOR_ROWS; ++row) {
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

    const char* name() const override { return "Band (row grouping)"; }

private:
    float smoothed_[BAND_COUNT] = {};
};


// ─────────────────────────────────────────────────────────────
//  DirectionalMapper  (two-microphone mode)
//
//  Requires two AudioEngine instances — one per mic.
//  Compares per-row energy between left and right mics to compute
//  an inter-aural level difference (ILD) weight.
//
//  Left motors  = columns 0, 1 within each row
//  Right motors = columns 2, 3 within each row
//
//  If the left mic is louder for a row, left-side motors in that
//  row vibrate proportionally harder, and vice versa.
//
//  To change left/right column assignments, edit LEFT_COLS and
//  RIGHT_COLS below.
//
//  Single-mic fallback: falls back to BandMapper so this mapper
//  can be selected even before a second mic is wired up.
// ─────────────────────────────────────────────────────────────

class DirectionalMapper : public IMapper {
public:
    // Column indices (within each row) assigned to each side
    static constexpr int LEFT_COLS [2] = { 0, 1 };
    static constexpr int RIGHT_COLS[2] = { 2, 3 };

    // Row band ranges — must match BandMapper
    static constexpr int ROW_BAND_START[MOTOR_ROWS] = { 0, 4, 8  };
    static constexpr int ROW_BAND_END  [MOTOR_ROWS] = { 3, 7, 11 };

    // Two-mic version — call this when you have two AudioEngine instances
    void map(const FilterOutput& leftInput,
             const FilterOutput& rightInput,
             int                 volume,
             MotorCommand&       cmd)
    {
        float scores[BAND_COUNT] = {};

        for (int row = 0; row < MOTOR_ROWS; ++row) {
            float leftEnergy = 0.0f, rightEnergy = 0.0f;
            const int nBands = ROW_BAND_END[row] - ROW_BAND_START[row] + 1;

            for (int b = ROW_BAND_START[row]; b <= ROW_BAND_END[row]; ++b) {
                leftEnergy  += leftInput[b].envelope;
                rightEnergy += rightInput[b].envelope;
            }
            leftEnergy  /= static_cast<float>(nBands);
            rightEnergy /= static_cast<float>(nBands);

            const float total       = leftEnergy + rightEnergy;
            const float leftWeight  = (total > 1e-6f) ? leftEnergy  / total : 0.5f;
            const float rightWeight = (total > 1e-6f) ? rightEnergy / total : 0.5f;

            for (int col : LEFT_COLS) {
                const int m = row * MOTORS_PER_ROW + col;
                scores[m] = leftEnergy * leftWeight;
            }
            for (int col : RIGHT_COLS) {
                const int m = row * MOTORS_PER_ROW + col;
                scores[m] = rightEnergy * rightWeight;
            }
        }

        applyVolumeGateCapAndSmooth(scores, volume, smoothed_, cmd);
    }

    // Single-mic fallback — delegates to BandMapper
    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        fallback_.map(input, volume, cmd);
    }

    const char* name() const override { return "Directional (2-mic ILD)"; }

private:
    BandMapper fallback_;
    float      smoothed_[BAND_COUNT] = {};
};