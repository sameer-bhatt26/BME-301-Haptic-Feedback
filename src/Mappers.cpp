#pragma once

#include "AudioEngine.cpp"
#include <array>
#include <algorithm>
#include <cstring>

// ============================================================
//  Mappers.cpp
//  Translates AudioEngine FilterOutput into a motor command array.
//
//  All mappers share the same interface (IMapper) so main.cpp can
//  hold a pointer and swap modes at runtime with a single line:
//
//      IMapper* activeMapper = &bandMapper;   // or &directMapper
//      activeMapper->map(filterOutput, volume, motorCommands);
//
//  When you add a new mode (e.g. DirectionalMapper), create a
//  class that inherits IMapper and implement map().  No other
//  files need to change.
//
//  Motor layout — 3 rows × 4 columns = 12 motors:
//
//      Index:  0   1   2   3    ← row 0 (low freq end)
//              4   5   6   7    ← row 1 (mid freq)
//              8   9  10  11    ← row 2 (high freq end)
//
//  This layout is used by BandMapper to group motors into rows.
//  DirectMapper uses it to assign one band per motor in order.
// ============================================================


// ─────────────────────────────────────────────────────────────
//  MotorCommand
//  Plain output struct passed from mapper → MotorController.
//  active[m] = true means motor m should run this frame.
// ─────────────────────────────────────────────────────────────

struct MotorCommand {
    bool active[BAND_COUNT] = {};  // one slot per motor (BAND_COUNT == MOTOR_COUNT)
};


// ─────────────────────────────────────────────────────────────
//  Mapping thresholds & limits
//
//  MOTOR_THRESHOLD — minimum envelope level to activate a motor.
//    0.0–1.0.  Raise if motors fire in silence; lower if they
//    don't fire enough on quiet sounds.
//
//  MAX_MOTORS_ACTIVE — hard cap on simultaneous motors.
//    Prevents current spikes on the Pi's 5V rail.
//    With 12 motors at ~100 mA each, 8 simultaneous = 800 mA,
//    which is within a standard Pi power supply's budget.
// ─────────────────────────────────────────────────────────────

static constexpr float MOTOR_THRESHOLD   = 0.05f;
static constexpr int   MAX_MOTORS_ACTIVE = 8;


// ─────────────────────────────────────────────────────────────
//  IMapper  —  base interface
//  All mapping modes implement this.
// ─────────────────────────────────────────────────────────────

class IMapper {
public:
    virtual ~IMapper() = default;

    /**
     * @brief  Convert FilterOutput into motor on/off commands.
     *
     * @param  input    Output from AudioEngine::process()
     * @param  volume   Volume scalar from VolumeAndFunctionButtons,
     *                  range [0, 100].  Scales envelope before
     *                  threshold comparison so the user can feel
     *                  more or less without changing the filter.
     * @param  cmd      Output: which motors to activate this frame.
     */
    virtual void map(const FilterOutput& input,
                     int                 volume,
                     MotorCommand&       cmd) = 0;

    // Human-readable name shown in console status line
    virtual const char* name() const = 0;
};


// ─────────────────────────────────────────────────────────────
//  Shared helper: volume scaling + threshold + MAX_MOTORS cap
//
//  Given an array of per-motor envelope scores, applies volume
//  scaling, threshold gating, and the MAX_MOTORS_ACTIVE cap,
//  then writes the result into cmd.
//
//  scores[m] should already be in [0, 1] before calling this.
// ─────────────────────────────────────────────────────────────

static void applyThresholdAndCap(const float scores[BAND_COUNT],
                                  int         volume,
                                  MotorCommand& cmd)
{
    // volume is 0–100; convert to a 0.0–1.0 multiplier
    const float volScale = static_cast<float>(volume) / 100.0f;

    // Collect motors whose scaled score exceeds the threshold
    struct Candidate { int idx; float score; };
    Candidate candidates[BAND_COUNT];
    int nCandidates = 0;

    for (int m = 0; m < BAND_COUNT; ++m) {
        const float scaled = scores[m] * volScale;
        if (scaled > MOTOR_THRESHOLD)
            candidates[nCandidates++] = { m, scaled };
    }

    // Sort descending by score — strongest signals fire first
    std::sort(candidates, candidates + nCandidates,
              [](const Candidate& a, const Candidate& b) {
                  return a.score > b.score;
              });

    // Apply MAX_MOTORS_ACTIVE cap and write output
    std::memset(cmd.active, 0, sizeof(cmd.active));
    const int limit = std::min(nCandidates, MAX_MOTORS_ACTIVE);
    for (int i = 0; i < limit; ++i)
        cmd.active[candidates[i].idx] = true;
}


// ─────────────────────────────────────────────────────────────
//  DirectMapper
//
//  One band → one motor, 1-to-1 in frequency order.
//  Band 0 (lowest frequency) drives motor 0, band 11 drives motor 11.
//
//  This gives the most granular spatial representation of the
//  audio spectrum across the sleeve.  Because AudioEngine already
//  uses log-spaced bands, the motor spacing naturally mirrors
//  human auditory perception.
//
//  Motor layout result:
//    Row 0:  motors 0–3   → bands 0–3   (50–299 Hz)
//    Row 1:  motors 4–7   → bands 4–7   (299–1796 Hz)
//    Row 2:  motors 8–11  → bands 8–11  (1796–8000 Hz)
// ─────────────────────────────────────────────────────────────

class DirectMapper : public IMapper {
public:

    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        // Scores are simply the per-band envelopes — direct 1-to-1
        float scores[BAND_COUNT];
        for (int m = 0; m < BAND_COUNT; ++m)
            scores[m] = input[m].envelope;

        applyThresholdAndCap(scores, volume, cmd);
    }

    const char* name() const override { return "Direct (1 band : 1 motor)"; }
};


// ─────────────────────────────────────────────────────────────
//  BandMapper
//
//  Groups the 12 bands into MOTOR_ROWS physical rows, then drives
//  all 4 motors in a row equally based on that row's combined
//  band energy.  Gives a simpler "low / mid / high" feel where
//  an entire row buzzes together.
//
//  Row assignment mirrors DirectMapper:
//    Row 0  →  bands 0–3   (low frequencies)
//    Row 1  →  bands 4–7   (mid frequencies)
//    Row 2  →  bands 8–11  (high frequencies)
//
//  To change which bands belong to which row, edit ROW_BAND_START
//  and ROW_BAND_END below.
// ─────────────────────────────────────────────────────────────

class BandMapper : public IMapper {
public:

    // ── Row configuration ────────────────────────────────────
    // ROW_BAND_START[r] = first band index assigned to row r
    // ROW_BAND_END[r]   = last  band index assigned to row r (inclusive)
    // These must cover all BAND_COUNT bands with no overlap or gaps.

    static constexpr int ROW_BAND_START[MOTOR_ROWS] = { 0, 4, 8  };
    static constexpr int ROW_BAND_END  [MOTOR_ROWS] = { 3, 7, 11 };

    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        float scores[BAND_COUNT] = {};

        for (int row = 0; row < MOTOR_ROWS; ++row) {
            // Compute average envelope across all bands in this row
            float rowEnergy = 0.0f;
            const int nBands = ROW_BAND_END[row] - ROW_BAND_START[row] + 1;
            for (int b = ROW_BAND_START[row]; b <= ROW_BAND_END[row]; ++b)
                rowEnergy += input[b].envelope;
            rowEnergy /= static_cast<float>(nBands);

            // Assign the same score to every motor in this row
            const int motorStart = row * MOTORS_PER_ROW;
            const int motorEnd   = motorStart + MOTORS_PER_ROW - 1;
            for (int m = motorStart; m <= motorEnd; ++m)
                scores[m] = rowEnergy;
        }

        applyThresholdAndCap(scores, volume, cmd);
    }

    const char* name() const override { return "Band (row grouping)"; }
};


// ─────────────────────────────────────────────────────────────
//  DirectionalMapper  (stub — requires 2 microphones)
//
//  Takes two FilterOutputs (left mic, right mic) and weights
//  motors on the left/right side of the sleeve by inter-aural
//  level difference (ILD).
//
//  Left motors  = columns 0 and 1 within each row  (indices 0,1,4,5,8,9)
//  Right motors = columns 2 and 3 within each row  (indices 2,3,6,7,10,11)
//
//  Usage from main.cpp:
//      directionalMapper.map(outLeft, outRight, volume, cmd);
//
//  The single-mic IMapper::map() is implemented as a no-op here;
//  call the two-mic overload directly when using this mapper.
// ─────────────────────────────────────────────────────────────

class DirectionalMapper : public IMapper {
public:

    // ── Column assignments ───────────────────────────────────
    // LEFT_COLS  — motor column indices considered "left side"
    // RIGHT_COLS — motor column indices considered "right side"
    // Within each row, motor index = row * MOTORS_PER_ROW + col

    static constexpr int LEFT_COLS [2] = { 0, 1 };
    static constexpr int RIGHT_COLS[2] = { 2, 3 };

    // Two-mic map — call this when you have two AudioEngine instances
    void map(const FilterOutput& leftInput,
             const FilterOutput& rightInput,
             int                 volume,
             MotorCommand&       cmd)
    {
        float scores[BAND_COUNT] = {};

        for (int row = 0; row < MOTOR_ROWS; ++row) {
            // Average envelope for this row from each mic
            float leftEnergy = 0.0f, rightEnergy = 0.0f;
            const int nBands = ROW_BAND_END[row] - ROW_BAND_START[row] + 1;

            for (int b = ROW_BAND_START[row]; b <= ROW_BAND_END[row]; ++b) {
                leftEnergy  += leftInput[b].envelope;
                rightEnergy += rightInput[b].envelope;
            }
            leftEnergy  /= static_cast<float>(nBands);
            rightEnergy /= static_cast<float>(nBands);

            // ILD weight: how much louder is one side vs the other?
            // weight = 0.5 means equal; 1.0 means fully left; 0.0 means fully right
            const float total = leftEnergy + rightEnergy;
            const float leftWeight  = (total > 1e-6f) ? (leftEnergy  / total) : 0.5f;
            const float rightWeight = (total > 1e-6f) ? (rightEnergy / total) : 0.5f;

            // Assign weighted energy to left and right motor columns
            for (int col : LEFT_COLS) {
                const int motorIdx = row * MOTORS_PER_ROW + col;
                scores[motorIdx] = leftEnergy * leftWeight;
            }
            for (int col : RIGHT_COLS) {
                const int motorIdx = row * MOTORS_PER_ROW + col;
                scores[motorIdx] = rightEnergy * rightWeight;
            }
        }

        applyThresholdAndCap(scores, volume, cmd);
    }

    // Single-mic fallback — falls back to band mapping
    void map(const FilterOutput& input, int volume, MotorCommand& cmd) override {
        fallback_.map(input, volume, cmd);
    }

    const char* name() const override { return "Directional (2-mic ILD)"; }

private:
    // Row band ranges (must match BandMapper)
    static constexpr int ROW_BAND_START[MOTOR_ROWS] = { 0, 4, 8  };
    static constexpr int ROW_BAND_END  [MOTOR_ROWS] = { 3, 7, 11 };

    BandMapper fallback_;
};