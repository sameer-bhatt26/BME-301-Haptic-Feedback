#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

// Constants for our haptic logic
#define SAMPLE_RATE 44100
#define MOTOR_COUNT 8
#define OVERDRIVE_MS 15   // Kickstart motor for 15ms
#define AMBIENT_CUTOFF 500.0f
#define EVENT_MIN 2000.0f
#define EVENT_MAX 5000.0f

class AudioEngine {
private:
    // Filter states (Simple IIR Low-Pass)
    float lowPassState = 0;
    float eventBandState = 0;
    
    // Envelope Follower state
    float envelope = 0;
    const float attack = 0.01f;  // Fast attack to catch transients
    const float release = 0.05f; // Slower release for "smooth" feel

    // Overdrive Tracking
    int overdriveCounters[MOTOR_COUNT] = {0};

public:
    AudioEngine() {}

    /**
     * @brief Processes a block of audio and converts it to PWM values
     * @param inputBuffer Raw audio samples from the microphone
     * @param pwmOutputs Array of 8 integers (0-255) to be sent to pigpio
     */
    void processAudio(const std::vector<float>& inputBuffer, int* pwmOutputs) {
        float ambientPower = 0;
        float eventPower = 0;

        for (float sample : inputBuffer) {
            float absSample = std::abs(sample);

            // 1. AMBIENT FILTER (Simple One-Pole Low Pass)
            // Focuses on low-frequency "presence" (0-500Hz)
            lowPassState += 0.1f * (absSample - lowPassState);
            ambientPower += lowPassState;

            // 2. EVENT FILTER & ENVELOPE FOLLOWER (2kHz - 5kHz)
            // We use a simple rectification + smoothing for the envelope
            if (absSample > envelope) 
                envelope += attack * (absSample - envelope);
            else 
                envelope += release * (absSample - envelope);
            
            eventPower = envelope;
        }

        // Normalize powers relative to buffer size
        ambientPower /= inputBuffer.size();

        // 3. MOTOR MAPPING & OVERDRIVE TRICK
        for (int i = 0; i < MOTOR_COUNT; i++) {
            int targetPWM = 0;

            // Simple Logic: 
            // Even motors = Ambient (low rumble)
            // Odd motors = Events (sharp speech/alerts)
            if (i % 2 == 0) {
                targetPWM = (int)(ambientPower * 255 * 5); // Scale up for sensitivity
            } else {
                targetPWM = (int)(eventPower * 255 * 10);
            }

            // Apply Overdrive Trick
            // If the sound is new and strong, blast 100% PWM to spin the ERM fast
            if (targetPWM > 50 && overdriveCounters[i] == 0) {
                pwmOutputs[i] = 255; // Initial kick
                overdriveCounters[i] = OVERDRIVE_MS; 
            } else if (overdriveCounters[i] > 0) {
                pwmOutputs[i] = 255; // Keep kicking for duration
                overdriveCounters[i]--;
            } else {
                // Normal proportional PWM (clamped 0-255)
                pwmOutputs[i] = std::min(255, std::max(0, targetPWM));
            }
        }
    }
};