#pragma once

// ============================================================
//  MotorControl.cpp
//  PCA9685 PWM driver for ERM haptic motors via I2C.
//
//  Hardware:
//    Raspberry Pi Zero 2 W  ->  PCA9685  ->  ULN2803A  ->  ERM motors
//    Communication: I2C (SDA/SCL, default bus /dev/i2c-1)
//    The ULN2803A is an inverting Darlington driver, but the
//    PCA9685 drives its inputs so active-high logic applies:
//      high PWM duty cycle = transistor on = motor spinning.
//
//  Install:
//    sudo apt install libi2c-dev
//    sudo raspi-config -> Interface Options -> I2C -> Enable
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -li2c -lportaudio -lm
//
//  PCA9685 I2C address:
//    Default is 0x40.  Check your board's A0-A5 solder jumpers
//    if you get an "open failed" error and change PCA9685_ADDR.
// ============================================================

#include "Mappers.cpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <array>
#include <algorithm>
#include <chrono>
#include <thread>


// ─────────────────────────────────────────────────────────────
//  I2C / PCA9685 hardware config
// ─────────────────────────────────────────────────────────────

// I2C bus device — /dev/i2c-1 on all modern Pi models
static constexpr const char* I2C_BUS      = "/dev/i2c-1";

// PCA9685 7-bit I2C address.  Default is 0x40.
// If you soldered any A0-A5 jumpers on the board, adjust here.
static constexpr uint8_t PCA9685_ADDR     = 0x40;

// PWM frequency for the PCA9685 in Hz.
// ERM motors respond well to 1000 Hz.  Valid range: ~24-1526 Hz.
// Lower frequency = more torque at low duty cycles but more audible whine.
// Higher frequency = quieter but less torque.
static constexpr float   PWM_FREQ_HZ      = 1000.0f;

// PCA9685 internal oscillator frequency (datasheet: 25 MHz)
static constexpr float   PCA9685_OSC_HZ   = 25000000.0f;

// PCA9685 has 12-bit resolution: 0 = fully off, 4095 = fully on
static constexpr int     PWM_RESOLUTION   = 4096;


// ─────────────────────────────────────────────────────────────
//  Motor -> PCA9685 channel mapping
//
//  Index in this array = motor index (0-11).
//  Value = PCA9685 output channel number (0-15).
//
//  Motor layout on the sleeve:
//    Motor 0   Motor 1   Motor 2   Motor 3   <- row 0 (low freq)
//    Motor 4   Motor 5   Motor 6   Motor 7   <- row 1 (mid freq)
//    Motor 8   Motor 9   Motor 10  Motor 11  <- row 2 (high freq)
//
//  Update these after any PCB revision — the rest of the code
//  uses only this table to determine which channel to write.
// ─────────────────────────────────────────────────────────────

static constexpr std::array<int, BAND_COUNT> MOTOR_CHANNELS = {
     0,   // Motor  0  — row 0, col 0
     1,   // Motor  1  — row 0, col 1
     2,   // Motor  2  — row 0, col 2
     3,   // Motor  3  — row 0, col 3
     4,   // Motor  4  — row 1, col 0
     5,   // Motor  5  — row 1, col 1
     6,   // Motor  6  — row 1, col 2
     7,   // Motor  7  — row 1, col 3
     8,   // Motor  8  — row 2, col 0
     9,   // Motor  9  — row 2, col 1
    10,   // Motor 10  — row 2, col 2
    11,   // Motor 11  — row 2, col 3
};

static constexpr int MOTOR_COUNT = static_cast<int>(MOTOR_CHANNELS.size());
static_assert(MOTOR_COUNT == BAND_COUNT,
    "MOTOR_CHANNELS must have one entry per band/motor.");


// ─────────────────────────────────────────────────────────────
//  ERM overdrive (kickstart) parameters
//
//  ERMs have rotational inertia — at low PWM duty cycles they
//  may not start spinning at all (the motor hums but doesn't turn).
//  The overdrive trick blasts 100% PWM for a short burst when a
//  motor transitions from off to on, then drops to the target level.
//
//  OVERDRIVE_DUTY    — PWM value during kickstart (4095 = 100%)
//  OVERDRIVE_FRAMES  — how many main-loop frames to hold the kick
//                      At 30 Hz: 1 frame ~= 33 ms.  Default 1 frame
//                      (~33 ms) is enough for most ERMs.  Increase
//                      to 2 if motors still fail to start reliably.
//
//  MIN_SPIN_DUTY     — below this 12-bit value the motor will not
//                      spin even with a kickstart.  Intensities that
//                      map below this are rounded up to MIN_SPIN_DUTY
//                      when a motor is active, or cut to 0 if the
//                      mapper decided the motor should be off.
//                      Tune this per your specific ERM model.
// ─────────────────────────────────────────────────────────────

static constexpr int OVERDRIVE_DUTY   = PWM_RESOLUTION - 1;  // 100%
static constexpr int OVERDRIVE_FRAMES = 1;                    // ~33 ms
static constexpr int MIN_SPIN_DUTY    = 512;   // ~12.5% — minimum to sustain spin


// ─────────────────────────────────────────────────────────────
//  PCA9685 register map  (from datasheet)
// ─────────────────────────────────────────────────────────────

static constexpr uint8_t PCA9685_MODE1      = 0x00;
static constexpr uint8_t PCA9685_PRESCALE   = 0xFE;
static constexpr uint8_t PCA9685_LED0_ON_L  = 0x06;  // base register for channel 0
// Each channel occupies 4 bytes: ON_L, ON_H, OFF_L, OFF_H
static constexpr uint8_t PCA9685_BYTES_PER_CHANNEL = 4;

static constexpr uint8_t MODE1_SLEEP  = 0x10;
static constexpr uint8_t MODE1_ALLCALL= 0x01;
static constexpr uint8_t MODE1_AI     = 0x20;  // auto-increment register address


// ─────────────────────────────────────────────────────────────
//  MotorController
// ─────────────────────────────────────────────────────────────

class MotorController {
public:

    // ── Init / shutdown ──────────────────────────────────────

    bool init() {
        // Open I2C bus
        i2cFd_ = open(I2C_BUS, O_RDWR);
        if (i2cFd_ < 0) {
            std::cerr << "[MotorControl] Cannot open " << I2C_BUS << "\n"
                      << "  Enable I2C: sudo raspi-config -> Interfaces -> I2C\n"
                      << "  Check bus:  ls /dev/i2c*\n";
            return false;
        }

        // Set the slave address
        if (ioctl(i2cFd_, I2C_SLAVE, PCA9685_ADDR) < 0) {
            std::cerr << "[MotorControl] Cannot address PCA9685 at 0x"
                      << std::hex << (int)PCA9685_ADDR << std::dec << "\n"
                      << "  Scan bus: i2cdetect -y 1\n";
            close(i2cFd_); i2cFd_ = -1;
            return false;
        }

        // Initialise PCA9685
        if (!initPCA9685()) {
            close(i2cFd_); i2cFd_ = -1;
            return false;
        }

        initialised_ = true;
        std::cout << "[MotorControl] PCA9685 ready at 0x"
                  << std::hex << (int)PCA9685_ADDR << std::dec
                  << " on " << I2C_BUS
                  << "  PWM freq: " << PWM_FREQ_HZ << " Hz\n";
        return true;
    }

    void shutdown() {
        if (!initialised_) return;
        allOff();
        // Put PCA9685 to sleep to stop PWM outputs
        writeReg(PCA9685_MODE1, MODE1_SLEEP);
        close(i2cFd_);
        i2cFd_       = -1;
        initialised_ = false;
        std::cout << "[MotorControl] PCA9685 shutdown.\n";
    }

    ~MotorController() { shutdown(); }


    // ── Main update — called once per main-loop frame ────────

    /**
     * @brief  Write per-motor PWM intensities from a MotorCommand.
     *
     *  Applies three layers on top of the raw intensity:
     *
     *  1. Overdrive kickstart — when a motor transitions from off
     *     to on, blasts OVERDRIVE_DUTY for OVERDRIVE_FRAMES then
     *     drops to the proportional target.
     *
     *  2. Minimum spin floor — clamps active motors to at least
     *     MIN_SPIN_DUTY so the ERM actually starts rotating.
     *
     *  3. Change detection — only writes channels that changed
     *     since last frame (reduces I2C bus traffic).
     *
     * @param cmd   MotorCommand from IMapper::map()
     */
    void update(const MotorCommand& cmd) {
        if (!initialised_) return;

        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const float intensity = cmd.intensity[m];
            const bool  wantOn    = (intensity > 0.001f);

            int targetDuty = 0;

            if (wantOn) {
                // Convert normalised intensity to 12-bit duty cycle
                targetDuty = static_cast<int>(intensity * (PWM_RESOLUTION - 1));

                // Overdrive kickstart on motor-start transition
                if (prevDuty_[m] == 0) {
                    // Motor was off — start overdrive countdown
                    overdriveCounter_[m] = OVERDRIVE_FRAMES;
                }

                if (overdriveCounter_[m] > 0) {
                    targetDuty = OVERDRIVE_DUTY;
                    --overdriveCounter_[m];
                } else {
                    // Ensure motor stays above minimum spin threshold
                    targetDuty = std::max(targetDuty, MIN_SPIN_DUTY);
                }
            } else {
                overdriveCounter_[m] = 0;
            }

            // Only write if the value changed — saves I2C bandwidth
            if (targetDuty != prevDuty_[m]) {
                setChannelDuty(MOTOR_CHANNELS[m], targetDuty);
                prevDuty_[m] = targetDuty;
            }
        }
    }


    // ── Utility ──────────────────────────────────────────────

    void allOff() {
        if (!initialised_) return;
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            setChannelDuty(MOTOR_CHANNELS[m], 0);
            prevDuty_[m]        = 0;
            overdriveCounter_[m]= 0;
        }
    }

    // Prints intensity bars for each motor in the console status
    void printState(const MotorCommand& cmd) const {
        std::cout << "[Motors] ";
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const float v = cmd.intensity[m];
            if      (v > 0.75f) std::cout << "█";
            else if (v > 0.50f) std::cout << "▓";
            else if (v > 0.25f) std::cout << "▒";
            else if (v > 0.01f) std::cout << "░";
            else                std::cout << " ";
        }
        std::cout << "  (█▓▒░ = intensity level)\n";
    }


private:

    // ── PCA9685 initialisation ───────────────────────────────

    bool initPCA9685() {
        // 1. Reset to known state (sleep mode, all outputs off)
        if (!writeReg(PCA9685_MODE1, MODE1_SLEEP)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // 2. Set PWM frequency via prescale register
        //    prescale = round(osc / (4096 * freq)) - 1
        const uint8_t prescale = static_cast<uint8_t>(
            std::round(PCA9685_OSC_HZ / (PWM_RESOLUTION * PWM_FREQ_HZ)) - 1);

        // Must be in sleep mode to write prescale
        if (!writeReg(PCA9685_PRESCALE, prescale)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // 3. Wake up with auto-increment enabled
        if (!writeReg(PCA9685_MODE1, MODE1_AI | MODE1_ALLCALL)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // 4. All channels off
        allChannelsOff();

        return true;
    }

    // ── PCA9685 channel write ────────────────────────────────

    // Sets a single channel to a 12-bit duty cycle value (0-4095).
    // Uses the LED_ON/LED_OFF register pair per PCA9685 datasheet.
    // For simple PWM (no phase shift): ON=0, OFF=duty.
    void setChannelDuty(int channel, int duty12bit) {
        duty12bit = std::clamp(duty12bit, 0, PWM_RESOLUTION - 1);

        const uint8_t reg = static_cast<uint8_t>(
            PCA9685_LED0_ON_L + channel * PCA9685_BYTES_PER_CHANNEL);

        uint8_t data[5];
        data[0] = reg;
        data[1] = 0x00;              // ON_L  — always start at count 0
        data[2] = 0x00;              // ON_H
        data[3] = duty12bit & 0xFF;  // OFF_L
        data[4] = (duty12bit >> 8) & 0x0F; // OFF_H (upper nibble only)

        if (write(i2cFd_, data, 5) != 5) {
            std::cerr << "[MotorControl] I2C write failed for channel "
                      << channel << "\n";
        }
    }

    void allChannelsOff() {
        for (int ch = 0; ch < 16; ++ch)
            setChannelDuty(ch, 0);
    }

    // ── Low-level I2C register write ─────────────────────────

    bool writeReg(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = { reg, value };
        if (write(i2cFd_, buf, 2) != 2) {
            std::cerr << "[MotorControl] Failed to write reg 0x"
                      << std::hex << (int)reg << std::dec << "\n";
            return false;
        }
        return true;
    }


    // ── State ────────────────────────────────────────────────

    bool initialised_ = false;
    int  i2cFd_       = -1;

    // Last written 12-bit duty cycle per motor (for change detection)
    int prevDuty_[MOTOR_COUNT] = {};

    // Remaining overdrive frames per motor (0 = not in overdrive)
    int overdriveCounter_[MOTOR_COUNT] = {};
};