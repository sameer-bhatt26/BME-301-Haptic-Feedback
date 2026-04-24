// ============================================================
//  motor_test.cpp
//  Standalone motor test — cycles each of the 12 motors through
//  5 amplitude levels (20%, 40%, 60%, 80%, 100%) for 2 seconds
//  each. Motors fire one at a time so you can feel and map each.
//
//  Compile:
//    g++ -std=c++17 -O2 -o motor_test motor_test.cpp -li2c
//
//  Run:
//    ./motor_test
//
//  No PortAudio or gpiod needed — I2C only.
// ============================================================

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <array>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>


// ─────────────────────────────────────────────────────────────
//  Config — must match MotorControl.cpp
// ─────────────────────────────────────────────────────────────

static constexpr const char* I2C_BUS       = "/dev/i2c-1";
static constexpr uint8_t     PCA9685_ADDR  = 0x40;
static constexpr float        PWM_FREQ_HZ  = 1000.0f;
static constexpr float        PCA9685_OSC  = 25000000.0f;
static constexpr int          PWM_MAX      = 4095;

// PCA9685 registers
static constexpr uint8_t REG_MODE1     = 0x00;
static constexpr uint8_t REG_PRESCALE  = 0xFE;
static constexpr uint8_t REG_LED0_ON_L = 0x06;
static constexpr uint8_t BYTES_PER_CH  = 4;
static constexpr uint8_t MODE1_SLEEP   = 0x10;
static constexpr uint8_t MODE1_AI      = 0x20;
static constexpr uint8_t MODE1_ALLCALL = 0x01;

// Motor -> PCA9685 channel map (from MotorControl.cpp)
static constexpr std::array<int, 12> MOTOR_CHANNELS = {
    // Row 0 — elbow — low freq  (motors 6,1,0,7 -> ch 8,1,0,9)
    8,   // index 0
    1,   // index 1
    0,   // index 2
    9,   // index 3
    // Row 1 — mid freq  (motors 8,2,3,9 -> ch 10,2,3,11)
    10,  // index 4
    2,   // index 5
    3,   // index 6
    11,  // index 7
    // Row 2 — wrist — high freq  (motors 10,5,4,11 -> ch 12,5,4,13)
    12,  // index 8
    5,   // index 9
    4,   // index 10
    13,  // index 11
};

// Test amplitudes as fractions of PWM_MAX
static constexpr std::array<float, 5> AMPLITUDES = {
    0.20f,  // 20%
    0.40f,  // 40%
    0.60f,  // 60%
    0.80f,  // 80%
    1.00f,  // 100%
};

static constexpr int HOLD_MS      = 2000;  // 2 seconds per amplitude
static constexpr int OVERDRIVE_MS = 50;    // 50 ms kickstart at full power
static constexpr int MIN_DUTY     = 256;   // minimum spin threshold (~6%)

// Row labels matching AudioEngine.cpp layout
static constexpr const char* MOTOR_LABELS[12] = {
    "Motor  0 | Row 0 (Low  freq) | Col 0",
    "Motor  1 | Row 0 (Low  freq) | Col 1",
    "Motor  2 | Row 0 (Low  freq) | Col 2",
    "Motor  3 | Row 0 (Low  freq) | Col 3",
    "Motor  4 | Row 1 (Mid  freq) | Col 0",
    "Motor  5 | Row 1 (Mid  freq) | Col 1",
    "Motor  6 | Row 1 (Mid  freq) | Col 2",
    "Motor  7 | Row 1 (Mid  freq) | Col 3",
    "Motor  8 | Row 2 (High freq) | Col 0",
    "Motor  9 | Row 2 (High freq) | Col 1",
    "Motor 10 | Row 2 (High freq) | Col 2",
    "Motor 11 | Row 2 (High freq) | Col 3",
};


// ─────────────────────────────────────────────────────────────
//  I2C helpers
// ─────────────────────────────────────────────────────────────

static int gFd = -1;

static bool writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    if (write(gFd, buf, 2) != 2) {
        std::cerr << "[Error] I2C write failed reg=0x"
                  << std::hex << (int)reg << std::dec << "\n";
        return false;
    }
    return true;
}

static void setChannelDuty(int channel, int duty) {
    duty = std::clamp(duty, 0, PWM_MAX);
    const uint8_t reg = static_cast<uint8_t>(REG_LED0_ON_L + channel * BYTES_PER_CH);
    uint8_t data[5];
    data[0] = reg;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = duty & 0xFF;
    data[4] = (duty >> 8) & 0x0F;
    if (write(gFd, data, 5) != 5)
        std::cerr << "[Error] I2C write failed ch=" << channel << "\n";
}

static void allOff() {
    for (int ch = 0; ch < 16; ++ch)
        setChannelDuty(ch, 0);
}

static bool initPCA9685() {
    writeReg(REG_MODE1, MODE1_SLEEP);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    const uint8_t prescale = static_cast<uint8_t>(
        std::round(PCA9685_OSC / (4096.0f * PWM_FREQ_HZ)) - 1);
    writeReg(REG_PRESCALE, prescale);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    writeReg(REG_MODE1, MODE1_AI | MODE1_ALLCALL);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    allOff();
    return true;
}


// ─────────────────────────────────────────────────────────────
//  Progress bar helper
// ─────────────────────────────────────────────────────────────

static void printBar(float amplitude) {
    const int filled = static_cast<int>(amplitude * 20.0f);
    std::cout << "  Amplitude: ["
              << std::string(filled, '#')
              << std::string(20 - filled, '-')
              << "] " << std::setw(3) << static_cast<int>(amplitude * 100.0f)
              << "%\n";
}


// ─────────────────────────────────────────────────────────────
//  Run one motor through all 5 amplitudes
// ─────────────────────────────────────────────────────────────

static void testMotor(int motorIdx) {
    const int ch = MOTOR_CHANNELS[motorIdx];

    std::cout << "\n╔══════════════════════════════════════════╗\n";
    std::cout << "  " << MOTOR_LABELS[motorIdx] << "\n";
    std::cout << "  PCA9685 channel: " << ch << "\n";
    std::cout << "╚══════════════════════════════════════════╝\n";

    for (float amp : AMPLITUDES) {
        const int targetDuty = std::max(
            static_cast<int>(amp * PWM_MAX), MIN_DUTY);

        printBar(amp);

        // Kickstart burst
        setChannelDuty(ch, PWM_MAX);
        std::this_thread::sleep_for(std::chrono::milliseconds(OVERDRIVE_MS));

        // Hold at target amplitude
        setChannelDuty(ch, targetDuty);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(HOLD_MS - OVERDRIVE_MS));

        // Brief off gap between amplitudes so each step is distinct
        setChannelDuty(ch, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    setChannelDuty(ch, 0);
    std::cout << "  Done.\n";
}


// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────

int main() {
    std::cout << "╔══════════════════════════════════════════╗\n";
    std::cout << "║       Haptic Sleeve — Motor Test         ║\n";
    std::cout << "║  12 motors x 5 amplitudes x 2 sec each  ║\n";
    std::cout << "╚══════════════════════════════════════════╝\n";

    // Open I2C
    gFd = open(I2C_BUS, O_RDWR);
    if (gFd < 0) {
        std::cerr << "[Error] Cannot open " << I2C_BUS << "\n"
                  << "  Enable I2C: sudo raspi-config -> Interfaces -> I2C\n";
        return 1;
    }
    if (ioctl(gFd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "[Error] Cannot reach PCA9685 at 0x"
                  << std::hex << (int)PCA9685_ADDR << std::dec << "\n"
                  << "  Scan bus: i2cdetect -y 1\n";
        close(gFd); return 1;
    }

    if (!initPCA9685()) {
        close(gFd); return 1;
    }

    std::cout << "\nPCA9685 ready. Starting test in 2 seconds...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Test each motor in order
    for (int m = 0; m < 12; ++m) {
        testMotor(m);
        // 500 ms gap between motors so you know one has ended
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    allOff();
    writeReg(REG_MODE1, MODE1_SLEEP);
    close(gFd);

    std::cout << "\n✓ Test complete. All motors off.\n";
    return 0;
}