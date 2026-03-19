// ============================================================
//  main.cpp
//  Entry point for the haptic feedback sleeve on Raspberry Pi.
//
//  Ties together:
//    AudioEngine   — DSP filtering (AudioEngine.cpp)
//    MotorControl  — GPIO output   (MotorControl.cpp)
//
//  Audio thread (PortAudio callback):
//    Mic → AudioEngine::process() → FilterOutput → shared slot
//
//  Main thread (~30 Hz):
//    Read shared slot → MotorController::update() → GPIO pins
//
//  Install dependencies on Pi:
//    sudo apt install libportaudio2 libportaudio-dev pigpio
//    sudo pigpiod
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -lportaudio -lpigpio -lrt -lm
//
//  Run:
//    sudo ./HapticSleeve
// ============================================================

#include "MotorControl.cpp"   // pulls in AudioEngine.cpp transitively

#include <portaudio.h>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>

// ─────────────────────────────────────────────────────────────
//  Audio config
// ─────────────────────────────────────────────────────────────

// 256 samples @ 44100 Hz ≈ 5.8 ms per callback.
// Increase to 512 if you get xruns on the Pi.
static constexpr int FRAMES_PER_BUFFER  = 256;

// -1 = use default input device (usually the USB mic on Pi)
static constexpr int INPUT_DEVICE_INDEX = -1;

// Print available mic devices on startup (handy for first run)
static constexpr bool LIST_DEVICES = true;


// ─────────────────────────────────────────────────────────────
//  Shared state  (audio thread ↔ main thread)
//  Lock-free ping-pong buffer — no mutexes in the audio path.
// ─────────────────────────────────────────────────────────────

struct SharedState {
    FilterOutput slots[2];
    std::atomic<int>  writeSlot { 0 };
    std::atomic<bool> running   { true };
};

static SharedState gState;


// ─────────────────────────────────────────────────────────────
//  PortAudio callback  (audio thread — no heap alloc, no I/O)
// ─────────────────────────────────────────────────────────────

struct CallbackData { AudioEngine engine; };

static int audioCallback(const void*                     inputBuffer,
                         void*                           /* out */,
                         unsigned long                   framesPerBuffer,
                         const PaStreamCallbackTimeInfo* /* time */,
                         PaStreamCallbackFlags           /* flags */,
                         void*                           userData)
{
    auto* cb  = static_cast<CallbackData*>(userData);
    auto* pcm = static_cast<const int16_t*>(inputBuffer);
    if (!pcm) return paContinue;

    // int16 → float, stack allocated
    float buf[FRAMES_PER_BUFFER];
    for (unsigned long i = 0; i < framesPerBuffer; ++i)
        buf[i] = pcm[i] / 32768.0f;

    const std::vector<float> samples(buf, buf + framesPerBuffer);
    FilterOutput result = cb->engine.process(samples);

    // Flip ping-pong slot
    const int ws = gState.writeSlot.load(std::memory_order_relaxed);
    gState.slots[ws] = result;
    gState.writeSlot.store(1 - ws, std::memory_order_release);

    return gState.running.load(std::memory_order_relaxed) ? paContinue : paComplete;
}


// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────

static void listDevices() {
    std::cout << "\nAvailable input devices:\n";
    for (int i = 0; i < Pa_GetDeviceCount(); ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info && info->maxInputChannels > 0)
            std::cout << "  [" << i << "] " << info->name << "\n";
    }
    std::cout << "  Default input: [" << Pa_GetDefaultInputDevice() << "]\n\n";
}

// ASCII band meter for console feedback
static void printStatus(const FilterOutput& out, const MotorController& motors) {
    std::cout << "\033[8A";   // move cursor up 8 lines to overwrite
    for (int b = 0; b < BAND_COUNT; ++b) {
        float e = out.bands[b].envelope;
        int filled = static_cast<int>(std::clamp(e, 0.0f, 1.0f) * 20);
        std::cout << std::left << std::setw(24)
                  << AudioEngine::bandName(static_cast<Band>(b))
                  << " [" << std::string(filled, '#')
                          << std::string(20 - filled, '-') << "] "
                  << std::fixed << std::setprecision(3) << e << "\n";
    }
    motors.printState();
    std::cout << "\n";
    std::cout.flush();
}

static void onSignal(int) {
    gState.running.store(false, std::memory_order_relaxed);
}


// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────

int main() {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // ── GPIO init ────────────────────────────────────────────
    MotorController motors;
    if (!motors.init()) return 1;

    // ── PortAudio init ───────────────────────────────────────
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "PortAudio init failed: " << Pa_GetErrorText(err) << "\n";
        return 1;
    }

    if (LIST_DEVICES) listDevices();

    const PaDeviceIndex devIdx = (INPUT_DEVICE_INDEX < 0)
                                     ? Pa_GetDefaultInputDevice()
                                     : INPUT_DEVICE_INDEX;

    const PaDeviceInfo* devInfo = Pa_GetDeviceInfo(devIdx);
    if (!devInfo) {
        std::cerr << "Invalid device index " << devIdx << "\n";
        Pa_Terminate();
        return 1;
    }

    std::cout << "Mic: [" << devIdx << "] " << devInfo->name << "\n";
    std::cout << "Buffer: " << FRAMES_PER_BUFFER << " samples  (~"
              << std::fixed << std::setprecision(1)
              << 1000.0f * FRAMES_PER_BUFFER / SAMPLE_RATE << " ms)\n\n";

    // ── Open audio stream ────────────────────────────────────
    PaStreamParameters params;
    params.device                    = devIdx;
    params.channelCount              = 1;
    params.sampleFormat              = paInt16;
    params.suggestedLatency          = devInfo->defaultLowInputLatency;
    params.hostApiSpecificStreamInfo = nullptr;

    CallbackData cbData;

    PaStream* stream = nullptr;
    err = Pa_OpenStream(&stream, &params, nullptr,
                        SAMPLE_RATE, FRAMES_PER_BUFFER,
                        paClipOff, audioCallback, &cbData);
    if (err != paNoError) {
        std::cerr << "Pa_OpenStream: " << Pa_GetErrorText(err) << "\n";
        Pa_Terminate();
        return 1;
    }

    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream: " << Pa_GetErrorText(err) << "\n";
        Pa_CloseStream(stream);
        Pa_Terminate();
        return 1;
    }

    std::cout << "Running — Ctrl+C to stop\n\n";
    // Reserve lines for the status display
    for (int i = 0; i < BAND_COUNT + 2; ++i) std::cout << "\n";

    // ── Main loop ────────────────────────────────────────────
    while (gState.running.load(std::memory_order_relaxed)) {

        // Read the slot the audio thread just finished
        const int rs = 1 - gState.writeSlot.load(std::memory_order_acquire);
        const FilterOutput out = gState.slots[rs];

        // Drive motors
        motors.update(out);

        // Console feedback (~30 Hz is plenty for a human to read)
        printStatus(out, motors);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // ── Shutdown ─────────────────────────────────────────────
    std::cout << "\nShutting down...\n";
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    motors.shutdown();   // allOff() + gpioTerminate()

    return 0;
}