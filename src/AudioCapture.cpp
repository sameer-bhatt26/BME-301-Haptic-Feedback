// ============================================================
//  AudioCapture.cpp
//  Real-time microphone capture → AudioEngine → console display.
//
//  Depends on:
//    • AudioEngine.cpp  (included directly — it uses #pragma once)
//    • PortAudio        (install: pacman -S mingw-w64-ucrt-x86_64-portaudio)
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve AudioCapture.cpp -lportaudio -lm
//
//  The PortAudio callback runs on a dedicated audio thread.
//  It writes FilterOutput into a lock-free slot that the main
//  thread reads for display / downstream motor mapping.
//  No mutexes in the audio path — keeps latency deterministic.
// ============================================================

#include "AudioEngine.cpp"   // pulls in all DSP classes

#include <portaudio.h>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <cstring>
#include <csignal>
#include <thread>
#include <chrono>

// ─────────────────────────────────────────────────────────────
//  Config
// ─────────────────────────────────────────────────────────────

// Buffer size: 256 samples @ 44100 Hz ≈ 5.8 ms per callback.
// Keep this a power of 2.  Smaller = lower latency, higher CPU.
// 128 is possible on fast machines; 512 is safer on slower ones.
static constexpr int FRAMES_PER_BUFFER = 256;

// Set to -1 to use the system default input device.
// Run with LIST_DEVICES = true first to find your mic index.
static constexpr int  INPUT_DEVICE_INDEX = -1;
static constexpr bool LIST_DEVICES       = true;


// ─────────────────────────────────────────────────────────────
//  Shared state between audio thread and main thread
//
//  We use a double-buffer (ping-pong) pattern:
//    Audio thread writes to slot [writeSlot]
//    Main thread reads from slot [1 - writeSlot]
//  The atomic flip is the only synchronisation needed.
// ─────────────────────────────────────────────────────────────

struct SharedState {
    FilterOutput slots[2];
    std::atomic<int> writeSlot { 0 };   // audio thread owns this slot
    std::atomic<bool> running  { true };
};

static SharedState gState;


// ─────────────────────────────────────────────────────────────
//  PortAudio callback — runs on the audio thread
//
//  Rules for this function:
//    • NO heap allocation (no new / malloc)
//    • NO blocking calls (no mutexes, no file I/O)
//    • NO std::cout
//  Violating these causes glitches / missed deadlines.
// ─────────────────────────────────────────────────────────────

struct CallbackData {
    AudioEngine engine;
};

static int audioCallback(const void*                     inputBuffer,
                         void*                           /* outputBuffer */,
                         unsigned long                   framesPerBuffer,
                         const PaStreamCallbackTimeInfo* /* timeInfo */,
                         PaStreamCallbackFlags           /* statusFlags */,
                         void*                           userData)
{
    auto* cb   = static_cast<CallbackData*>(userData);
    auto* pcm  = static_cast<const int16_t*>(inputBuffer);

    if (!pcm) return paContinue;   // input underrun — skip frame

    // Convert int16 → float [-1, 1] without heap allocation
    // Stack allocation is fine for small fixed-size buffers
    float floatBuf[FRAMES_PER_BUFFER];
    for (unsigned long i = 0; i < framesPerBuffer; ++i)
        floatBuf[i] = pcm[i] / 32768.0f;

    // Wrap in a std::vector view — AudioEngine takes const vector&
    // This does NOT allocate; we're constructing from a pointer range.
    const std::vector<float> samples(floatBuf, floatBuf + framesPerBuffer);

    // Run the DSP pipeline
    FilterOutput result = cb->engine.process(samples);

    // Write to the inactive slot, then atomically flip
    const int ws = gState.writeSlot.load(std::memory_order_relaxed);
    gState.slots[ws] = result;
    gState.writeSlot.store(1 - ws, std::memory_order_release);

    return gState.running.load(std::memory_order_relaxed)
               ? paContinue
               : paComplete;
}


// ─────────────────────────────────────────────────────────────
//  Console display helpers
// ─────────────────────────────────────────────────────────────

// Simple ASCII bar for a value in [0, 1]
static std::string bar(float v, int width = 20) {
    int filled = static_cast<int>(std::clamp(v, 0.0f, 1.0f) * width);
    return std::string(filled, '#') + std::string(width - filled, '-');
}

static void printOutput(const FilterOutput& out) {
    // Move cursor up BAND_COUNT lines to overwrite previous output
    std::cout << "\033[" << BAND_COUNT << "A";

    for (int b = 0; b < BAND_COUNT; ++b) {
        const auto& band = out.bands[b];
        std::cout << std::left  << std::setw(26)
                  << AudioEngine::bandName(static_cast<Band>(b))
                  << "  env ["  << bar(band.envelope) << "]  "
                  << std::fixed << std::setprecision(3) << band.envelope
                  << "   peak " << std::fixed << std::setprecision(3)
                  << band.peak
                  << "          \n";   // trailing spaces clear old chars
    }
    std::cout.flush();
}

static void listDevices() {
    int count = Pa_GetDeviceCount();
    std::cout << "\nAvailable input devices:\n";
    for (int i = 0; i < count; ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info && info->maxInputChannels > 0) {
            std::cout << "  [" << i << "] " << info->name
                      << "  (" << info->maxInputChannels << " ch, "
                      << info->defaultSampleRate << " Hz)\n";
        }
    }
    std::cout << "\nDefault input device index: "
              << Pa_GetDefaultInputDevice() << "\n\n";
}


// ─────────────────────────────────────────────────────────────
//  Ctrl-C handler — signals the audio thread to stop cleanly
// ─────────────────────────────────────────────────────────────

static void onSignal(int) {
    gState.running.store(false, std::memory_order_relaxed);
}


// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────

int main() {
    std::signal(SIGINT, onSignal);

    // ── Init PortAudio ───────────────────────────────────────
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "PortAudio init failed: " << Pa_GetErrorText(err) << "\n";
        return 1;
    }

    if (LIST_DEVICES)
        listDevices();

    // ── Choose device ────────────────────────────────────────
    const PaDeviceIndex deviceIdx = (INPUT_DEVICE_INDEX < 0)
                                        ? Pa_GetDefaultInputDevice()
                                        : INPUT_DEVICE_INDEX;

    const PaDeviceInfo* devInfo = Pa_GetDeviceInfo(deviceIdx);
    if (!devInfo) {
        std::cerr << "Invalid device index: " << deviceIdx << "\n";
        Pa_Terminate();
        return 1;
    }

    std::cout << "Using device [" << deviceIdx << "]: " << devInfo->name << "\n";
    std::cout << "Buffer: " << FRAMES_PER_BUFFER << " frames  (~"
              << std::fixed << std::setprecision(1)
              << (1000.0f * FRAMES_PER_BUFFER / SAMPLE_RATE)
              << " ms)\n\n";

    // ── Stream parameters ────────────────────────────────────
    PaStreamParameters inputParams;
    inputParams.device                    = deviceIdx;
    inputParams.channelCount              = 1;           // mono
    inputParams.sampleFormat              = paInt16;     // matches AudioEngine input
    inputParams.suggestedLatency          = devInfo->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    // ── Callback state ───────────────────────────────────────
    CallbackData cbData;
    // AudioEngine default-constructs to 44100 Hz — matches SAMPLE_RATE constant.
    // If you change SAMPLE_RATE, call: cbData.engine.configure(NEW_RATE);

    // ── Open stream ──────────────────────────────────────────
    PaStream* stream = nullptr;
    err = Pa_OpenStream(&stream,
                        &inputParams,
                        nullptr,             // no output
                        SAMPLE_RATE,
                        FRAMES_PER_BUFFER,
                        paClipOff,
                        audioCallback,
                        &cbData);

    if (err != paNoError) {
        std::cerr << "Pa_OpenStream failed: " << Pa_GetErrorText(err) << "\n";
        Pa_Terminate();
        return 1;
    }

    // ── Start streaming ──────────────────────────────────────
    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream failed: " << Pa_GetErrorText(err) << "\n";
        Pa_CloseStream(stream);
        Pa_Terminate();
        return 1;
    }

    std::cout << "Listening... (Ctrl+C to stop)\n\n";

    // Print blank lines so the first overwrite has something to move up over
    for (int i = 0; i < BAND_COUNT; ++i) std::cout << "\n";

    // ── Main loop — read output, display, hand off to mappers ─
    while (gState.running.load(std::memory_order_relaxed)) {

        // Read from the slot the audio thread just finished writing
        const int rs = 1 - gState.writeSlot.load(std::memory_order_acquire);
        const FilterOutput out = gState.slots[rs];

        printOutput(out);

        // ── TODO: pass `out` to your motor mapping layer here ──
        //
        // Examples (not implemented yet):
        //
        //   DirectMapper::map(out, pwmValues);
        //   BandMapper::map(out, pwmValues);
        //   DirectionalMapper::map(outLeft, outRight, pwmValues);
        //   motorController.write(pwmValues);
        //
        // ──────────────────────────────────────────────────────

        // ~30 Hz display refresh — well within haptic response threshold
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // ── Shutdown ─────────────────────────────────────────────
    std::cout << "\nStopping...\n";
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    return 0;
}