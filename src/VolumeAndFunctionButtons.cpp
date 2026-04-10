#include <iostream>
#include <atomic>
#include <mutex>
#include <string>

// --- SYSTEM STATES ---
// If these are defined in another file, change them to 'extern std::atomic<...>'
std::atomic<bool> system_on{true}; // Set to true here so the functions work if you test them immediately
std::atomic<int> current_function{1};
std::atomic<int> current_volume{50};

std::mutex cout_mutex; // Prevents overlapping console output if called from multiple threads

// Thread-safe print helper
void safe_print(const std::string& msg) {
    std::lock_guard<std::mutex> lock(cout_mutex);
    std::cout << msg << std::endl;
}

// --- CORE BEHAVIORS ---
void switch_function() {
    if (system_on) {
        // Calculate next function safely
        int next_func = current_function + 1;
        if (next_func > 3) {
            next_func = 1;
        }
        current_function = next_func;
        
        safe_print("\n>>> Switched to Function " + std::to_string(current_function) + " <<<");
    }
}

void increase_volume() {
    if (system_on) {
        int vol = current_volume + 5;
        
        // Clamp the maximum volume to 100%
        if (vol > 100) {
            current_volume = 100;
            safe_print("[Volume Maxed Out at 100%]");
        } else {
            current_volume = vol;
            safe_print("Volume Increased: " + std::to_string(current_volume) + "%");
        }
    }
}

void decrease_volume() {
    if (system_on) {
        int vol = current_volume - 5;
        
        // Clamp the minimum volume to 0%
        if (vol < 0) {
            current_volume = 0;
            safe_print("[Volume Muted at 0%]");
        } else {
            current_volume = vol;
            safe_print("Volume Decreased: " + std::to_string(current_volume) + "%");
        }
    }
}