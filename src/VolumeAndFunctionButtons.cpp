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

//Everything below needs to be added to the main method
// // --- GPIO EVENT LISTENER THREAD ---
// void gpio_event_loop() {
//     // Note: Use /dev/gpiochip4 for Raspberry Pi 4/5. 
//     // Use /dev/gpiochip0 for Pi 3 and older.
//     auto chip = gpiod::chip("/dev/gpiochip4"); 
    
//     gpiod::line::offset power_pin = 17;
//     gpiod::line::offset func_pin = 27;
//     gpiod::line::offset volup_pin = 22;
//     gpiod::line::offset voldown_pin = 23;

//     // Configure pins as inputs with pull-ups, falling edge detection (press), and 100ms debounce
//     gpiod::line_settings settings;
//     settings.set_direction(gpiod::line::direction::INPUT)
//             .set_bias(gpiod::line::bias::PULL_UP)
//             .set_edge_detection(gpiod::line::edge::FALLING)
//             .set_debounce_period(std::chrono::milliseconds(100));

//     gpiod::line_config line_cfg;
//     line_cfg.add_line_settings(power_pin, settings)
//             .add_line_settings(func_pin, settings)
//             .add_line_settings(volup_pin, settings)
//             .add_line_settings(voldown_pin, settings);

//     gpiod::request_config req_cfg;
//     req_cfg.set_consumer("button_controller");

//     auto request = chip.request_lines(req_cfg, line_cfg);
//     gpiod::edge_event_buffer buffer;

//     while (keep_running) {
//         // Wait for events with a 100ms timeout so the loop can exit cleanly when keep_running turns false
//         if (request.wait_edge_events(std::chrono::milliseconds(100))) {
//             request.read_edge_events(buffer);
//             for (const auto& event : buffer) {
//                 auto offset = event.line_offset();
//                 if (offset == power_pin) toggle_power();
//                 else if (offset == func_pin) switch_function();
//                 else if (offset == volup_pin) increase_volume();
//                 else if (offset == voldown_pin) decrease_volume();
//             }
//         }
//     }
// }

// // --- SIGNAL HANDLER (For Ctrl+C) ---
// void signal_handler(int signum) {
//     keep_running = false;
// }

// // --- MAIN CONTINUOUS LOOP ---
// int main() {
//     // Catch Ctrl+C to exit safely
//     std::signal(SIGINT, signal_handler);

//     safe_print("Script started. System is currently OFF.");
//     safe_print("Buttons: Power (17) | Function (27) | Vol+ (22) | Vol- (23)");

//     // Start the GPIO listening loop in the background
//     std::thread event_thread(gpio_event_loop);

//     while (keep_running) {
//         if (system_on) {
//             safe_print("[Running] Function: " + std::to_string(current_function) + 
//                        " | Volume: " + std::to_string(current_volume) + "%");
            
//             // Sleep in smaller chunks so Ctrl+C responds faster than waiting 3 full seconds
//             for(int i = 0; i < 30 && keep_running; ++i) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
//             }
//         } else {
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }
//     }

//     safe_print("\nExiting script safely...");
    
//     // Clean up the background thread before exiting
//     if (event_thread.joinable()) {
//         event_thread.join();
//     }

//     return 0;
// }