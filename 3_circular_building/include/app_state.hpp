#pragma once

#include "thermal_network.hpp"
#include "contract_abstraction.hpp"
#include "contract_solver.hpp"
#include "weather_strategy.hpp"

#include <vector>
#include <array>
#include <string>
#include <deque>
#include <memory>

namespace circular {

using namespace model;
using namespace abstraction;
using namespace synthesis;
using namespace environment;

// Global Application State holding building network, synthesis results, simulation engine, and UI settings
struct AppState {
    // --- Building Network Configuration (Slide 15) ---
    int    num_rooms            = 16;       // Number of circular rooms N >= 3 (Slide 15)
    double alpha                = ALPHA;    // 0.45 (inter-room conduction)
    double beta                 = BETA;     // 0.045 (exterior loss)
    double gamma                = GAMMA;    // 0.09 (heater coefficient)
    double safe_min             = SAFE_MIN; // 17.0 C (Slide 16)
    double safe_max             = SAFE_MAX; // 22.0 C (Slide 16)
    double Th                   = TH_NOM;   // 50.0 C (Slide 15)

    // --- Weather Environment Strategy (Strategy Pattern) ---
    WeatherFactory::Type weather_type = WeatherFactory::Type::Nominal;
    std::unique_ptr<IWeatherStrategy> weather_strategy;
    double current_te           = TE_NOM;   // Current ambient outside temperature Te(t)
    double manual_te            = TE_NOM;   // Manual slider value (-1.0 C default)

    // --- Synthesis Engine State (Strategy Pattern) ---
    bool   use_parallel         = true;     // OpenMP multi-core parallelization toggle
    bool   controller_ready     = false;
    bool   solving              = false;
    SynthesisResult synthesis_res;          // Reports Tabs, Tctrl, peak memory, % controllable (Slide 17)
    char   controller_filename[128] = "circular_contract.bin";

    // --- Simulation Execution State ---
    bool   sim_running          = false;
    bool   sim_paused           = false;
    double sim_time_sec         = 0.0;
    int    sim_step             = 0;
    int    sim_max_steps        = 86400;    // 24 hours (86,400 steps)
    float  sim_speed            = 60.0f;    // Playback speed (1x .. 600x)

    std::vector<double> room_temps;         // Temperatures T_1 .. T_N in [17.0, 22.0] C
    std::vector<double> room_heaters;       // Controls u_1 .. u_N in {0.0, 0.3, 0.6}

    // --- Time-Series History (Rolling plot buffer) ---
    struct HistoryPoint {
        float time_sec;
        float Te;
        float min_T;
        float max_T;
        float avg_T;
        float active_heaters_pct;
    };
    std::deque<HistoryPoint> history;
    size_t max_history_size     = 600;

    // --- Kamke-Muller Extreme Trajectory Bounds ---
    bool show_extreme_bounds    = true;
    std::vector<double> extreme_lower;      // Trajectory under coldest weather and u = 0.0
    std::vector<double> extreme_upper;      // Trajectory under warmest weather and u = 0.6

    // --- Monte Carlo Perturbation Bundle (Plot All Routes - 50 runs) ---
    struct BundleRun {
        std::vector<float> times;
        std::vector<float> min_temps;
        std::vector<float> max_temps;
        bool all_safe = true;
    };
    bool show_bundle            = false;
    bool bundle_ready           = false;
    int  bundle_total           = 50;
    int  bundle_safe_count      = 50;
    std::vector<BundleRun> trajectory_bundle;

    // --- Performance & Health Metrics ---
    int    active_heaters_count = 0;        // Number of rooms with u_i > 0
    double heater_duty_cycle    = 0.0;      // Average percentage of non-zero heating
    long long total_heater_ticks= 0;
    long long total_room_ticks  = 0;
    double global_min_temp      = 22.0;
    double global_max_temp      = 17.0;
    bool   safety_violated      = false;
    int    violation_room_id    = -1;
    double total_energy_kwh     = 0.0;

    // --- UI View Options ---
    bool show_labels            = true;
    bool show_flux_arrows       = true;
    bool show_heaters           = true;
    bool show_weather_center    = true;
    bool show_temp_plot         = true;
    int  selected_room          = -1;
};

// Initializes default application context, synthesizes contract, and sets room temperatures
void init_app_state(AppState& state);

// Resets room temperatures to comfort midpoint (19.5 C) and resets clock
void reset_room_temperatures(AppState& state);

// Re-creates weather strategy based on selected type
void update_weather_strategy(AppState& state);

} // namespace circular
