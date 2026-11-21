#include "app_state.hpp"
#include <cmath>

namespace circular {

void update_weather_strategy(AppState& state) {
    state.weather_strategy = WeatherFactory::create(state.weather_type, state.manual_te);
    state.current_te = state.weather_strategy->get_temperature(state.sim_time_sec);
}

void reset_room_temperatures(AppState& state) {
    const int N = state.num_rooms;
    state.room_temps.resize(N);
    state.room_heaters.assign(N, 0.0);

    // Initial safe comfort temperature around 19.5 C (comfort midpoint of [17.0, 22.0] C)
    for (int i = 0; i < N; ++i) {
        state.room_temps[i] = 19.0 + 0.1 * static_cast<double>(i % 10);
    }

    state.sim_time_sec = 0.0;
    state.sim_step     = 0;

    if (!state.weather_strategy) {
        update_weather_strategy(state);
    } else {
        state.current_te = state.weather_strategy->get_temperature(0.0);
    }

    state.active_heaters_count = 0;
    state.total_heater_ticks   = 0;
    state.total_room_ticks     = 0;
    state.heater_duty_cycle    = 0.0;
    state.global_min_temp      = SAFE_MAX;
    state.global_max_temp      = SAFE_MIN;
    state.safety_violated      = false;
    state.violation_room_id    = -1;
    state.total_energy_kwh     = 0.0;

    state.history.clear();
}

void init_app_state(AppState& state) {
    update_weather_strategy(state);
    reset_room_temperatures(state);

    // Synthesize Assume-Guarantee contract
    CompositionalContractSolver solver;
    state.synthesis_res = solver.synthesize(state.num_rooms, state.use_parallel);
    state.controller_ready = true;
}

} // namespace circular
