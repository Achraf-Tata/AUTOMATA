#include "simulation_engine.hpp"
#include <random>
#include <algorithm>
#include <chrono>

namespace circular::engine {

void trigger_synthesis(AppState& state) {
    state.solving = true;

    CompositionalContractSolver solver;
    state.synthesis_res = solver.synthesize(state.num_rooms, state.use_parallel);
    state.controller_ready = true;

    state.solving = false;
}

void step_simulation(AppState& state) {
    const int N = state.num_rooms;
    if (state.room_temps.size() != static_cast<size_t>(N)) {
        reset_room_temperatures(state);
    }

    // 1. Evaluate current outside weather Te from configured strategy
    if (!state.weather_strategy) {
        update_weather_strategy(state);
    }
    state.current_te = state.weather_strategy->get_temperature(state.sim_time_sec);

    // 2. Feedback control evaluation: u_i in {0.0, 0.3, 0.6}
    int active_heaters = 0;
    const auto& policy = state.synthesis_res.local_policy;

    for (int i = 0; i < N; ++i) {
        int cell = UniformGrid1D::quantize(state.room_temps[i]);
        int u_idx = 0;

        if (cell >= 0 && cell < ND_CELLS) {
            u_idx = policy[cell];
        } else if (state.room_temps[i] < SAFE_MIN) {
            u_idx = 2; // Emergency maximum heating (0.6)
        } else {
            u_idx = 0; // Emergency heater OFF (0.0)
        }

        double u_val = CONTROLS[u_idx];
        state.room_heaters[i] = u_val;
        if (u_val > 0.0) ++active_heaters;
    }
    state.active_heaters_count = active_heaters;

    // 3. Step multi-room thermal network model (Slide 15)
    ThermalNetworkModel model;
    model.alpha = state.alpha;
    model.beta  = state.beta;
    model.gamma = state.gamma;
    model.Th    = state.Th;

    model.step(state.room_temps, state.room_heaters, state.current_te);

    // 4. Update performance metrics
    state.sim_time_sec += 1.0;
    ++state.sim_step;

    state.total_heater_ticks += active_heaters;
    state.total_room_ticks   += N;
    if (state.total_room_ticks > 0) {
        state.heater_duty_cycle = (100.0 * state.total_heater_ticks) / static_cast<double>(state.total_room_ticks);
    }

    // Cumulative energy estimation
    double step_power = 0.0;
    for (int i = 0; i < N; ++i) {
        step_power += state.room_heaters[i] * 5.0; // kW scaling
    }
    state.total_energy_kwh += (step_power * 1.0) / 3600.0;

    double step_min_T = 100.0;
    double step_max_T = -100.0;
    double sum_T      = 0.0;

    for (int i = 0; i < N; ++i) {
        double t_val = state.room_temps[i];
        if (t_val < step_min_T) step_min_T = t_val;
        if (t_val > step_max_T) step_max_T = t_val;
        sum_T += t_val;

        if (t_val < state.global_min_temp) state.global_min_temp = t_val;
        if (t_val > state.global_max_temp) state.global_max_temp = t_val;

        // Check safety violation (comfort zone S = [17.0, 22.0] C)
        if (t_val < SAFE_MIN || t_val > SAFE_MAX) {
            state.safety_violated   = true;
            state.violation_room_id = i;
        }
    }

    // 5. Append rolling history (subsample if recording at high speed)
    if (state.sim_step % 2 == 0 || state.history.size() < 10) {
        AppState::HistoryPoint pt{};
        pt.time_sec           = static_cast<float>(state.sim_time_sec);
        pt.Te                 = static_cast<float>(state.current_te);
        pt.min_T              = static_cast<float>(step_min_T);
        pt.max_T              = static_cast<float>(step_max_T);
        pt.avg_T              = static_cast<float>(sum_T / N);
        pt.active_heaters_pct = static_cast<float>((100.0 * active_heaters) / N);

        state.history.push_back(pt);
        if (state.history.size() > state.max_history_size) {
            state.history.pop_front();
        }
    }
}

void reset_simulation(AppState& state) {
    reset_room_temperatures(state);
}

void generate_extreme_envelope(AppState& state, int horizon_steps) {
    state.extreme_lower.clear();
    state.extreme_upper.clear();

    state.extreme_lower.reserve(horizon_steps);
    state.extreme_upper.reserve(horizon_steps);

    const int N = state.num_rooms;
    std::vector<double> T_lo(N, SAFE_MIN);
    std::vector<double> T_hi(N, SAFE_MAX);

    ThermalNetworkModel model;
    model.alpha = state.alpha;
    model.beta  = state.beta;
    model.gamma = state.gamma;
    model.Th    = state.Th;

    for (int t = 0; t < horizon_steps; ++t) {
        double min_val = T_lo[0];
        double max_val = T_hi[0];
        for (int i = 1; i < N; ++i) {
            if (T_lo[i] < min_val) min_val = T_lo[i];
            if (T_hi[i] > max_val) max_val = T_hi[i];
        }
        state.extreme_lower.push_back(min_val);
        state.extreme_upper.push_back(max_val);

        // Kamke-Muller extreme step:
        // Lower bound with coldest outside weather (-3 C) and u = 0.0
        // Upper bound with warmest outside weather (+1 C) and u = 0.6
        model.propagate_extreme_bounds(T_lo, T_hi, -3.0, 1.0);
    }
}

void generate_trajectory_bundle(AppState& state, int num_runs) {
    state.trajectory_bundle.clear();
    state.trajectory_bundle.resize(num_runs);

    const int N = state.num_rooms;
    const int horizon = 3600; // 1-hour simulation
    const int subsample = 30; // Record every 30 s for smooth rendering

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_t0(SAFE_MIN + 0.5, SAFE_MAX - 0.5);
    std::uniform_real_distribution<double> dist_noise(-0.8, 0.8);

    ThermalNetworkModel model;
    model.alpha = state.alpha;
    model.beta  = state.beta;
    model.gamma = state.gamma;
    model.Th    = state.Th;

    int safe_runs = 0;
    const auto& policy = state.synthesis_res.local_policy;

    for (int run = 0; run < num_runs; ++run) {
        auto& bundle = state.trajectory_bundle[run];
        bundle.all_safe = true;

        std::vector<double> T(N);
        for (int i = 0; i < N; ++i) {
            T[i] = dist_t0(rng);
        }

        std::vector<double> u(N, 0.0);

        for (int t = 0; t <= horizon; ++t) {
            double Te = std::clamp(TE_NOM + dist_noise(rng), -4.0, 2.0);

            double step_min = T[0];
            double step_max = T[0];

            for (int i = 0; i < N; ++i) {
                int cell = UniformGrid1D::quantize(T[i]);
                int u_idx = (cell >= 0 && cell < ND_CELLS) ? policy[cell] : 1;
                u[i] = CONTROLS[u_idx];

                if (T[i] < step_min) step_min = T[i];
                if (T[i] > step_max) step_max = T[i];

                if (T[i] < SAFE_MIN || T[i] > SAFE_MAX) {
                    bundle.all_safe = false;
                }
            }

            if (t % subsample == 0) {
                bundle.times.push_back(static_cast<float>(t));
                bundle.min_temps.push_back(static_cast<float>(step_min));
                bundle.max_temps.push_back(static_cast<float>(step_max));
            }

            model.step(T, u, Te);
        }

        if (bundle.all_safe) ++safe_runs;
    }

    state.bundle_total = num_runs;
    state.bundle_safe_count = safe_runs;
    state.bundle_ready = true;
    state.show_bundle = true;
}

} // namespace circular::engine
