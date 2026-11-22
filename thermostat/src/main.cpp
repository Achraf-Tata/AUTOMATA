#include "count_input.hpp"
#include "thermostat.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#elif defined(__linux__) || defined(__APPLE__)
#include <sys/resource.h>
#endif

namespace {

std::optional<std::size_t> peak_rss_kib() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, memory.cb)) {
        return memory.PeakWorkingSetSize / 1024;
    }
#elif defined(__linux__) || defined(__APPLE__)
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
#ifdef __APPLE__
        return static_cast<std::size_t>(usage.ru_maxrss) / 1024;
#else
        return static_cast<std::size_t>(usage.ru_maxrss);
#endif
    }
#endif
    return std::nullopt;
}

void print_peak_rss() {
    if (const auto peak = peak_rss_kib()) {
        std::cout << "Peak process RSS: " << *peak << " KiB\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--help") {
        std::cout << "Usage: thermostat_cli [rooms [ticks]]\n"
                     "Omit ticks to certify the building without simulating it.\n";
        return 0;
    }

    try {
        if (argc > 3) {
            throw std::invalid_argument("Usage: thermostat_cli [rooms [ticks]]");
        }
        const std::size_t rooms = argc >= 2 ? thermostat::parse_room_count(argv[1]) : 3;
        const std::size_t ticks = argc == 3 ? thermostat::parse_count(argv[2], "Ticks") : 0;

        const auto controller = thermostat::build_controller();
        std::cout << "Rooms: " << rooms << '\n'
                  << std::fixed << std::setprecision(3)
                  << "T_abs: " << controller.abstraction_us << " us\n"
                  << "T_ctrl: " << controller.synthesis_us << " us\n"
                  << "Certified initial cells: "
                  << thermostat::certified_percentage(controller.policy, rooms) << "%\n";
        print_peak_rss();

        if (argc == 3) {
            std::vector<double> initial(rooms);
            for (std::size_t i = 0; i < rooms; ++i) {
                initial[i] = i % 2 == 0 ? thermostat::min_temperature : thermostat::max_temperature;
            }
            thermostat::Building building(std::move(initial));
            for (std::size_t tick = 0; tick < ticks; ++tick) {
                building.step(controller.policy);
            }
            const auto temperatures = building.temperatures();
            const auto [coldest, warmest] = std::minmax_element(temperatures.begin(), temperatures.end());
            std::cout << "After " << building.tick() << " ticks: ["
                      << *coldest << ", " << *warmest << "] C\n";
            print_peak_rss();
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
