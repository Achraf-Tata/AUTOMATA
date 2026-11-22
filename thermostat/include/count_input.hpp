#pragma once

#include <charconv>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace thermostat {

inline std::size_t parse_count(std::string_view value, std::string_view name) {
    if (!value.empty()) {
        std::size_t count{};
        const char* end = value.data() + value.size();
        const auto [parsed, error] = std::from_chars(value.data(), end, count);
        if (parsed == end) {
            if (error == std::errc{}) {
                return count;
            }
            if (error == std::errc::result_out_of_range) {
                throw std::invalid_argument(std::string(name) + " exceeds the maximum of "
                                            + std::to_string(std::numeric_limits<std::size_t>::max()) + ".");
            }
        }
    }
    throw std::invalid_argument(std::string(name) + " must be a nonnegative integer.");
}

inline std::size_t parse_room_count(std::string_view value) {
    const auto count = parse_count(value, "Room count");
    if (count < 3) {
        throw std::invalid_argument("A circular building needs at least three rooms.");
    }
    return count;
}

// The live dashboard keeps two temperature vectors and visits every room on each tick.
inline constexpr std::size_t live_room_limit = 1'000'000;

inline void check_live_room_limit(std::size_t count) {
    if (count > live_room_limit) {
        throw std::invalid_argument("Live simulation supports up to 1,000,000 rooms. "
                                    "For a larger building, enter its room count and choose Certify.");
    }
}

} // namespace thermostat
