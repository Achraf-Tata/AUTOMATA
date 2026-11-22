#pragma once

#include <charconv>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace thermostat {

// Parses a non-negative integer from a string view using std::from_chars (C++17/20).
// Why std::from_chars?
//   - Zero memory allocation and zero string copies.
//   - Extremely fast and safe: clearly reports invalid characters and integer overflow
//     without relying on slow exceptions for normal flow control.
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

// Parses and validates the room count for a circular building.
// A circular ring topology requires at least 3 rooms so that each room's left and right
// neighbors are distinct physical rooms.
inline std::size_t parse_room_count(std::string_view value) {
    const auto count = parse_count(value, "Room count");
    if (count < 3) {
        throw std::invalid_argument("A circular building needs at least three rooms.");
    }
    return count;
}

// Safety threshold for the interactive live graphical simulation.
// Note: The theoretical controller certification supports ANY N up to infinity (O(1)),
// but simulating 1,000,000 live temperature trajectories in real time at 60 FPS
// requires allocating memory for 2 million doubles (~16 MB). We guard against an accidental
// input like 100,000,000 rooms freezing the user's interactive desktop window.
inline constexpr std::size_t live_room_limit = 1'000'000;

inline void check_live_room_limit(std::size_t count) {
    if (count > live_room_limit) {
        throw std::invalid_argument("Live simulation supports up to 1,000,000 rooms. "
                                    "For a larger building, enter its room count and choose Certify.");
    }
}

} // namespace thermostat
