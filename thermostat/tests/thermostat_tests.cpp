#include "count_input.hpp"
#include "thermostat.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using namespace thermostat;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// Reference continuous thermal update from Slide 15 evaluated in double-precision floating point.
// Used as a golden standard to verify the integer-scaled abstraction.
double slide_update(double own, double left, double right, double heater) {
    return own + 0.45 * (left + right - 2.0 * own)
               + 0.045 * (-1.0 - own) + 0.09 * (50.0 - own) * heater;
}

// ============================================================================
// Test 1: Abstraction Verification (Comparing with the 8 Extreme Corners)
// ============================================================================
//
// For each cell [own_min, own_max], the two neighbors can independently vary
// over [17, 22] x [17, 22].
// This forms a 3-dimensional cube with 2^3 = 8 corners:
//   (own_endpoint, left_endpoint, right_endpoint)
//
// This test evaluates the slide equation at ALL 8 CORNERS in double precision,
// scales by 2000, and verifies that our integer-arithmetic build_abstraction()
// produces the EXACT same successor bitmask.
void test_abstraction() {
    // 1. Verify quantization boundaries
    require(quantize(17.0) == 0, "17 C should be in the first cell.");
    require(quantize(std::nextafter(17.5, 17.0)) == 0, "The first cell is open on the right.");
    require(quantize(17.5) == 1, "Shared endpoints belong to the upper cell.");
    require(quantize(22.0) == 9, "22 C should be in the last cell.");
    require(!quantize(22.01) && !quantize(16.99), "Out-of-range temperatures are unsafe.");
    require(!quantize(std::numeric_limits<double>::quiet_NaN()), "NaN is an invalid measurement.");

    // 2. Verify transition table against all 8 corners
    const auto table = build_abstraction();
    for (int bin = 0; bin < bin_count; ++bin) {
        for (int action = 0; action < action_count; ++action) {
            // Evaluate the continuous slide equation at all eight corners
            long lower = std::numeric_limits<long>::max();
            long upper = std::numeric_limits<long>::min();
            for (double own : {17.0 + 0.5 * bin, 17.5 + 0.5 * bin}) {
                for (double left : {17.0, 22.0}) {
                    for (double right : {17.0, 22.0}) {
                        const long tick = std::lround(2000 * slide_update(own, left, right, 0.3 * action));
                        lower = std::min(lower, tick);
                        upper = std::max(upper, tick);
                    }
                }
            }

            BinMask expected = lower < 34000 || upper > 44000 ? unsafe : 0;
            for (int target = 0; target < bin_count; ++target) {
                const int start = 34000 + 1000 * target;
                const int end = start + 1000;
                if (upper >= start && (lower < end || (target == 9 && lower == end))) {
                    expected |= bin_bit(target);
                }
            }
            require(table[bin][action] == expected, "Transition disagrees with the corner calculation.");
        }
    }
}

// ============================================================================
// Test 2: Synthesis Verification (Fixed-Point Convergence & Backward Pruning)
// ============================================================================
//
// 1. Verifies that the synthesized policy marks all 10 comfort bins as winning,
//    and selects heater setting 0.3 as the robust invariant action.
// 2. Creates an artificial transition system with an unsafe state to verify that
//    our backward fixed-point correctly cascades and eliminates unsafe chains.
void test_synthesis() {
    const auto policy = synthesize(build_abstraction());
    require(policy.winning == all_bins, "All ten comfort cells must be winning.");
    for (const auto mask : policy.allowed) {
        require(mask == action_bit(1), "The only robust heater setting is 0.3.");
    }
    require(certified_percentage(policy, 1000000) == 100.0, "All building cells are certified.");

    // Artificial test graph to test fixed-point elimination:
    //   Bin 0 -> Bin 1
    //   Bin 1 -> Bin 2
    //   Bin 2 -> Unsafe!
    //   Bin 3 -> Bin 3 (safe self-loop)
    //
    // The backward fixed point must prune Bin 2 (leads to unsafe), then Bin 1 (leads to Bin 2),
    // then Bin 0 (leads to Bin 1). Only Bin 3 should survive!
    Transitions example{};
    example[0][0] = bin_bit(1);
    example[1][0] = bin_bit(2);
    example[2][0] = unsafe | bin_bit(3);
    example[3][2] = bin_bit(3);
    const auto reduced = synthesize(example);
    require(reduced.winning == bin_bit(3), "The fixed point must remove the unsafe chain and dead ends.");
    require(reduced.allowed[3] == action_bit(2), "The fixed point must retain the only safe action.");
    require(reduced.allowed[0] == 0, "Losing cells must have no allowed action.");
}

// ============================================================================
// Test 3: Input Parsing & Guard Verification
// ============================================================================
void test_room_count_input() {
    require(parse_room_count("3") == 3, "A three-room building must be accepted.");
    require(parse_room_count("3000000000") == 3'000'000'000ULL,
            "A room count above INT_MAX must keep its value.");

    const auto maximum = std::numeric_limits<std::size_t>::max();
    const auto maximum_text = std::to_string(maximum);
    require(parse_room_count(maximum_text) == maximum, "The largest representable room count must be accepted.");

    const auto rejects = [](std::string_view text) {
        try {
            (void)parse_room_count(text);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    require(rejects(maximum_text + "0"), "Room counts beyond the integer range must be rejected.");
    require(rejects("-3") && rejects("2") && rejects("8.5") && rejects(""),
            "Room counts must be whole numbers of at least three.");

    check_live_room_limit(live_room_limit);
    bool refused_live_run = false;
    try {
        check_live_room_limit(parse_room_count("1000000000"));
    } catch (const std::invalid_argument&) {
        refused_live_run = true;
    }
    require(refused_live_run, "A billion rooms must be refused before live simulation allocates temperatures.");
    const auto policy = synthesize(build_abstraction());
    require(certified_percentage(policy, parse_room_count("1000000000")) == 100.0,
            "A billion rooms must still be eligible for certification.");
}

// ============================================================================
// Test 4: Simulation & Boundary Wrapping Verification
// ============================================================================
void test_simulation() {
    const auto policy = build_controller().policy;
    Building building({17.0, 20.0, 22.0});
    building.step(policy);
    const auto next = building.temperatures();
    require(std::abs(next[0] - 20.681) < 1e-12, "Room 0 must wrap to room 2 on its left.");
    require(std::abs(next[1] - 19.415) < 1e-12, "Room 1 must read the original snapshot.");
    require(std::abs(next[2] - 18.571) < 1e-12, "Room 2 must wrap to room 0 on its right.");
    for (int tick = 1; tick < 100; ++tick) {
        building.step(policy);
    }
    require(building.tick() == 100, "A safe trajectory must remain nonblocking.");
}

} // namespace

int main() {
    try {
        test_abstraction();
        test_synthesis();
        test_room_count_input();
        test_simulation();
        std::cout << "Thermostat tests passed.\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
