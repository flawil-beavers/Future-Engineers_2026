#pragma once
#include <stdint.h>

// WRO2026 Figure8c: middle is solitary; two pillars occupy opposite ends.
// Bits0..5 represent two seats at each of three stations. Return a station
// mask of inferred empty locations, never camera-observed CLEAR evidence.
constexpr uint8_t obstacle_section_inferred_empty(uint8_t confirmedSeats)
{
    if (confirmedSeats == 4 || confirmedSeats == 8)
        return 5;
    const uint8_t first = confirmedSeats & 3;
    const uint8_t middle = confirmedSeats & 12;
    const uint8_t last = confirmedSeats & 48;
    return (middle == 0 && (first == 1 || first == 2) &&
            (last == 16 || last == 32)) ? 2 : 0;
}
static_assert(obstacle_section_inferred_empty(0) == 0, "Unknown remains unknown");
static_assert(obstacle_section_inferred_empty(4) == 5, "Middle seat0 is solitary");
static_assert(obstacle_section_inferred_empty(8) == 5, "Middle seat1 is solitary");
static_assert(obstacle_section_inferred_empty(1) == 0, "One end alone proves nothing");
static_assert(obstacle_section_inferred_empty(17) == 2, "Two end pillars exclude middle");
static_assert(obstacle_section_inferred_empty(34) == 2, "Opposite side also works");
static_assert(obstacle_section_inferred_empty(5) == 0, "End plus middle is contradictory");
static_assert(obstacle_section_inferred_empty(12) == 0, "Two middle seats are contradictory");
static_assert(obstacle_section_inferred_empty(21) == 0, "Three pillars are contradictory");

// In CHECK_ALL mode, confirmed pillars resolve only their own stations.
// Empty stations must be established by the existing camera CLEAR process.
constexpr uint8_t obstacle_section_empty_for_mode(
    uint8_t confirmedSeats, bool checkAllStations)
{
    return checkAllStations ? 0 : obstacle_section_inferred_empty(confirmedSeats);
}
static_assert(obstacle_section_empty_for_mode(4, true) == 0,
              "Check-all must inspect both ends after a middle pillar");
static_assert(obstacle_section_empty_for_mode(17, true) == 0,
              "Check-all must inspect the middle after two end pillars");
static_assert(obstacle_section_empty_for_mode(21, true) == 0,
              "Check-all must not infer from three occupied stations");
static_assert(obstacle_section_empty_for_mode(4, false) == 5,
              "Official mode keeps the solitary-middle rule");
static_assert(obstacle_section_empty_for_mode(17, false) == 2,
              "Official mode keeps the opposite-ends rule");
