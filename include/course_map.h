#pragma once

#include <Arduino.h>
#include "vision.h"

constexpr uint8_t COURSE_SECTION_COUNT = 4;
constexpr uint8_t COURSE_MAX_OBSTACLES_PER_SECTION = 2;
constexpr uint8_t COURSE_STATIONS_PER_SECTION = 3;
constexpr uint8_t COURSE_SEATS_PER_STATION = 2;

enum CourseSeatState : uint8_t
{
    COURSE_SEAT_UNKNOWN,
    COURSE_SEAT_CLEAR,
    COURSE_SEAT_RED,
    COURSE_SEAT_GREEN
};

struct CourseSeat
{
    CourseSeatState state = COURSE_SEAT_UNKNOWN;
    uint8_t observations = 0;
    float xMm = 0.0f;
    float yMm = 0.0f;
};

struct CourseObstacle
{
    bool known = false;
    ColorType color = ColorType::NONE;
    float firstDetectionDistanceMm = 0.0f;
    int16_t firstImageX = 0;
    int16_t firstBottomY = 0;
    uint8_t firstLap = 0;
    uint8_t observations = 0;
};

struct CourseSection
{
    CourseObstacle obstacles[COURSE_MAX_OBSTACLES_PER_SECTION];
    CourseSeat seats[COURSE_STATIONS_PER_SECTION]
                    [COURSE_SEATS_PER_STATION];
    bool originKnown = false;
    bool visited = false;
    int8_t successfulLane = 0;
};

void course_map_reset();

void course_map_enter_section(
    uint8_t section,
    bool originKnown);

void course_map_record_obstacle(
    uint8_t section,
    uint8_t lap,
    ColorType color,
    float sectionDistanceMm,
    int16_t imageX,
    int16_t bottomY);

/** Record the canonical known-geometry seat used by the Pure Pursuit path. */
void course_map_record_seat_obstacle(
    uint8_t section,
    uint8_t station,
    uint8_t side,
    ColorType color,
    float xMm,
    float yMm);

/** Mark both possible seats at a station clear after adequate camera views. */
void course_map_record_clear_station(
    uint8_t section,
    uint8_t station,
    float rightXmm,
    float rightYmm,
    float leftXmm,
    float leftYmm);

const CourseSection &course_map_get_section(uint8_t section);
void course_map_record_successful_lane(
    uint8_t section,
    int8_t lane);
void course_map_print();
