#pragma once

#include <Arduino.h>
#include "vision.h"

constexpr uint8_t COURSE_SECTION_COUNT = 4;
constexpr uint8_t COURSE_MAX_OBSTACLES_PER_SECTION = 2;

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
    bool originKnown = false;
    bool visited = false;
    // True only after the complete section has been driven with camera
    // learning enabled. A learned empty section is not an unknown section.
    bool learningComplete = false;
    int8_t successfulLane = 0;
};

void course_map_reset();

void course_map_enter_section(
    uint8_t section,
    bool originKnown);

void course_map_clear_obstacles(uint8_t section);

void course_map_record_obstacle(
    uint8_t section,
    uint8_t lap,
    ColorType color,
    float sectionDistanceMm,
    int16_t imageX,
    int16_t bottomY);

const CourseSection &course_map_get_section(uint8_t section);
void course_map_record_successful_lane(
    uint8_t section,
    int8_t lane);
void course_map_mark_learning_complete(uint8_t section);
void course_map_print();
