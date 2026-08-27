#include "course_map.h"

#include <math.h>
#include "logger.h"

#define Serial robot_logger

static CourseSection courseSections[COURSE_SECTION_COUNT];

static const char *colorName(ColorType color)
{
    if (color == ColorType::RED)
        return "RED";
    if (color == ColorType::GREEN)
        return "GREEN";
    return "NONE";
}

void course_map_reset()
{
    for (uint8_t section = 0; section < COURSE_SECTION_COUNT; ++section)
    {
        courseSections[section] = CourseSection();
    }

    Serial.println("[MAP] Reset");
}

void course_map_enter_section(
    uint8_t section,
    bool originKnown)
{
    if (section >= COURSE_SECTION_COUNT)
        return;

    courseSections[section].visited = true;
    courseSections[section].originKnown =
        courseSections[section].originKnown || originKnown;

    Serial.print("[MAP] Enter section ");
    Serial.print(section);
    Serial.print(" origin=");
    Serial.println(originKnown ? "KNOWN" : "START_OFFSET");
}

void course_map_record_obstacle(
    uint8_t section,
    uint8_t lap,
    ColorType color,
    float sectionDistanceMm,
    int16_t imageX,
    int16_t bottomY)
{
    if (
        section >= COURSE_SECTION_COUNT ||
        (color != ColorType::RED && color != ColorType::GREEN))
    {
        return;
    }

    CourseSection &courseSection = courseSections[section];

    // At startup the vehicle can be placed in any of the six zones, so the
    // distance origin of section 0 is unknown. Keep the event in the log but
    // learn its fixed seat only after returning through a calibrated corner.
    if (!courseSection.originKnown)
    {
        Serial.print("[MAP] Unanchored start-section ");
        Serial.print(colorName(color));
        Serial.print(" offset_mm=");
        Serial.print(sectionDistanceMm, 0);
        Serial.print(" x=");
        Serial.print(imageX);
        Serial.print(" bottom=");
        Serial.println(bottomY);
        // Colour and encounter order are still valuable. The learned-lap
        // planner deliberately ignores this unanchored millimetre value.
    }

    // A completed avoidance can occasionally see the same pillar again.
    // Merge only nearby observations of the same colour. Two official seats
    // in one section are far enough apart to remain separate.
    for (uint8_t i = 0; i < COURSE_MAX_OBSTACLES_PER_SECTION; ++i)
    {
        CourseObstacle &stored = courseSection.obstacles[i];
        if (
            stored.known &&
            stored.color == color &&
            fabsf(stored.firstDetectionDistanceMm - sectionDistanceMm) <
                300.0f)
        {
            if (stored.observations < 255)
                ++stored.observations;
            return;
        }
    }

    for (uint8_t i = 0; i < COURSE_MAX_OBSTACLES_PER_SECTION; ++i)
    {
        CourseObstacle &stored = courseSection.obstacles[i];
        if (stored.known)
            continue;

        stored.known = true;
        stored.color = color;
        stored.firstDetectionDistanceMm = sectionDistanceMm;
        stored.firstImageX = imageX;
        stored.firstBottomY = bottomY;
        stored.firstLap = lap;
        stored.observations = 1;

        Serial.print("[MAP] S");
        Serial.print(section);
        Serial.print(" obstacle ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(colorName(color));
        Serial.print(" local_mm=");
        Serial.print(sectionDistanceMm, 0);
        Serial.print(" x=");
        Serial.print(imageX);
        Serial.print(" bottom=");
        Serial.println(bottomY);
        return;
    }

    Serial.print("[MAP] Section ");
    Serial.print(section);
    Serial.println(" already contains two obstacles");
}

void course_map_record_seat_obstacle(
    uint8_t section,
    uint8_t station,
    uint8_t side,
    ColorType color,
    float xMm,
    float yMm)
{
    if (section >= COURSE_SECTION_COUNT ||
        station >= COURSE_STATIONS_PER_SECTION ||
        side >= COURSE_SEATS_PER_STATION ||
        (color != ColorType::RED && color != ColorType::GREEN))
        return;

    CourseSeat &seat = courseSections[section].seats[station][side];
    seat.state = color == ColorType::RED
                     ? COURSE_SEAT_RED
                     : COURSE_SEAT_GREEN;
    seat.xMm = xMm;
    seat.yMm = yMm;
    if (seat.observations < 255)
        ++seat.observations;

    // The official geometry permits one pillar position at a station. Once
    // one side is confirmed, the opposite seat is known to be unoccupied.
    CourseSeat &opposite =
        courseSections[section].seats[station][side == 0 ? 1 : 0];
    if (opposite.state == COURSE_SEAT_UNKNOWN)
        opposite.state = COURSE_SEAT_CLEAR;

    courseSections[section].visited = true;
    courseSections[section].originKnown = true;

    Serial.print("[MAP] Confirmed S");
    Serial.print(section);
    Serial.print(" station=");
    Serial.print(station);
    Serial.print(" side=");
    Serial.print(side == 0 ? "RIGHT" : "LEFT");
    Serial.print(" color=");
    Serial.println(colorName(color));
}

void course_map_record_clear_station(
    uint8_t section,
    uint8_t station,
    float rightXmm,
    float rightYmm,
    float leftXmm,
    float leftYmm)
{
    if (section >= COURSE_SECTION_COUNT ||
        station >= COURSE_STATIONS_PER_SECTION)
        return;

    CourseSeat &right = courseSections[section].seats[station][0];
    CourseSeat &left = courseSections[section].seats[station][1];
    if (right.state == COURSE_SEAT_UNKNOWN)
        right.state = COURSE_SEAT_CLEAR;
    if (left.state == COURSE_SEAT_UNKNOWN)
        left.state = COURSE_SEAT_CLEAR;
    right.xMm = rightXmm;
    right.yMm = rightYmm;
    left.xMm = leftXmm;
    left.yMm = leftYmm;
    if (right.observations < 255)
        ++right.observations;
    if (left.observations < 255)
        ++left.observations;

    courseSections[section].visited = true;
    courseSections[section].originKnown = true;
    Serial.print("[MAP] Clear S");
    Serial.print(section);
    Serial.print(" station=");
    Serial.println(station);
}

const CourseSection &course_map_get_section(uint8_t section)
{
    static CourseSection empty;
    if (section >= COURSE_SECTION_COUNT)
        return empty;
    return courseSections[section];
}

void course_map_record_successful_lane(
    uint8_t section,
    int8_t lane)
{
    if (section >= COURSE_SECTION_COUNT ||
        (lane != -1 && lane != 1))
        return;

    courseSections[section].successfulLane = lane;
    Serial.print("[MAP] S");
    Serial.print(section);
    Serial.print(" successful lane=");
    Serial.println(lane < 0 ? "LEFT" : "RIGHT");
}

void course_map_print()
{
    Serial.println("===== COURSE MAP =====");
    for (uint8_t section = 0; section < COURSE_SECTION_COUNT; ++section)
    {
        const CourseSection &courseSection = courseSections[section];
        Serial.print("S");
        Serial.print(section);
        Serial.print(" origin=");
        Serial.println(courseSection.originKnown ? "KNOWN" : "UNKNOWN");
        Serial.print("  successful_lane=");
        Serial.println(
            courseSection.successfulLane < 0
                ? "LEFT"
                : (courseSection.successfulLane > 0
                       ? "RIGHT"
                       : "UNKNOWN"));

        for (uint8_t i = 0; i < COURSE_MAX_OBSTACLES_PER_SECTION; ++i)
        {
            const CourseObstacle &stored = courseSection.obstacles[i];
            if (!stored.known)
                continue;

            Serial.print("  #");
            Serial.print(i);
            Serial.print(" ");
            Serial.print(colorName(stored.color));
            Serial.print(" local_mm=");
            Serial.print(stored.firstDetectionDistanceMm, 0);
            Serial.print(" seen=");
            Serial.println(stored.observations);
        }

        Serial.print("  seats=");
        for (uint8_t station = 0;
             station < COURSE_STATIONS_PER_SECTION;
             ++station)
        {
            for (uint8_t side = 0;
                 side < COURSE_SEATS_PER_STATION;
                 ++side)
            {
                const CourseSeatState state =
                    courseSection.seats[station][side].state;
                const char symbol =
                    state == COURSE_SEAT_RED
                        ? 'R'
                        : (state == COURSE_SEAT_GREEN
                               ? 'G'
                               : (state == COURSE_SEAT_CLEAR ? '-' : '?'));
                Serial.print(symbol);
            }
            if (station + 1 < COURSE_STATIONS_PER_SECTION)
                Serial.print(' ');
        }
        Serial.println();
    }
}
