#pragma once

#include "vision.h"
#include "config.h"

// One shape gate for component selection and acquisition.
inline bool obstacle_blob_shape_valid(
    const Blob *obstacle)
{
    if (obstacle == nullptr)
    {
        return false;
    }

    if (!obstacle->found)
    {
        return false;
    }

    if (
        obstacle->color !=
            ColorType::RED &&
        obstacle->color !=
            ColorType::GREEN)
    {
        return false;
    }

    const uint32_t minimumArea =
        (obstacle->color == ColorType::RED)
            ? OBSTACLE_RED_MIN_AREA
            : OBSTACLE_GREEN_MIN_AREA;

    const int minimumHeight =
        (obstacle->color == ColorType::RED)
            ? OBSTACLE_RED_MIN_HEIGHT
            : OBSTACLE_GREEN_MIN_HEIGHT;

    if (obstacle->area < minimumArea)
    {
        return false;
    }

    if (obstacle->height() < minimumHeight)
    {
        return false;
    }

    if (obstacle->maxY < OBSTACLE_MIN_BOTTOM_Y)
    {
        return false;
    }

    if (obstacle->minY > OBSTACLE_MAX_TOP_Y)
    {
        return false;
    }

    if (
        obstacle->centerX < OBSTACLE_START_MIN_X ||
        obstacle->centerX > OBSTACLE_START_MAX_X ||
        obstacle->width() > OBSTACLE_MAX_START_WIDTH ||
        obstacle->height() > OBSTACLE_MAX_START_HEIGHT)
    {
        return false;
    }

    if (
        static_cast<float>(obstacle->width()) >
        static_cast<float>(obstacle->height()) *
            OBSTACLE_MAX_WIDTH_HEIGHT_RATIO)
    {
        return false;
    }

    return true;
}
