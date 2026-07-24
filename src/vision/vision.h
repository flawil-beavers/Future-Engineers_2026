#pragma once

#include <Arduino.h>

enum class ColorType : uint8_t
{
    NONE = 0,
    RED,
    GREEN,
    ORANGE,
    BLUE
};

struct HSV
{
    uint16_t h;   // 0 ... 359
    uint8_t s;    // 0 ... 255
    uint8_t v;    // 0 ... 255
};

struct RGB
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct Blob
{
    bool found = false;
    ColorType color = ColorType::NONE;

    int16_t centerX = 0;
    int16_t centerY = 0;

    int16_t minX = 0;
    int16_t minY = 0;
    int16_t maxX = 0;
    int16_t maxY = 0;

    uint32_t area = 0;

    void reset(ColorType newColor)
    {
        found = false;
        color = newColor;

        centerX = 0;
        centerY = 0;

        minX = 0;
        minY = 0;
        maxX = 0;
        maxY = 0;

        area = 0;
    }

    int width() const
    {
        if (!found)
            return 0;

        return maxX - minX + 1;
    }

    int height() const
    {
        if (!found)
            return 0;

        return maxY - minY + 1;
    }
    int centerError(uint16_t imageWidth = 320) const
    {
    if (!found)
        return 0;

    return centerX - (imageWidth / 2);
}
};

struct VisionResult
{
    Blob red;
    Blob green;
    Blob orange;
    Blob blue;

    uint32_t processingTimeUs = 0;

    void clear()
    {
        red.reset(ColorType::RED);
        green.reset(ColorType::GREEN);
        orange.reset(ColorType::ORANGE);
        blue.reset(ColorType::BLUE);

        processingTimeUs = 0;
    }
};

class Vision
{
public:
    Vision();

    void begin();

    bool update(
        uint8_t* buffer,
        uint16_t width,
        uint16_t height
    );

    const VisionResult& getResult() const;

    HSV getHSVAt(
        const uint8_t* buffer,
        uint16_t width,
        uint16_t height,
        uint16_t x,
        uint16_t y
    ) const;

private:
    VisionResult result;

    // We inspect every second pixel.
    static constexpr uint8_t PIXEL_STEP = 2;

    // 320 / 2 = 160
    // 240 / 2 = 120
    static constexpr uint16_t MAX_SAMPLE_WIDTH = 160;
    static constexpr uint16_t MAX_SAMPLE_HEIGHT = 120;
    static constexpr uint16_t MAX_SAMPLES =
        MAX_SAMPLE_WIDTH * MAX_SAMPLE_HEIGHT;

    // Camera is mounted upside down.
    static constexpr bool ROTATE_180 = true;

    // RGB565 byte order.
    static constexpr bool RGB565_MSB_FIRST = true;

    // ============================================================
// Regions of Interest
// ============================================================

// Red / green obstacle detection
static constexpr uint16_t OBSTACLE_Y_MIN = 35;
static constexpr uint16_t OBSTACLE_Y_MAX = 239;

// Orange / blue floor line detection
static constexpr uint16_t LINE_Y_MIN = 115;
static constexpr uint16_t LINE_Y_MAX = 239;

    // One byte per sampled pixel:
    // NONE / RED / GREEN / ORANGE / BLUE
    uint8_t colorMap[MAX_SAMPLES];

    // Flood-fill queue.
    uint16_t queue[MAX_SAMPLES];

    RGB readRGB565(
        const uint8_t* buffer,
        uint32_t pixelIndex
    ) const;

    HSV rgbToHSV(const RGB& rgb) const;

    ColorType classifyColor(const HSV& hsv) const;

    uint16_t minimumBlobSamples(ColorType color) const;

    void findLargestBlobs(
        uint16_t sampleWidth,
        uint16_t sampleHeight
    );

    void processComponent(
        uint16_t startIndex,
        ColorType color,
        uint16_t sampleWidth,
        uint16_t sampleHeight
    );

    Blob& blobForColor(ColorType color);
};