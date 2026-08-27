#include "vision.h"
#include "config.h"

Vision::Vision()
{
    result.clear();
}

void Vision::begin()
{
    result.clear();

    // Populate from the existing conversion and threshold functions so this
    // is bit-for-bit equivalent to the former per-pixel classification.
    for (uint32_t raw = 0; raw <= 0xFFFFU; ++raw)
    {
        RGB rgb;
        const uint16_t rgb565 = static_cast<uint16_t>(raw);
        const uint8_t r5 = (rgb565 >> 11) & 0x1F;
        const uint8_t g6 = (rgb565 >> 5) & 0x3F;
        const uint8_t b5 = rgb565 & 0x1F;
        rgb.r = (r5 << 3) | (r5 >> 2);
        rgb.g = (g6 << 2) | (g6 >> 4);
        rgb.b = (b5 << 3) | (b5 >> 2);
        colorLookup[raw] = static_cast<uint8_t>(
            classifyColor(rgbToHSV(rgb)));
    }
}

const VisionResult &Vision::getResult() const
{
    return result;
}

// ============================================================
// RGB565 -> RGB888
// ============================================================

RGB Vision::readRGB565(
    const uint8_t *buffer,
    uint32_t pixelIndex) const
{
    const uint32_t byteIndex = pixelIndex * 2;

    uint16_t raw;

    if (RGB565_MSB_FIRST)
    {
        raw =
            (static_cast<uint16_t>(buffer[byteIndex]) << 8) |
            buffer[byteIndex + 1];
    }
    else
    {
        raw =
            static_cast<uint16_t>(buffer[byteIndex]) |
            (static_cast<uint16_t>(buffer[byteIndex + 1]) << 8);
    }

    RGB rgb;

    const uint8_t r5 = (raw >> 11) & 0x1F;
    const uint8_t g6 = (raw >> 5) & 0x3F;
    const uint8_t b5 = raw & 0x1F;

    rgb.r = (r5 << 3) | (r5 >> 2);
    rgb.g = (g6 << 2) | (g6 >> 4);
    rgb.b = (b5 << 3) | (b5 >> 2);

    return rgb;
}

uint16_t Vision::readRGB565Raw(
    const uint8_t *buffer,
    uint32_t pixelIndex) const
{
    const uint32_t byteIndex = pixelIndex * 2;
    if (RGB565_MSB_FIRST)
    {
        return
            (static_cast<uint16_t>(buffer[byteIndex]) << 8) |
            buffer[byteIndex + 1];
    }
    return
        static_cast<uint16_t>(buffer[byteIndex]) |
        (static_cast<uint16_t>(buffer[byteIndex + 1]) << 8);
}

// ============================================================
// RGB -> HSV
// ============================================================

HSV Vision::rgbToHSV(const RGB &rgb) const
{
    const uint8_t maxValue =
        max(rgb.r, max(rgb.g, rgb.b));

    const uint8_t minValue =
        min(rgb.r, min(rgb.g, rgb.b));

    const uint8_t delta = maxValue - minValue;

    HSV hsv;

    hsv.v = maxValue;

    if (maxValue == 0)
    {
        hsv.s = 0;
    }
    else
    {
        hsv.s =
            static_cast<uint16_t>(delta) * 255 /
            maxValue;
    }

    if (delta == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    int16_t hue;

    if (maxValue == rgb.r)
    {
        hue =
            60 *
            (static_cast<int16_t>(rgb.g) -
             static_cast<int16_t>(rgb.b)) /
            delta;
    }
    else if (maxValue == rgb.g)
    {
        hue =
            120 +
            60 *
                (static_cast<int16_t>(rgb.b) -
                 static_cast<int16_t>(rgb.r)) /
                delta;
    }
    else
    {
        hue =
            240 +
            60 *
                (static_cast<int16_t>(rgb.r) -
                 static_cast<int16_t>(rgb.g)) /
                delta;
    }

    if (hue < 0)
    {
        hue += 360;
    }

    hsv.h = hue;

    return hsv;
}

// ============================================================
// Colour classification
//
// Thresholds based on your measured colours:
//
// RED:
// H=0   S=217 V=156
//
// GREEN:
// H=120 S=98  V=67
//
// BLUE:
// H=240 S=70  V=90
//
// ORANGE:
// H=14  S=144 V=132
// ============================================================

ColorType Vision::classifyColor(const HSV &hsv) const
{
    // The measured green WRO block is very dark with this camera
    // (typically V=28). Reject only pixels darker than that sample.
    if (hsv.v < 20)
    {
        return ColorType::NONE;
    }

    // RED

    if (
        (hsv.h <= VISION_RED_HUE_LOW_MAX ||
         hsv.h >= VISION_RED_HUE_HIGH_MIN) &&
        hsv.s >= VISION_RED_MIN_SATURATION &&
        hsv.v >= VISION_RED_MIN_VALUE)
    {
        return ColorType::RED;
    }

    // ORANGE

    if (
        hsv.h >= VISION_ORANGE_HUE_MIN &&
        hsv.h <= VISION_ORANGE_HUE_MAX &&
        hsv.s >= 90 &&
        hsv.v >= 60)
    {
        return ColorType::ORANGE;
    }

    // GREEN

    if (
        hsv.h >= 45 &&
        hsv.h <= 180 &&
        hsv.s >= 30 &&
        hsv.v >= 20 &&
        hsv.v <= 100)
    {
        return ColorType::GREEN;
    }

    // BLUE

    if (
        hsv.h >= 200 &&
        hsv.h <= 270 &&
        hsv.s >= 45 &&
        hsv.v >= 45)
    {
        return ColorType::BLUE;
    }

    return ColorType::NONE;
}

// ============================================================
// Minimum blob sizes
// ============================================================

uint16_t Vision::minimumBlobSamples(ColorType color) const
{
    switch (color)
    {
    // Blocks should create relatively large regions.
    case ColorType::RED:
    case ColorType::GREEN:
        return 20;

    // Lines can be thinner.
    case ColorType::ORANGE:
    case ColorType::BLUE:
        return 8;

    default:
        return 65535;
    }
}

// ============================================================
// Get corresponding result blob
// ============================================================

Blob &Vision::blobForColor(ColorType color)
{
    switch (color)
    {
    case ColorType::RED:
        return result.red;

    case ColorType::GREEN:
        return result.green;

    case ColorType::ORANGE:
        return result.orange;

    case ColorType::BLUE:
        return result.blue;

    default:
        return result.red;
    }
}

// ============================================================
// Process one connected component
// ============================================================

void Vision::processComponent(
    uint16_t startIndex,
    ColorType color,
    uint16_t sampleWidth,
    uint16_t sampleHeight)
{
    uint16_t queueRead = 0;
    uint16_t queueWrite = 0;

    queue[queueWrite++] = startIndex;

    // Mark as visited immediately.
    colorMap[startIndex] =
        static_cast<uint8_t>(ColorType::NONE);

    uint32_t sampleCount = 0;

    uint32_t sumX = 0;
    uint32_t sumY = 0;

    int16_t minX = 32767;
    int16_t minY = 32767;

    int16_t maxX = -1;
    int16_t maxY = -1;

    while (queueRead < queueWrite)
    {
        const uint16_t index =
            queue[queueRead++];

        const uint16_t gridX =
            index % sampleWidth;

        const uint16_t gridY =
            index / sampleWidth;

        const int16_t imageX =
            gridX * PIXEL_STEP;

        const int16_t imageY =
            gridY * PIXEL_STEP;

        ++sampleCount;

        sumX += imageX;
        sumY += imageY;

        if (imageX < minX)
            minX = imageX;

        if (imageX > maxX)
            maxX = imageX;

        if (imageY < minY)
            minY = imageY;

        if (imageY > maxY)
            maxY = imageY;

        // ----------------------------------------------------
        // Check 8 neighbouring pixels
        // ----------------------------------------------------

        for (int8_t dy = -1; dy <= 1; ++dy)
        {
            for (int8_t dx = -1; dx <= 1; ++dx)
            {
                if (dx == 0 && dy == 0)
                    continue;

                const int16_t neighbourX =
                    static_cast<int16_t>(gridX) + dx;

                const int16_t neighbourY =
                    static_cast<int16_t>(gridY) + dy;

                if (
                    neighbourX < 0 ||
                    neighbourY < 0 ||
                    neighbourX >= sampleWidth ||
                    neighbourY >= sampleHeight)
                {
                    continue;
                }

                const uint16_t neighbourIndex =
                    neighbourY * sampleWidth +
                    neighbourX;

                if (
                    colorMap[neighbourIndex] ==
                    static_cast<uint8_t>(color))
                {
                    // Mark visited immediately so it is
                    // never added to the queue twice.

                    colorMap[neighbourIndex] =
                        static_cast<uint8_t>(
                            ColorType::NONE);

                    if (queueWrite < MAX_SAMPLES)
                    {
                        queue[queueWrite++] =
                            neighbourIndex;
                    }
                }
            }
        }
    }

    // Ignore small regions/noise.

    if (
        sampleCount <
        minimumBlobSamples(color))
    {
        return;
    }

    Blob &best =
        blobForColor(color);

    const uint32_t estimatedArea =
        sampleCount *
        PIXEL_STEP *
        PIXEL_STEP;

    // We only want the largest connected blob
    // of each colour.

    if (
        best.found &&
        estimatedArea <= best.area)
    {
        return;
    }

    best.found = true;
    best.color = color;

    best.centerX =
        static_cast<int16_t>(
            sumX / sampleCount);

    best.centerY =
        static_cast<int16_t>(
            sumY / sampleCount);

    best.minX = minX;
    best.minY = minY;

    best.maxX = maxX;
    best.maxY = maxY;

    best.area = estimatedArea;
}

// ============================================================
// Find all connected components
// ============================================================

void Vision::findLargestBlobs(
    uint16_t sampleWidth,
    uint16_t sampleHeight)
{
    const uint32_t samples =
        static_cast<uint32_t>(sampleWidth) *
        sampleHeight;

    // Both configured ROIs reject every colour above OBSTACLE_Y_MIN (80 is
    // earlier than LINE_Y_MIN 115), and update() has already zeroed it.
    const uint32_t firstActiveSample =
        static_cast<uint32_t>(OBSTACLE_Y_MIN / PIXEL_STEP) *
        sampleWidth;

    for (uint32_t i = firstActiveSample; i < samples; ++i)
    {
        const ColorType color =
            static_cast<ColorType>(
                colorMap[i]);

        if (color == ColorType::NONE)
        {
            continue;
        }

        processComponent(
            static_cast<uint16_t>(i),
            color,
            sampleWidth,
            sampleHeight);
    }
}

// ============================================================
// Main image processing
// ============================================================

bool Vision::update(
    uint8_t *buffer,
    uint16_t width,
    uint16_t height)
{
    if (buffer == nullptr)
    {
        return false;
    }

    const uint16_t sampleWidth =
        width / PIXEL_STEP;

    const uint16_t sampleHeight =
        height / PIXEL_STEP;

    if (
        sampleWidth > MAX_SAMPLE_WIDTH ||
        sampleHeight > MAX_SAMPLE_HEIGHT)
    {
        return false;
    }

    const uint32_t startTime =
        micros();

    result.clear();

    const uint16_t firstActiveGridY =
        OBSTACLE_Y_MIN / PIXEL_STEP;
    memset(
        colorMap,
        static_cast<uint8_t>(ColorType::NONE),
        static_cast<size_t>(firstActiveGridY) * sampleWidth);

    // ========================================================
    // STEP 1
    //
    // Classify every sampled pixel.
    // ========================================================

    for (
        uint16_t gridY = firstActiveGridY;
        gridY < sampleHeight;
        ++gridY)
    {
        for (
            uint16_t gridX = 0;
            gridX < sampleWidth;
            ++gridX)
        {
            uint16_t logicalX =
                gridX * PIXEL_STEP;

            uint16_t logicalY =
                gridY * PIXEL_STEP;

            // Camera is mounted upside down.
            // Convert our logical image coordinates
            // to physical camera coordinates.

            uint16_t sourceX = logicalX;
            uint16_t sourceY = logicalY;

            if (ROTATE_180)
            {
                sourceX =
                    width - 1 - logicalX;

                sourceY =
                    height - 1 - logicalY;
            }

            const uint32_t pixelIndex =
                static_cast<uint32_t>(sourceY) *
                    width +
                sourceX;

            ColorType color = static_cast<ColorType>(
                colorLookup[readRGB565Raw(buffer, pixelIndex)]);

            // ============================================================
            // Apply Regions of Interest
            // ============================================================

            // Red and green are obstacles.
            // Ignore them outside the obstacle ROI.

            if (
                color == ColorType::RED ||
                color == ColorType::GREEN)
            {
                if (
                    logicalY < OBSTACLE_Y_MIN ||
                    logicalY > OBSTACLE_Y_MAX)
                {
                    color = ColorType::NONE;
                }
            }

            // Orange and blue are floor lines.
            // Ignore them outside the line ROI.

            if (
                color == ColorType::ORANGE ||
                color == ColorType::BLUE)
            {
                if (
                    logicalY < LINE_Y_MIN ||
                    logicalY > LINE_Y_MAX)
                {
                    color = ColorType::NONE;
                }
            }

            const uint16_t mapIndex =
                gridY * sampleWidth +
                gridX;

            colorMap[mapIndex] =
                static_cast<uint8_t>(
                    color);
        }
    }

    // ========================================================
    // STEP 2
    //
    // Find connected regions.
    // ========================================================

    findLargestBlobs(
        sampleWidth,
        sampleHeight);

    result.processingTimeUs =
        micros() - startTime;

    return true;
}

// ============================================================
// HSV value at one logical image position
// ============================================================

HSV Vision::getHSVAt(
    const uint8_t *buffer,
    uint16_t width,
    uint16_t height,
    uint16_t x,
    uint16_t y) const
{
    HSV empty = {0, 0, 0};

    if (
        buffer == nullptr ||
        x >= width ||
        y >= height)
    {
        return empty;
    }

    uint16_t sourceX = x;
    uint16_t sourceY = y;

    if (ROTATE_180)
    {
        sourceX =
            width - 1 - x;

        sourceY =
            height - 1 - y;
    }

    const uint32_t pixelIndex =
        static_cast<uint32_t>(sourceY) *
            width +
        sourceX;

    const RGB rgb =
        readRGB565(
            buffer,
            pixelIndex);

    return rgbToHSV(rgb);
}
