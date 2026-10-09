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

bool Vision::findGreenSeatCandidate(
    const uint8_t *buffer,
    uint16_t width,
    uint16_t height,
    int16_t expectedX,
    int16_t expectedFootY,
    GreenSeatCandidate &candidate) const
{
    candidate = GreenSeatCandidate();
    if (buffer == nullptr || width != 320 || height != 240 ||
        expectedX < OBSTACLE_START_MIN_X ||
        expectedX > OBSTACLE_START_MAX_X)
        return false;

    const auto rawAt = [&](int x, int y) -> uint16_t {
        const uint16_t sourceX = ROTATE_180 ? width - 1 - x : x;
        const uint16_t sourceY = ROTATE_180 ? height - 1 - y : y;
        return readRGB565Raw(
            buffer, static_cast<uint32_t>(sourceY) * width + sourceX);
    };
    const auto valueAt = [&](int x, int y) -> int {
        const uint16_t raw = rawAt(x, y);
        const uint8_t r5 = (raw >> 11) & 0x1F;
        const uint8_t g6 = (raw >> 5) & 0x3F;
        const uint8_t b5 = raw & 0x1F;
        const int red = (r5 << 3) | (r5 >> 2);
        const int green = (g6 << 2) | (g6 >> 4);
        const int blue = (b5 << 3) | (b5 >> 2);
        return red > green ? (red > blue ? red : blue)
                           : (green > blue ? green : blue);
    };
    const auto meanValue = [&](int centerX, int halfWidth,
                               int yStart, int yEnd) -> int {
        uint32_t sum = 0;
        uint16_t samples = 0;
        for (int y = yStart; y < yEnd; y += 2)
            for (int x = centerX - halfWidth;
                 x < centerX + halfWidth; x += 2)
            {
                if (x < 0 || x >= width)
                    continue;
                sum += valueAt(x, y);
                ++samples;
            }
        return samples == 0 ? -1 : static_cast<int>(sum / samples);
    };
    const auto flankValue = [&](int centerX, int halfWidth,
                                int yStart, int yEnd) -> int {
        const int left = meanValue(centerX - 40, halfWidth, yStart, yEnd);
        const int right = meanValue(centerX + 40, halfWidth, yStart, yEnd);
        return left > right ? left : right;
    };

    int bestFootError = 32767;
    for (int centerX = expectedX - OBSTACLE_GREEN_SEAT_SEARCH_HALF_WIDTH_PX;
         centerX <= expectedX + OBSTACLE_GREEN_SEAT_SEARCH_HALF_WIDTH_PX;
         centerX += 2)
    {
        if (centerX < OBSTACLE_START_MIN_X ||
            centerX > OBSTACLE_START_MAX_X)
            continue;

        const int middleValue = meanValue(centerX, 12, 104, 132);
        const int contrast = flankValue(centerX, 12, 104, 132) - middleValue;
        if (middleValue < 0 || contrast < OBSTACLE_GREEN_SEAT_MIN_BAND_CONTRAST)
            continue;

        // Find the physical foot from the last strongly contrasting row.
        // Unlike a horizontal green wall strip, this must continue down to
        // the projected ground position of the mapped seat.
        int footY = -1;
        for (int y = 96; y < 210; y += 2)
        {
            const int middle = meanValue(centerX, 10, y, y + 2);
            if (flankValue(centerX, 10, y, y + 2) - middle >=
                OBSTACLE_GREEN_SEAT_MIN_DARK_CONTRAST)
                footY = y;
        }
        if (footY < 100)
            continue;

        uint16_t darkSamples = 0;
        uint32_t sumX = 0;
        uint32_t sumY = 0;
        int minX = width;
        int minY = height;
        int maxX = -1;
        int maxY = -1;
        int previousDarkRow = -1;
        int largestDarkRowGap = 0;
        for (int y = 96; y <= footY; y += 2)
        {
            const int flank = flankValue(centerX, 12, y, y + 2);
            bool darkRow = false;
            for (int x = centerX - 20; x < centerX + 20; x += 2)
            {
                if (x < 0 || x >= width ||
                    flank - valueAt(x, y) <
                        OBSTACLE_GREEN_SEAT_MIN_DARK_CONTRAST)
                    continue;
                darkRow = true;
                ++darkSamples;
                sumX += x;
                sumY += y;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
            if (darkRow)
            {
                if (previousDarkRow >= 0 &&
                    y - previousDarkRow > largestDarkRowGap)
                    largestDarkRowGap = y - previousDarkRow;
                previousDarkRow = y;
            }
        }
        if (darkSamples == 0 ||
            largestDarkRowGap > OBSTACLE_GREEN_SEAT_MAX_DARK_ROW_GAP_PX)
            continue;

        Blob blob;
        blob.found = true;
        blob.color = ColorType::GREEN;
        blob.centerX = static_cast<int16_t>(sumX / darkSamples);
        blob.centerY = static_cast<int16_t>(sumY / darkSamples);
        blob.minX = minX;
        blob.minY = minY;
        blob.maxX = maxX;
        blob.maxY = maxY;
        blob.area = static_cast<uint32_t>(darkSamples) * 4U;
        if (blob.area < OBSTACLE_GREEN_MIN_AREA ||
            blob.height() < OBSTACLE_GREEN_MIN_HEIGHT ||
            blob.maxY < OBSTACLE_MIN_BOTTOM_Y ||
            blob.minY > OBSTACLE_MAX_TOP_Y ||
            blob.centerX < OBSTACLE_START_MIN_X ||
            blob.centerX > OBSTACLE_START_MAX_X ||
            blob.width() > OBSTACLE_MAX_START_WIDTH ||
            blob.height() > OBSTACLE_MAX_START_HEIGHT ||
            static_cast<float>(blob.width()) >
                static_cast<float>(blob.height()) *
                    OBSTACLE_MAX_WIDTH_HEIGHT_RATIO)
            continue;

        const int footError = abs(footY - expectedFootY);
        if (footError <= OBSTACLE_SEAT_SILHOUETTE_FOOT_TOLERANCE_PX)
            candidate.silhouetteFound = true;
        // Count existing HSV-green pixels in the lower pillar band without
        // demanding that they join the wall or one another into a blob.
        uint16_t greenSamples = 0;
        for (int y = 100; y < 140; y += 2)
            for (int x = centerX - 18; x < centerX + 18; x += 2)
            {
                if (x < 0 || x >= width)
                    continue;
                const uint16_t raw = rawAt(x, y);
                if (colorLookup[raw] == static_cast<uint8_t>(ColorType::GREEN))
                {
                    ++greenSamples;
                    continue;
                }
                const uint8_t r5 = (raw >> 11) & 31, g6 = (raw >> 5) & 63, b5 = raw & 31;
                const int r = (r5 << 3) | (r5 >> 2);
                const int g = (g6 << 2) | (g6 >> 4);
                const int b = (b5 << 3) | (b5 >> 2);
                const int low = r < b ? r : b;
                // Dominant green means hue60..180; saturation guards gray.
                greenSamples += g > r && g >= b && g >= 20 &&
                    g <= OBSTACLE_GREEN_SEAT_MAX_COLOR_VALUE &&
                    (g-low)*255 >= g*30;
            }
        if (greenSamples < OBSTACLE_GREEN_SEAT_MIN_COLOR_SAMPLES)
            continue;
        if (candidate.blob.found &&
            (footError > bestFootError ||
             (footError == bestFootError &&
              greenSamples <= candidate.greenSamples)))
            continue;
        bestFootError = footError;
        candidate.blob = blob;
        candidate.greenSamples = greenSamples;
        candidate.brightnessContrast = contrast;
    }
    return candidate.blob.found;
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
    // Invalid/empty frames must not preserve previous detections or overrun
    // the fixed map when the supplied image is shorter than the active ROI.
    result.clear();
    if (!buffer || width < PIXEL_STEP || height <= OBSTACLE_Y_MIN ||
        width > MAX_SAMPLE_WIDTH * PIXEL_STEP ||
        height > MAX_SAMPLE_HEIGHT * PIXEL_STEP ||
        width % PIXEL_STEP || height % PIXEL_STEP) return false;
    const uint16_t sampleWidth = width / PIXEL_STEP;
    const uint16_t sampleHeight = height / PIXEL_STEP;

    const uint32_t startTime =
        micros();

    uint32_t qualityValueSum = 0;
    result.minValue = 255;

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

            const uint16_t raw = readRGB565Raw(buffer, pixelIndex);
            ColorType color = static_cast<ColorType>(colorLookup[raw]);
            // Reuse an already-read pixel every 16 pixels in each direction.
            // No HSV conversion, extra image pass, automatic gain or extra log.
            if ((gridX & 7U) == 0 && (gridY & 7U) == 0) {
                const uint8_t r5 = (raw >> 11) & 31, g6 = (raw >> 5) & 63, b5 = raw & 31;
                const uint8_t r = (r5 << 3) | (r5 >> 2);
                const uint8_t g = (g6 << 2) | (g6 >> 4);
                const uint8_t b = (b5 << 3) | (b5 >> 2);
                const uint8_t v = r > g ? (r > b ? r : b) : (g > b ? g : b);
                ++result.qualitySamples;
                qualityValueSum += v;
                if (v < result.minValue) result.minValue = v;
                if (v > result.maxValue) result.maxValue = v;
                if (v < 20) ++result.darkSamples;
                if (r >= 250 && g >= 250 && b >= 250) ++result.clippedSamples;
            }

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

    result.meanValue = result.qualitySamples
        ? qualityValueSum / result.qualitySamples : 0;
    if (!result.qualitySamples) result.minValue = 0;
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
