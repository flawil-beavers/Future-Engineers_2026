#include "camera_snapshot.h"
#include "obstacle.h"
#include "motor_control.h"
#include "mode_manager.h"
#include "logger.h"
#include <USB/PluggableUSBSerial.h>
#include <string.h>

namespace {
constexpr uint32_t imageBytes = 320UL * 240 * 2;

uint32_t crc32(const uint8_t *data, size_t count)
{
    uint32_t crc = 0xFFFFFFFFUL;
    while (count--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
    }
    return ~crc;
}

bool sendBytes(const uint8_t *data, size_t count, uint32_t started)
{
    while (count) {
        if (!_SerialUSB.connected() || millis() - started > 10000) return false;
        uint32_t sent = 0;
        // SDK's transmit API lacks const on its read-only source buffer.
        _SerialUSB.send_nb(const_cast<uint8_t *>(data), count > 256 ? 256 : count, &sent);
        data += sent;
        count -= sent;
        if (!sent) delay(1);
    }
    return true;
}
}

void camera_snapshot_export()
{
    if (current_mode != MODE_CAMERA_CALIBRATION || system_enabled) {
        robot_logger.println("[CAMSHOT ERROR] Use c0 with drive enable OFF first");
        return;
    }
    stop(false);
    const uint32_t acquisitionStart = millis();
    while (!camera.capture()) {
        if (millis() - acquisitionStart > 1000) {
            robot_logger.println("[CAMSHOT ERROR] Fresh frame timeout");
            return;
        }
        delay(1);
    }
    if (camera.getBufferSize() != imageBytes) {
        robot_logger.println("[CAMSHOT ERROR] Unexpected frame size");
        return;
    }
    const uint32_t frame = camera.getCompletedFrameCount();
    // Only this stopped diagnostic pauses DMA; normal driving is unaffected.
    // The published frame remains immutable throughout CRC and USB transfer.
    if (!camera.pauseForDiagnostic()) {
        camera.resumeAfterDiagnostic();
        robot_logger.println("[CAMSHOT ERROR] Could not pause camera");
        return;
    }
    const uint8_t *const snapshot = camera.getBuffer();
    char header[192];
    const int length = snprintf(header, sizeof(header),
        "\n[CAMSHOT] v=1 width=320 height=240 bytes=%lu crc32=%08lx frame=%lu "
        "exposure=%u msb_first=%u rotate180=%u\n",
        static_cast<unsigned long>(imageBytes),
        static_cast<unsigned long>(crc32(snapshot, imageBytes)),
        static_cast<unsigned long>(frame), camera.getExposureLines(),
        vision.rgb565MsbFirst() ? 1U : 0U, vision.rotates180() ? 1U : 0U);
    const uint32_t started = millis();
    const char footer[] = "\n[CAMSHOT END]\n";
    // Normal telemetry cannot interleave: this command runs only while stopped.
    // Raw bytes bypass the finite RAM telemetry log and use bounded USB writes.
    const bool failed = length <= 0 || static_cast<size_t>(length) >= sizeof(header) ||
        !sendBytes(reinterpret_cast<const uint8_t *>(header), length, started) ||
        !sendBytes(snapshot, imageBytes, started) ||
        !sendBytes(reinterpret_cast<const uint8_t *>(footer), sizeof(footer)-1, started);
    const bool resumed = camera.resumeAfterDiagnostic();
    if (failed)
        robot_logger.println("[CAMSHOT ERROR] Transfer disconnected or timed out");
    if (!resumed)
        robot_logger.println("[CAMSHOT ERROR] Camera restart failed");
}
