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

void camera_green_seat_diagnostic(int expected_x, int expected_foot_y)
{
    if (current_mode != MODE_CAMERA_CALIBRATION || system_enabled ||
        expected_x < 30 || expected_x > 290 ||
        expected_foot_y < 105 || expected_foot_y > 205)
    {
        robot_logger.println(
            "[CAM SEAT ERROR] Use c0 with drive OFF, x30..290, foot_y105..205");
        return;
    }
    stop(false);
    struct Sample {
        uint32_t frame = 0;
        bool legacyValid = false;
        bool candidateValid = false;
        int16_t x = 0;
        int16_t foot = 0;
        uint16_t greenSamples = 0;
        int16_t contrast = 0;
        uint32_t processingUs = 0;
    } samples[10];

    for (uint8_t index = 0; index < 10; ++index)
    {
        const uint32_t started = millis();
        while (!camera.capture())
        {
            if (millis() - started > 1500)
            {
                robot_logger.println("[CAM SEAT ERROR] Fresh frame timeout");
                return;
            }
            delay(1);
        }
        if (!vision.update(camera.getBuffer(), camera.getWidth(),
                           camera.getHeight()))
        {
            robot_logger.println("[CAM SEAT ERROR] Vision update failed");
            return;
        }
        Sample &sample = samples[index];
        sample.frame = camera.getCompletedFrameCount();
        sample.legacyValid = obstacle_blob_valid_for_acquisition(
            &vision.getResult().green);
        GreenSeatCandidate candidate;
        const uint32_t startedProcessingUs = micros();
        sample.candidateValid = vision.findGreenSeatCandidate(
            camera.getBuffer(), camera.getWidth(), camera.getHeight(),
            expected_x, expected_foot_y, candidate);
        sample.processingUs = micros() - startedProcessingUs;
        if (sample.candidateValid)
        {
            sample.x = candidate.blob.centerX;
            sample.foot = candidate.blob.maxY;
            sample.greenSamples = candidate.greenSamples;
            sample.contrast = candidate.brightnessContrast;
        }
    }

    robot_logger.print("[CAM SEAT] expected_x/foot=");
    robot_logger.print(expected_x);
    robot_logger.print("/");
    robot_logger.println(expected_foot_y);
    for (uint8_t index = 0; index < 10; ++index)
    {
        const Sample &sample = samples[index];
        robot_logger.print("[CAM SEAT] sample=");
        robot_logger.print(index + 1);
        robot_logger.print(" frame=");
        robot_logger.print(sample.frame);
        robot_logger.print(" legacy=");
        robot_logger.print(sample.legacyValid ? 1 : 0);
        robot_logger.print(" candidate=");
        robot_logger.print(sample.candidateValid ? 1 : 0);
        robot_logger.print(" x/foot=");
        robot_logger.print(sample.x);
        robot_logger.print("/");
        robot_logger.print(sample.foot);
        robot_logger.print(" samples/contrast=");
        robot_logger.print(sample.greenSamples);
        robot_logger.print("/");
        robot_logger.print(sample.contrast);
        robot_logger.print(" roi_us=");
        robot_logger.println(sample.processingUs);
    }
}
