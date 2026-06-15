#include "camera.h"
#include "arducam_dvp.h"
#include "GC2145/gc2145.h"

// Camera hardware instance
static GC2145 galaxyCore;
static Camera cam(galaxyCore);
static FrameBuffer fb;
static CameraResults latest_results;

// Processing constants
constexpr int WIDTH = 320;
constexpr int HEIGHT = 240;
constexpr int COLOR_THRESHOLD = 40; // Sensitivity for color detection

void camera_setup() {
    Serial.println("Initializing GC2145 Camera...");
    
    // Using 320x240 RGB565. This resolution covers the full 80 deg DFOV 
    // by downsampling the sensor array rather than cropping.
    if (!cam.begin(CAMERA_R320x240, CAMERA_RGB565, 30)) {
        Serial.println("CRITICAL ERROR: Camera initialization failed!");
        return;
    }

    // Optional: Mirror or flip if the camera is mounted upside down
    // cam.setHorizontalMirror(true);
    // cam.setVerticalFlip(true);

    Serial.println("Camera initialized successfully (Full DFOV active).");
}

void camera_update() {
    if (cam.grabFrame(fb, 1000) != 0) {
        return; // Failed to grab frame
    }

    uint16_t* buffer = (uint16_t*)fb.getBuffer();
    
    // Reset results for this frame
    latest_results.red_block = {false, 0, 0, 0};
    latest_results.green_block = {false, 0, 0, 0};

    long red_x_sum = 0, red_y_sum = 0, red_count = 0;
    long green_x_sum = 0, green_y_sum = 0, green_count = 0;

    // Simple one-pass scan for color blobs
    // Note: RGB565 is RRRRRGGGGGGBBBBB
    for (int y = 0; y < HEIGHT; y += 2) { // Skip lines for performance
        for (int x = 0; x < WIDTH; x += 2) {
            uint16_t pixel = buffer[y * WIDTH + x];
            
            // Extract components (0-255 scale for easier math)
            int r = ((pixel >> 11) & 0x1F) << 3;
            int g = ((pixel >> 5) & 0x3F) << 2;
            int b = (pixel & 0x1F) << 3;

            // Detection Logic
            // Red: High Red, low Green and Blue
            if (r > (g + COLOR_THRESHOLD) && r > (b + COLOR_THRESHOLD)) {
                red_x_sum += x;
                red_y_sum += y;
                red_count++;
            }
            // Green: High Green, low Red and Blue
            else if (g > (r + COLOR_THRESHOLD) && g > (b + COLOR_THRESHOLD)) {
                green_x_sum += x;
                green_y_sum += y;
                green_count++;
            }
        }
    }

    // Update Red Result
    if (red_count > 20) { // Minimum pixel count to ignore noise
        latest_results.red_block.found = true;
        latest_results.red_block.x = red_x_sum / red_count;
        latest_results.red_block.y = red_y_sum / red_count;
        latest_results.red_block.size = red_count;
    }

    // Update Green Result
    if (green_count > 20) {
        latest_results.green_block.found = true;
        latest_results.green_block.x = green_x_sum / green_count;
        latest_results.green_block.y = green_y_sum / green_count;
        latest_results.green_block.size = green_count;
    }
}

CameraResults get_camera_results() {
    return latest_results;
}