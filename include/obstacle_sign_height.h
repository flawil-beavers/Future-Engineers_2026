#pragma once

#include <math.h>
#include "config.h"
#include "vision.h"

// Height uses the measured ground plane and vertical vanishing point. Never
// derive range from an assumed 100mm object in the surprise-rule path.
namespace obstacle_sign_height {
enum Class : uint8_t { UNKNOWN=0, SHORT=50, NORMAL=100, TALL=150 };

inline float estimate(const Blob &b, bool componentTop=false) {
    if (!b.found || b.minX<=2 || b.maxX>=317 || b.maxY>=237) return NAN;
    const float top=componentTop ? b.minY:b.heightTopY;
    if ((!componentTop && !b.heightTopMeasured) || top<0 || top>=b.maxY)
        return NAN;
    const float foot=b.maxY-OBSTACLE_CAMERA_FOOT_EDGE_SLOPE*
        fabsf(b.centerX-OBSTACLE_CAMERA_PRINCIPAL_X_PX);
    const float horizon=OBSTACLE_CAMERA_GROUND_HORIZON_Y;
    const float f=OBSTACLE_CAMERA_FOCAL_Y_PX;
    const float tangent=(OBSTACLE_CAMERA_PRINCIPAL_Y_PX-horizon)/f;
    const float cameraHeight=OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/
        (f*(1+tangent*tangent));
    if (foot-horizon<20) return NAN;
    const float perspective=(1+tangent*tangent)/
        (1+tangent*(OBSTACLE_CAMERA_PRINCIPAL_Y_PX-top)/f);
    return cameraHeight*(foot-top)/(foot-horizon)*perspective;
}

inline bool basePlausible(const Blob &b) {
    const float denominator=b.maxY-OBSTACLE_CAMERA_GROUND_HORIZON_Y;
    if (denominator<20 || b.width()<7) return false;
    const int width=b.heightBaseWidthPx>0 ? b.heightBaseWidthPx:b.width();
    const float widthMm=width*OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/
        (denominator*OBSTACLE_CAMERA_FOCAL_X_PX);
    // A square base presents50..70.7mm depending on the view; allow image
    // sampling/ground-plane error, but not a room-wide strip or a thin line.
    return widthMm>=30 && widthMm<=95;
}

inline Class classify(const Blob &b, bool componentTop=false) {
    if (!basePlausible(b)) return UNKNOWN;
    const float mm=estimate(b,componentTop);
    if (!isfinite(mm)) return UNKNOWN;
    if (!componentTop && b.heightTopClipped)
        return mm>=132 && mm<=165 ? TALL:UNKNOWN;
    if (mm>=32 && mm<=68) return SHORT;
    if (mm>=80 && mm<=120) return NORMAL;
    if (mm>=132 && mm<=178) return TALL;
    return UNKNOWN; // ambiguous bands must obtain a better view
}

inline bool shortShape(const Blob &b) {
    if (!OBSTACLE_SURPRISE_ENABLED || !b.found ||
        (b.color!=ColorType::RED && b.color!=ColorType::GREEN) ||
        b.minY<=80 || b.maxY<OBSTACLE_MIN_BOTTOM_Y || b.maxY>=237 ||
        b.centerX<OBSTACLE_START_MIN_X || b.centerX>OBSTACLE_START_MAX_X ||
        b.width()>OBSTACLE_MAX_START_WIDTH || b.height()<11 ||
        b.area<(b.color==ColorType::RED ? 150U:200U) ||
        b.area<0.35f*b.width()*b.height()) return false;
    return classify(b,true)==SHORT;
}

inline bool measuredShape(const Blob &b) {
    if(!OBSTACLE_SURPRISE_ENABLED || !b.heightTopMeasured || !b.found ||
       (b.color!=ColorType::RED && b.color!=ColorType::GREEN) ||
       b.centerX<OBSTACLE_START_MIN_X || b.centerX>OBSTACLE_START_MAX_X ||
       b.maxY<OBSTACLE_MIN_BOTTOM_Y || b.width()>OBSTACLE_MAX_START_WIDTH ||
       b.height()<11 || b.area<.35f*b.width()*b.height())return false;
    const auto height=classify(b);
    const uint32_t area=height==SHORT ? (b.color==ColorType::RED ? 150U:200U):
        (b.color==ColorType::RED ? OBSTACLE_RED_MIN_AREA:OBSTACLE_GREEN_MIN_AREA);
    return height!=UNKNOWN && b.area>=area;
}
} // namespace obstacle_sign_height
