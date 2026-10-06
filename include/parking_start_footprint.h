#pragma once

#include <math.h>

// Physical chassis, centre protrusions and four wheels, in the same rear-axle
// frame as simulation/parking_exit_swept_search.py. This intentionally avoids
// the empty corners of a single bounding box next to the pink parking pieces.
namespace parking_start_footprint {
struct Point { float x, y; };
struct Quad { Point p[4]; };

inline Quad rectangle(float x, float y, float hx, float hy, float angle) {
    Quad q{};
    const float c = cosf(angle), s = sinf(angle);
    const float sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
    for (int i = 0; i < 4; ++i)
        q.p[i] = {x + c*sx[i]*hx - s*sy[i]*hy,
                  y + s*sx[i]*hx + c*sy[i]*hy};
    return q;
}
inline bool overlaps(const Quad &a, const Quad &b) {
    for (int side = 0; side < 2; ++side) {
        const Quad &q = side ? b : a;
        for (int i = 0; i < 4; ++i) {
            const Point axis = {q.p[i].y-q.p[(i+1)%4].y,
                                q.p[(i+1)%4].x-q.p[i].x};
            float amin=INFINITY, amax=-INFINITY, bmin=INFINITY, bmax=-INFINITY;
            for (int j=0; j<4; ++j) {
                const float av=a.p[j].x*axis.x+a.p[j].y*axis.y;
                const float bv=b.p[j].x*axis.x+b.p[j].y*axis.y;
                amin=fminf(amin,av); amax=fmaxf(amax,av);
                bmin=fminf(bmin,bv); bmax=fmaxf(bmax,bv);
            }
            if (amax < bmin || bmax < amin) return false;
        }
    }
    return true;
}

// Shared physical polygons for parking checks and passing-side verification.
// wheelSign uses firmware steering sign; +/-1 checks both steering extremes.
inline void robotQuads(float x, float y, float headingDeg, int wheelSign,
                       float bodyMargin, Quad (&bodies)[6]) {
    constexpr float rad=0.017453292519943295f;
    const float h=headingDeg*rad, c=cosf(h), s=sinf(h);
    const float left=wheelSign<0 ? -37.545f : (wheelSign>0 ? 62.455f : 0);
    const float right=wheelSign<0 ? -62.455f : (wheelSign>0 ? 37.545f : 0);
    const Quad local[6] = {
        rectangle(37.5f,0,72.5f+bodyMargin,50+bodyMargin,0),
        rectangle(42.5f,0,82.5f+bodyMargin,25+bodyMargin,0),
        rectangle(0,50,21.6f+bodyMargin,12.5f+bodyMargin,0),
        rectangle(0,-50,21.6f+bodyMargin,12.5f+bodyMargin,0),
        rectangle(100,50,21.6f+bodyMargin,12.5f+bodyMargin,left*rad),
        rectangle(100,-50,21.6f+bodyMargin,12.5f+bodyMargin,right*rad)
    };
    for (int i=0;i<6;++i) {
        bodies[i]=local[i];
        for (Point &p : bodies[i].p) {
            const float lx=p.x, ly=p.y;
            p={x+c*lx-s*ly,y+s*lx+c*ly};
        }
    }
}

// Fixed field coordinates; parking piece is x=480..500, y=-1500..-1300.
// Piece faces include 5 mm reserve. Gap is checked at both measured extremes.
inline bool safe(float x, float y, float headingDeg, int wheelSign,
                 bool keepBehindNearSeat, float bodyMargin=0.0f,
                 bool keepAheadMiddleSeat=false) {
    if (!isfinite(x) || !isfinite(y) || !isfinite(headingDeg)) return false;
    Quad bodies[6];
    robotQuads(x,y,headingDeg,wheelSign,bodyMargin,bodies);
    const Quad fixed=rectangle(490,-1400,15,100,0);
    const Quad moved[2] = {rectangle(480-242.5f-10,-1400,15,100,0),
                          rectangle(480-252.5f-10,-1400,15,100,0)};
    for (Quad &q : bodies) {
        for (Point &p : q.p) {
            if (p.y <= -1495 || (keepBehindNearSeat && p.x >= 500) ||
                (keepAheadMiddleSeat && p.x <= 0)) return false;
        }
        if (overlaps(q,fixed) || overlaps(q,moved[0]) || overlaps(q,moved[1]))
            return false;
    }
    return true;
}
} // namespace parking_start_footprint
