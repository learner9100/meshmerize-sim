#pragma once
#include "maze_graph.h"

// Placeholder numbers until measured on the real car, see robot.cfg.
// Integers so the simulator and the firmware add up the same milliseconds.
struct CostModel {
    int mm_per_s = 500;
    int node_ms  = 300;
    int turn_ms  = 400;
    int uturn_ms = 1000;
};

enum class Turn { Straight, Corner, UTurn };

// zero vectors (no position data) count as straight
inline Turn turnKind(int in_dx, int in_dy, int out_dx, int out_dy){
    if((in_dx == 0 && in_dy == 0) || (out_dx == 0 && out_dy == 0)) return Turn::Straight;
    const long cross = (long)in_dx * out_dy - (long)in_dy * out_dx;
    const long dot   = (long)in_dx * out_dx + (long)in_dy * out_dy;
    if(cross != 0) return Turn::Corner;
    return dot < 0 ? Turn::UTurn : Turn::Straight;
}

inline int moveCostMs(const CostModel& m, int length_mm, Turn t){
    int ms = (int)((long)length_mm * 1000 / m.mm_per_s) + m.node_ms;
    if(t == Turn::Corner) ms += m.turn_ms;
    if(t == Turn::UTurn)  ms += m.uturn_ms;
    return ms;
}

// floor(sqrt(v)), rounding down keeps lower bounds valid
inline int isqrt(long v){
    if(v <= 0) return 0;
    long r = 1;
    while(r * r <= v) r <<= 1;
    long lo = r >> 1, hi = r;
    while(lo + 1 < hi){
        const long mid = (lo + hi) / 2;
        if(mid * mid <= v) lo = mid; else hi = mid;
    }
    return (int)lo;
}

inline int straightMm(const Node& a, const Node& b){
    const long dx = a.x_mm - b.x_mm;
    const long dy = a.y_mm - b.y_mm;
    return isqrt(dx * dx + dy * dy);
}

inline int manhattanMm(const Node& a, const Node& b){
    const int dx = a.x_mm > b.x_mm ? a.x_mm - b.x_mm : b.x_mm - a.x_mm;
    const int dy = a.y_mm > b.y_mm ? a.y_mm - b.y_mm : b.y_mm - a.y_mm;
    return dx + dy;
}
