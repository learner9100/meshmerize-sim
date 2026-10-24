#pragma once
#include <cstring>
#include "explorer.h"
#include "robot_model.h"

// A named exploration policy. Friends' teams add theirs to builtinStrategies
// (or build their own list) and the compare tool does the rest.
struct Strategy{
    const char* name;
    const char* blurb;
    ExploreConfig cfg;
};

constexpr int MAX_STRATEGIES = 16;

inline int builtinStrategies(Strategy* out, const CostModel& belief, int limit_ms, int margin_ms){
    int n = 0;
    ExploreConfig base;
    base.model = belief;

    {   Strategy s{"first-path", "stop the moment End is seen (depth-first)", base};
        s.cfg.stop_at_end = true;
        out[n++] = s; }

    {   Strategy s{"dfs-full", "map everything, plain depth-first backtracking", base};
        out[n++] = s; }

    {   Strategy s{"nearest-full", "map everything, always go to the closest unexplored exit", base};
        s.cfg.order = Frontier::Nearest;
        out[n++] = s; }

    {   Strategy s{"prune", "nearest-first, skip branches that cannot beat the best route", base};
        s.cfg.order = Frontier::Nearest;
        s.cfg.prune = true;
        out[n++] = s; }

    {   Strategy s{"prune-guard", "prune, and head for End when the clock gets tight", base};
        s.cfg.order = Frontier::Nearest;
        s.cfg.prune = true;
        s.cfg.limit_ms = limit_ms;
        s.cfg.margin_ms = margin_ms;
        out[n++] = s; }

    {   Strategy s{"nearest-guard", "map everything unless the clock gets tight", base};
        s.cfg.order = Frontier::Nearest;
        s.cfg.limit_ms = limit_ms;
        s.cfg.margin_ms = margin_ms;
        out[n++] = s; }

    return n;
}

inline const Strategy* findStrategy(const Strategy* list, int n, const char* name){
    for(int i=0;i<n;i++){
        if(!std::strcmp(list[i].name, name)) return &list[i];
    }
    return nullptr;
}
