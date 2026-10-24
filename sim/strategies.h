#pragma once
#include <cstring>
#include "explorer.h"
#include "robot_model.h"
#include "scoring.h"

// Anything that can drive a Dry Run. Your own explorer goes behind this.
typedef ExploreResult (*ExploreFn)(TimedWalker& w, MazeGraph& known);

// A named exploration policy: either a configuration of the built-in
// Explorer, or a custom function. Friends' teams add theirs to a list and
// the compare tool does the rest.
struct Strategy{
    const char* name;
    const char* blurb;
    ExploreConfig cfg;
    ExploreFn custom = nullptr;
};

constexpr int MAX_STRATEGIES = 16;

inline ScoreBreakdown playStrategy(const MazeGraph& truth, int start, int end,
                                   const Strategy& s, const AttemptSetup& setup){
    if(s.custom) return playAttemptWith(truth, start, end, setup, s.custom);
    return playAttempt(truth, start, end, s.cfg, setup);
}

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

    {   Strategy s{"prune-grid", "prune with the Manhattan bound: only valid for 90-degree mazes", base};
        s.cfg.order = Frontier::Nearest;
        s.cfg.prune = true;
        s.cfg.manhattan = true;
        out[n++] = s; }

    {   Strategy s{"prune-guard", "prune-grid, and head for End when the clock gets tight", base};
        s.cfg.order = Frontier::Nearest;
        s.cfg.prune = true;
        s.cfg.manhattan = true;
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
