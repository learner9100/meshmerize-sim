#pragma once
#include "explorer.h"
#include "solver.h"
#include "timed_walker.h"
#include "robot_model.h"

// Total = A + B + C + S + L + T - P, from the Techfest page.
// Anything the page leaves open is marked "assumption" below.
struct Rules{
    int dry_limit_s = 180;
    int run_limit_s = 150;
    int checkpoint_pts = 25;
    int dry_bonus = 30;
    int shortest_bonus = 30;
    int led_pts = 5;
};

struct AttemptSetup{
    RobotConfig robot;
    Rules rules;
    Metric metric;              // how the robot picks its Actual Run route
    double penalties = 0;       // P, still undefined by the organisers
};

struct ScoreBreakdown{
    double A = 0, B = 0, C = 0, S = 0, L = 0, T = 0, P = 0;
    double total = 0;

    double dry_s = 0;
    double run_s = 0;
    double end_found_s = -1;
    bool dry_done = false;      // physically on End with a map
    bool run_done = false;      // retraced to End inside the limit
    bool is_shortest = false;   // retraced route as short as the true optimum
    bool guaranteed = false;    // explorer proved nothing shorter was left
    int checkpoints = 0;
    int walked_mm = 0;
    int path_mm = 0;
    int true_mm = 0;
    int path_turns = 0;
    int known_nodes = 0;
    int known_edges = 0;
    ExploreStatus status = ExploreStatus::Ok;
};

// Dry Run (explore, finish at End), then Actual Run (replay the chosen route
// on the real maze with a fresh clock). explore_fn drives the Dry Run.
template<class Fn>
ScoreBreakdown playAttemptWith(const MazeGraph& truth, int start, int end,
                               const AttemptSetup& setup, Fn&& explore_fn){
    static MazeGraph known;
    static PathFinder finder;
    static Path path;

    ScoreBreakdown sb;
    const Rules& r = setup.rules;

    if(!finder.find(truth, start, end, Metric{}, path)) return sb;
    sb.true_mm = path.total_mm;

    // Dry Run
    known = MazeGraph{};
    TimedWalker dry(truth, start, setup.robot.dry);
    if(setup.robot.use_odometry) dry.useOdometry(setup.robot.odom);

    const ExploreResult res = explore_fn(dry, known);
    sb.status = res.status;
    sb.walked_mm = dry.totalMm();
    sb.known_nodes = known.nodeCount();
    sb.known_edges = known.edgeCount();
    sb.dry_s = dry.elapsedMs() / 1000.0;
    sb.end_found_s = res.end_found_ms < 0 ? -1 : res.end_found_ms / 1000.0;
    sb.guaranteed = res.complete;
    sb.dry_done = res.status == ExploreStatus::Ok && res.end_found && dry.type() == NodeType::End;

    // assumption: a Dry Run that never reaches End earns neither B, C nor L
    if(sb.dry_done){
        sb.B = r.dry_bonus;
        sb.L = r.led_pts;
        sb.C = sb.dry_s < r.dry_limit_s ? r.dry_limit_s - sb.dry_s : 0;
    }
    // assumption: Dry Run overrun is taken off the Actual Run limit
    const double overrun = sb.dry_s > r.dry_limit_s ? sb.dry_s - r.dry_limit_s : 0;
    const double run_limit = r.run_limit_s - overrun;

    TimedWalker run(truth, start, setup.robot.run);
    if(sb.dry_done && finder.find(known, res.start, res.end, setup.metric, path)){
        sb.path_mm = path.total_mm;
        sb.path_turns = path.turns;
        int cmds[MAX_NODES];
        const int n = pathToCommands(known, path, cmds, MAX_NODES);
        bool ok = n >= 0;
        for(int i=0; ok && i<n; i++) ok = run.go(cmds[i]) >= 0;
        sb.run_s = run.elapsedMs() / 1000.0;
        sb.run_done = ok && run.type() == NodeType::End && sb.run_s <= run_limit;
        // assumption: judges compare length, any route of optimal length counts
        sb.is_shortest = sb.run_done && run.totalMm() == sb.true_mm;
        if(sb.is_shortest){
            sb.S = r.shortest_bonus;
            sb.T = r.run_limit_s - sb.run_s;
        }
    }

    for(int i=0; i<truth.nodeCount(); i++){
        if(truth.node(i).type != NodeType::CheckPoint) continue;
        if(dry.visited(i) || run.visited(i)) sb.checkpoints++;
    }
    sb.A = (double)r.checkpoint_pts * sb.checkpoints;
    sb.P = setup.penalties;
    sb.total = sb.A + sb.B + sb.C + sb.S + sb.L + sb.T - sb.P;
    return sb;
}

inline ScoreBreakdown playAttempt(const MazeGraph& truth, int start, int end,
                                  const ExploreConfig& cfg, const AttemptSetup& setup){
    static Explorer explorer;
    ExploreConfig ecfg = cfg;
    if(setup.robot.use_odometry){
        ecfg.trust_ids = false;
        ecfg.merge_mm = setup.robot.merge_mm;
    }
    return playAttemptWith(truth, start, end, setup,
        [&](TimedWalker& w, MazeGraph& known){ return explorer.run(w, known, ecfg); });
}
