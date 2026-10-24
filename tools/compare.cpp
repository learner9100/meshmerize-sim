// compare: run every strategy over many random mazes and print the numbers.
//
//   compare
//   compare --seeds 1000 --size 12 12 --loops 6 --checkpoints 4
//   compare --robot robot.cfg --csv results.csv

#include <cstdio>
#include <cstring>
#include "args.h"
#include "batch.h"
#include "wall_follower.h"

static const char* const kOptions[] = {
    "--seeds", "--first", "--size", "--loops", "--checkpoints", "--cell", "--robot",
    "--csv", "--by-time", "--margin", "--only", "--help"
};

static void usage(){
    std::printf(
        "compare: score every strategy on many random mazes\n\n"
        "  --seeds N         number of mazes (default 500)\n"
        "  --first S         first seed (default 1)\n"
        "  --size W H        cells (default 10 10, max %d)\n"
        "  --loops K         extra openings per maze (default 3)\n"
        "  --checkpoints C   random checkpoint cells (default 0)\n"
        "  --cell MM         cell size in mm (default 230)\n"
        "  --robot FILE      robot timing/odometry config\n"
        "  --csv FILE        also write one row per attempt\n"
        "  --by-time         pick the Actual Run route by time instead of distance\n"
        "  --margin S        safety margin for the time guard (default 20)\n"
        "  --only NAME       run a single strategy\n", MAX_GRID);
}

int main(int argc, char** argv){
    Args args(argc, argv);
    if(args.has("--help")){ usage(); return 0; }
    if(!args.onlyKnown(kOptions, sizeof kOptions / sizeof kOptions[0])){ usage(); return 2; }

    BatchParams p;
    p.seeds = args.intAfter("--seeds", 500);
    p.first_seed = args.intAfter("--first", 1);
    p.width = args.intAfter("--size", 10, 1);
    p.height = args.intAfter("--size", 10, 2);
    p.loops = args.intAfter("--loops", 3);
    p.checkpoints = args.intAfter("--checkpoints", 0);
    p.cell_mm = args.intAfter("--cell", 230);
    const int margin_s = args.intAfter("--margin", 20);

    const char* robot_file = args.strAfter("--robot", nullptr);
    if(robot_file){
        const int rc = loadRobotConfig(robot_file, p.setup.robot);
        if(rc == -1){ std::fprintf(stderr, "cannot open %s\n", robot_file); return 2; }
        if(rc > 0){ std::fprintf(stderr, "%s: bad line %d\n", robot_file, rc); return 2; }
        if(rc < -1){ std::fprintf(stderr, "%s: speeds must be positive\n", robot_file); return 2; }
    }
    p.setup.metric.by_time = args.has("--by-time");
    p.setup.metric.model = p.setup.robot.run;

    Strategy all[MAX_STRATEGIES];
    int n = builtinStrategies(all, p.setup.robot.dry, p.setup.rules.dry_limit_s * 1000, margin_s * 1000);
    all[n++] = Strategy{"wall-left", "left-hand wall following (examples/wall_follower.h)", ExploreConfig{}, wallFollowerExplore};
    const char* only = args.strAfter("--only", nullptr);
    if(only){
        const Strategy* s = findStrategy(all, n, only);
        if(!s){ std::fprintf(stderr, "no strategy called '%s'\n", only); return 2; }
        all[0] = *s;
        n = 1;
    }

    FILE* csv = nullptr;
    const char* csv_file = args.strAfter("--csv", nullptr);
    if(csv_file){
        csv = std::fopen(csv_file, "w");
        if(!csv){ std::fprintf(stderr, "cannot write %s\n", csv_file); return 2; }
    }

    const RobotConfig& r = p.setup.robot;
    std::printf("mazes: seeds %d..%d, %dx%d cells of %d mm, %d loop(s), %d checkpoint(s)\n",
                p.first_seed, p.first_seed + p.seeds - 1, p.width, p.height, p.cell_mm, p.loops, p.checkpoints);
    std::printf("robot: dry %d mm/s + %d/%d/%d ms (node/turn/u-turn), run %d mm/s + %d/%d/%d ms%s\n",
                r.dry.mm_per_s, r.dry.node_ms, r.dry.turn_ms, r.dry.uturn_ms,
                r.run.mm_per_s, r.run.node_ms, r.run.turn_ms, r.run.uturn_ms,
                robot_file ? "" : "  [placeholder values, not measured]");
    if(r.use_odometry) std::printf("odometry: scale error %.3f, noise %.3f, merge radius %d mm\n",
                                   r.odom.scale_err, r.odom.noise, r.merge_mm);
    std::printf("route choice: by %s, guard margin %d s\n\n", p.setup.metric.by_time ? "time" : "distance", margin_s);

    StrategyStats stats[MAX_STRATEGIES];
    const BatchSummary sum = runBatch(p, all, n, stats, csv);
    if(csv) std::fclose(csv);

    if(sum.mazes == 0){
        std::printf("no maze could be built with these settings (%d skipped)\n", sum.skipped);
        return 1;
    }
    std::printf("%d mazes (%d skipped), %.1f nodes each, true shortest %.2f m on average\n",
                sum.mazes, sum.skipped, sum.mean_nodes, sum.mean_true_m);
    std::printf("fastest route differs from shortest route in %.1f%% of mazes\n\n", sum.pct_metric_differs);

    printStats(stats, n);
    std::printf("\nscore = mean total points, sd = spread; short%% = retraced a shortest route;\n"
                "sure%% = explorer proved it; >180%% = Dry Run over the limit; cps = checkpoints hit\n");
    if(csv_file) std::printf("per-attempt rows written to %s\n", csv_file);
    return 0;
}
