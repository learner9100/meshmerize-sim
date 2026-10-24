#pragma once
#include <cstdio>
#include <cmath>
#include "maze_gen.h"
#include "scoring.h"
#include "strategies.h"

struct BatchParams{
    int first_seed = 1;
    int seeds = 500;
    int width = 10;
    int height = 10;
    int loops = 3;            // extra openings per maze
    int checkpoints = 0;
    int cell_mm = 230;
    AttemptSetup setup;
};

struct StrategyStats{
    const char* name = "";
    int attempts = 0;
    double mean_score = 0, sd_score = 0;
    double pct_shortest = 0;      // Actual Run retraced a route of optimal length
    double pct_guaranteed = 0;    // explorer proved the route optimal before leaving
    double pct_dry_done = 0;
    double pct_dry_over = 0;      // Dry Run longer than the limit
    double mean_dry_s = 0, mean_run_s = 0;
    double mean_walked_m = 0;
    double mean_checkpoints = 0;
};

struct BatchSummary{
    int mazes = 0;
    int skipped = 0;              // generator or graph capacity said no
    double mean_nodes = 0;
    double mean_true_m = 0;
    double pct_metric_differs = 0; // time-optimal route differs from distance-optimal
};

// Runs every strategy on every seed. Deterministic: same params, same table.
// If csv is not null one row per (seed, strategy) is appended to it.
inline BatchSummary runBatch(const BatchParams& p, const Strategy* strats, int nstrat,
                             StrategyStats* out, FILE* csv = nullptr){
    static GridMaze maze;
    static MazeGraph truth;
    static PathFinder finder;
    static Path by_dist, by_time;

    struct Acc{
        double score = 0, score2 = 0, dry = 0, run = 0, walked = 0, cps = 0;
        int shortest = 0, guaranteed = 0, dry_done = 0, dry_over = 0, n = 0;
    };
    Acc acc[MAX_STRATEGIES];
    BatchSummary sum;
    double nodes_total = 0, true_total = 0;
    int differs = 0;

    if(csv) std::fprintf(csv, "seed,strategy,total,A,B,C,S,L,T,dry_s,run_s,dry_done,run_done,"
                              "shortest,guaranteed,walked_mm,path_mm,true_mm,checkpoints,status\n");

    for(int i=0; i<p.seeds; i++){
        const unsigned int seed = (unsigned int)(p.first_seed + i);
        int start, end;
        if(!generateGrid(maze, p.width, p.height, p.loops, seed) ||
           !placeCheckpoints(maze, p.checkpoints, seed) ||
           !buildGraph(maze, p.cell_mm, truth, start, end)){
            sum.skipped++;
            continue;
        }
        sum.mazes++;
        nodes_total += truth.nodeCount();

        Metric timed;
        timed.by_time = true;
        timed.model = p.setup.robot.run;
        finder.find(truth, start, end, Metric{}, by_dist);
        finder.find(truth, start, end, timed, by_time);
        true_total += by_dist.total_mm;
        if(by_time.total_mm != by_dist.total_mm) differs++;

        for(int s=0; s<nstrat; s++){
            const ScoreBreakdown sb = playStrategy(truth, start, end, strats[s], p.setup);
            Acc& a = acc[s];
            a.n++;
            a.score += sb.total;
            a.score2 += sb.total * sb.total;
            a.dry += sb.dry_s;
            a.run += sb.run_s;
            a.walked += sb.walked_mm;
            a.cps += sb.checkpoints;
            if(sb.is_shortest) a.shortest++;
            if(sb.guaranteed) a.guaranteed++;
            if(sb.dry_done) a.dry_done++;
            if(sb.dry_s > p.setup.rules.dry_limit_s) a.dry_over++;

            if(csv) std::fprintf(csv, "%u,%s,%.3f,%.0f,%.0f,%.3f,%.0f,%.0f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                seed, strats[s].name, sb.total, sb.A, sb.B, sb.C, sb.S, sb.L, sb.T,
                sb.dry_s, sb.run_s, sb.dry_done, sb.run_done, sb.is_shortest, sb.guaranteed,
                sb.walked_mm, sb.path_mm, sb.true_mm, sb.checkpoints, (int)sb.status);
        }
    }

    for(int s=0; s<nstrat; s++){
        const Acc& a = acc[s];
        StrategyStats& st = out[s];
        st = StrategyStats{};
        st.name = strats[s].name;
        st.attempts = a.n;
        if(a.n == 0) continue;
        const double n = a.n;
        st.mean_score = a.score / n;
        const double var = a.score2 / n - st.mean_score * st.mean_score;
        st.sd_score = var > 0 ? std::sqrt(var) : 0;
        st.pct_shortest = 100.0 * a.shortest / n;
        st.pct_guaranteed = 100.0 * a.guaranteed / n;
        st.pct_dry_done = 100.0 * a.dry_done / n;
        st.pct_dry_over = 100.0 * a.dry_over / n;
        st.mean_dry_s = a.dry / n;
        st.mean_run_s = a.run / n;
        st.mean_walked_m = a.walked / n / 1000.0;
        st.mean_checkpoints = a.cps / n;
    }

    if(sum.mazes > 0){
        sum.mean_nodes = nodes_total / sum.mazes;
        sum.mean_true_m = true_total / sum.mazes / 1000.0;
        sum.pct_metric_differs = 100.0 * differs / sum.mazes;
    }
    return sum;
}

inline void printStats(const StrategyStats* st, int n){
    std::printf("%-14s %8s %7s %7s %7s %7s %7s %7s %8s %6s\n",
        "strategy", "score", "sd", "short%", "sure%", "dry s", ">180%", "run s", "walked m", "cps");
    for(int i=0;i<n;i++){
        const StrategyStats& s = st[i];
        std::printf("%-14s %8.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %8.1f %6.2f\n",
            s.name, s.mean_score, s.sd_score, s.pct_shortest, s.pct_guaranteed,
            s.mean_dry_s, s.pct_dry_over, s.mean_run_s, s.mean_walked_m, s.mean_checkpoints);
    }
}
