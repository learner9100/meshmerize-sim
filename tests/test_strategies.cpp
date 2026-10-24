#include <cstdio>
#include <cstring>
#include "batch.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

int main() {
    Strategy strats[MAX_STRATEGIES];
    const int n = builtinStrategies(strats, dryRunModel(), 180000, 20000);
    check("seven built-in strategies", n == 7);

    bool unique = true;
    for(int i=0;i<n;i++)
        for(int j=i+1;j<n;j++)
            if(!std::strcmp(strats[i].name, strats[j].name)) unique = false;
    check("strategy names are unique", unique);
    check("findStrategy finds prune", findStrategy(strats, n, "prune") != nullptr
          && findStrategy(strats, n, "prune")->cfg.prune);
    check("findStrategy rejects unknown name", findStrategy(strats, n, "teleport") == nullptr);

    AttemptSetup setup;

    // if the explorer says complete, the retraced route must really be optimal
    {
        bool honest = true, full_ok = true, all_done = true, first_fine = true;
        int first_missed = 0;
        for(unsigned int seed = 1; seed <= 30; seed++){
            GridMaze m;
            MazeGraph truth;
            int s, e;
            generateGrid(m, 8, 8, 3, seed);
            buildGraph(m, 230, truth, s, e);
            for(int i=0;i<n;i++){
                ScoreBreakdown sb = playAttempt(truth, s, e, strats[i].cfg, setup);
                if(sb.status != ExploreStatus::Ok || !sb.dry_done || !sb.run_done) all_done = false;
                if(sb.guaranteed && !sb.is_shortest) honest = false;
                const bool full = !std::strcmp(strats[i].name, "dfs-full")
                               || !std::strcmp(strats[i].name, "nearest-full")
                               || !std::strcmp(strats[i].name, "prune")
                               || !std::strcmp(strats[i].name, "prune-grid");
                if(full && (!sb.guaranteed || !sb.is_shortest)) full_ok = false;
                if(!std::strcmp(strats[i].name, "first-path")){
                    if(sb.path_mm < sb.true_mm) first_fine = false;
                    if(!sb.is_shortest) first_missed++;
                }
            }
        }
        check("8x8 x30: every strategy completes both runs", all_done);
        check("8x8 x30: 'guaranteed' is never claimed for a non-shortest route", honest);
        check("8x8 x30: full, nearest, prune and prune-grid always find the shortest route", full_ok);
        check("8x8 x30: first-path never reports a route shorter than the truth", first_fine);
        check("8x8 x30: first-path misses the shortest route at least once", first_missed > 0);
    }

    // prune must not drive more than mapping everything, on the whole
    {
        long prune_mm = 0, full_mm = 0;
        bool never_more = true;
        const Strategy* pr = findStrategy(strats, n, "prune");
        const Strategy* nf = findStrategy(strats, n, "nearest-full");
        for(unsigned int seed = 1; seed <= 30; seed++){
            GridMaze m;
            MazeGraph truth;
            int s, e;
            generateGrid(m, 10, 10, 4, seed);
            buildGraph(m, 230, truth, s, e);
            ScoreBreakdown a = playAttempt(truth, s, e, pr->cfg, setup);
            ScoreBreakdown b = playAttempt(truth, s, e, nf->cfg, setup);
            prune_mm += a.walked_mm;
            full_mm += b.walked_mm;
            if(a.walked_mm > b.walked_mm) never_more = false;
        }
        check("10x10 x30: prune drives less than nearest-full in total", prune_mm < full_mm);
        std::printf("  (prune %ld mm vs nearest-full %ld mm; prune longer in some maze: %s)\n",
                    prune_mm, full_mm, never_more ? "no" : "yes");
    }

    // the guard only acts once End is known; if End came with margin to spare
    // the Dry Run must finish under the limit
    {
        ExploreConfig guard;
        guard.order = Frontier::Nearest;
        guard.model = dryRunModel();
        guard.limit_ms = 120000;
        guard.margin_ms = 20000;
        int fired = 0, late = 0;
        bool under = true, honest = true;
        for(unsigned int seed = 1; seed <= 40; seed++){
            GridMaze m;
            MazeGraph truth;
            int s, e;
            generateGrid(m, 12, 12, 4, seed);
            if(!buildGraph(m, 230, truth, s, e)) continue;
            ScoreBreakdown sb = playAttempt(truth, s, e, guard, setup);
            if(!sb.dry_done) under = false;
            if(sb.guaranteed) continue;
            if(sb.end_found_s > 100.0){ late++; continue; }
            fired++;
            if(sb.dry_s > 120.0) under = false;
            if(sb.guaranteed && !sb.is_shortest) honest = false;
        }
        check("12x12 guard at 120 s: fired with time to spare at least once", fired > 0);
        check("12x12 guard at 120 s: in those mazes the dry run stayed under 120 s", under);
        check("12x12 guard at 120 s: still honest about guarantees", honest);
        std::printf("  (guard cut exploration short in %d mazes; End itself came too late in %d)\n", fired, late);
    }

    // the batch runner is deterministic
    {
        BatchParams p;
        p.seeds = 15;
        p.width = 8;
        p.height = 8;
        p.loops = 2;
        p.checkpoints = 3;
        StrategyStats s1[MAX_STRATEGIES], s2[MAX_STRATEGIES];
        BatchSummary b1 = runBatch(p, strats, n, s1);
        BatchSummary b2 = runBatch(p, strats, n, s2);
        bool same = b1.mazes == b2.mazes && b1.skipped == b2.skipped && b1.mean_nodes == b2.mean_nodes;
        for(int i=0;i<n;i++){
            if(s1[i].attempts != s2[i].attempts || s1[i].mean_score != s2[i].mean_score
               || s1[i].sd_score != s2[i].sd_score || s1[i].pct_shortest != s2[i].pct_shortest
               || s1[i].mean_dry_s != s2[i].mean_dry_s || s1[i].mean_walked_m != s2[i].mean_walked_m) same = false;
        }
        check("batch: two runs with the same seeds give the same table", same);
        check("batch: 15 mazes, none skipped", b1.mazes == 15 && b1.skipped == 0);
        check("batch: every strategy ran 15 attempts", s1[0].attempts == 15 && s1[n-1].attempts == 15);
        bool cps = true;
        for(int i=0;i<n;i++) if(s1[i].mean_checkpoints < 0 || s1[i].mean_checkpoints > 3) cps = false;
        check("batch: mean checkpoints within 0..3", cps);

        p.width = MAX_GRID + 1;
        BatchSummary bad = runBatch(p, strats, n, s1);
        check("batch: impossible size is counted as skipped", bad.mazes == 0 && bad.skipped == 15);
    }

    return failures;
}
