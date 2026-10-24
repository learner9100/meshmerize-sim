// show: one maze, the true shortest route and what a strategy does on it
//
//   show --seed 7
//   show --seed 7 --strategy prune --all

#include <cstdio>
#include <cstring>
#include "args.h"
#include "ascii.h"
#include "batch.h"
#include "wall_follower.h"

static const char* const kOptions[] = {
    "--seed", "--size", "--loops", "--checkpoints", "--cell", "--strategy", "--all",
    "--robot", "--by-time", "--margin", "--help"
};

static void usage(){
    std::printf(
        "show: draw a generated maze and what a strategy does on it\n\n"
        "  --seed N          maze seed (default 1)\n"
        "  --size W H        cells (default 10 10, max %d)\n"
        "  --loops K         extra openings, each adds one loop (default 3)\n"
        "  --checkpoints C   random checkpoint cells (default 0)\n"
        "  --cell MM         cell size in mm (default 230)\n"
        "  --strategy NAME   play one attempt and show the trace\n"
        "  --all             score every built-in strategy on this maze\n"
        "  --robot FILE      robot timing/odometry config (key = value)\n"
        "  --by-time         let the robot pick its Actual Run route by time, not distance\n"
        "  --margin S        safety margin for the time guard (default 20)\n", MAX_GRID);
}

static const char* compass(const MazeGraph& g, int from, int to){
    const int dx = g.node(to).x_mm - g.node(from).x_mm;
    const int dy = g.node(to).y_mm - g.node(from).y_mm;
    if(dy < 0 && dx == 0) return "N";
    if(dx > 0 && dy == 0) return "E";
    if(dy > 0 && dx == 0) return "S";
    if(dx < 0 && dy == 0) return "W";
    return "?";
}

static void printRoute(const MazeGraph& g, const Path& p, int cell_mm){
    std::printf("route as driven:");
    for(int i=0;i<p.edge_count;i++){
        std::printf(" %s%d", compass(g, p.nodes[i], p.nodes[i+1]), g.edge(p.edges[i]).length_mm / cell_mm);
    }
    std::printf("   (direction + cells, %d turns)\n", p.turns);
}

static void printScore(const ScoreBreakdown& sb){
    std::printf("  dry run   %7.2f s  %s%s\n", sb.dry_s,
                sb.dry_done ? "reached End" : "FAILED",
                sb.guaranteed ? ", map proven complete" : "");
    if(sb.end_found_s >= 0) std::printf("  End seen  %7.2f s into the dry run\n", sb.end_found_s);
    std::printf("  actual    %7.2f s  %s, route %d mm vs true %d mm%s\n", sb.run_s,
                sb.run_done ? "reached End" : "FAILED", sb.path_mm, sb.true_mm,
                sb.is_shortest ? " (shortest)" : "");
    std::printf("  map       %d nodes, %d edges, drove %.2f m, status %d\n",
                sb.known_nodes, sb.known_edges, sb.walked_mm / 1000.0, (int)sb.status);
    std::printf("  points    A %.0f + B %.0f + C %.2f + S %.0f + L %.0f + T %.2f - P %.0f = %.2f\n",
                sb.A, sb.B, sb.C, sb.S, sb.L, sb.T, sb.P, sb.total);
}

int main(int argc, char** argv){
    Args args(argc, argv);
    if(args.has("--help")){ usage(); return 0; }
    if(!args.onlyKnown(kOptions, sizeof kOptions / sizeof kOptions[0])){ usage(); return 2; }

    const unsigned int seed = (unsigned int)args.intAfter("--seed", 1);
    const int w = args.intAfter("--size", 10, 1);
    const int h = args.intAfter("--size", 10, 2);
    const int loops = args.intAfter("--loops", 3);
    const int cps = args.intAfter("--checkpoints", 0);
    const int cell = args.intAfter("--cell", 230);
    const char* strat_name = args.strAfter("--strategy", nullptr);
    const int margin_s = args.intAfter("--margin", 20);

    AttemptSetup setup;
    const char* robot_file = args.strAfter("--robot", nullptr);
    if(robot_file){
        const int rc = loadRobotConfig(robot_file, setup.robot);
        if(rc == -1){ std::fprintf(stderr, "cannot open %s\n", robot_file); return 2; }
        if(rc > 0){ std::fprintf(stderr, "%s: bad line %d\n", robot_file, rc); return 2; }
        if(rc < -1){ std::fprintf(stderr, "%s: speeds must be positive\n", robot_file); return 2; }
    }
    setup.metric.by_time = args.has("--by-time");
    setup.metric.model = setup.robot.run;

    static GridMaze maze;
    static MazeGraph truth;
    int start, end;
    if(!generateGrid(maze, w, h, loops, seed)){
        std::fprintf(stderr, "cannot generate a %dx%d maze with %d extra openings\n", w, h, loops);
        return 1;
    }
    if(!placeCheckpoints(maze, cps, seed)){
        std::fprintf(stderr, "cannot place %d checkpoints in %dx%d\n", cps, w, h);
        return 1;
    }
    if(!buildGraph(maze, cell, truth, start, end)){
        std::fprintf(stderr, "maze has more nodes than MAX_NODES=%d allows\n", MAX_NODES);
        return 1;
    }

    static PathFinder finder;
    static Path by_dist, by_time;
    finder.find(truth, start, end, Metric{}, by_dist);
    Metric timed;
    timed.by_time = true;
    timed.model = setup.robot.run;
    finder.find(truth, start, end, timed, by_time);

    std::printf("maze seed %u, %dx%d cells of %d mm, %d loop(s), %d checkpoint(s)\n",
                seed, w, h, cell, loops, cps);
    std::printf("%d nodes, %d edges, %d passages\n", truth.nodeCount(), truth.edgeCount(), maze.passageCount());
    std::printf("true shortest: %d mm, %d turns, %.2f s at Actual Run pace\n",
                by_dist.total_mm, by_dist.turns, pathTimeMs(truth, by_dist, setup.robot.run) / 1000.0);
    if(by_time.total_mm != by_dist.total_mm){
        std::printf("fastest route differs: %d mm, %d turns, %.2f s  <- choosing it would forfeit S and T\n",
                    by_time.total_mm, by_time.turns, by_time.cost / 1000.0);
    } else {
        std::printf("fastest route is the same as the shortest one\n");
    }
    std::printf("\n");

    Overlay o;
    markPath(truth, by_dist, cell, o, '*');
    printMaze(maze, &o);
    std::printf("S start, E end, C checkpoint, * shortest route\n");
    printRoute(truth, by_dist, cell);

    Strategy strats[MAX_STRATEGIES];
    int nstrat = builtinStrategies(strats, setup.robot.dry, setup.rules.dry_limit_s * 1000, margin_s * 1000);
    strats[nstrat++] = Strategy{"wall-left", "left-hand wall following (examples/wall_follower.h)", ExploreConfig{}, wallFollowerExplore};

    if(strat_name){
        const Strategy* st = findStrategy(strats, nstrat, strat_name);
        if(!st){
            std::fprintf(stderr, "no strategy called '%s'. Choices:", strat_name);
            for(int i=0;i<nstrat;i++) std::fprintf(stderr, " %s", strats[i].name);
            std::fprintf(stderr, "\n");
            return 2;
        }

        // replay the dry run here too so we can draw the trace
        static MazeGraph known;
        static Explorer explorer;
        TimedWalker dry(truth, start, setup.robot.dry);
        ExploreConfig cfg = st->cfg;
        if(setup.robot.use_odometry){
            dry.useOdometry(setup.robot.odom);
            cfg.trust_ids = false;
            cfg.merge_mm = setup.robot.merge_mm;
        }
        known = MazeGraph{};
        ExploreResult res = st->custom ? st->custom(dry, known) : explorer.run(dry, known, cfg);

        ScoreBreakdown sb = playStrategy(truth, start, end, *st, setup);
        std::printf("\n== %s: %s ==\n", st->name, st->blurb);
        printScore(sb);

        Overlay t;
        markTrace(truth, dry.trace(), dry.traceLen(), cell, t, '.');
        if(res.end_found && finder.find(known, res.start, res.end, setup.metric, by_time)){
            TimedWalker run(truth, start, setup.robot.run);
            int cmds[MAX_NODES];
            const int n = pathToCommands(known, by_time, cmds, MAX_NODES);
            for(int i=0;i<n;i++) if(run.go(cmds[i]) < 0) break;
            for(int i=0;i+1<run.traceLen();i++) markCorridor(truth, run.trace()[i], run.trace()[i+1], cell, t, '*', false);
        }
        std::printf("\n");
        printMaze(maze, &t);
        std::printf(". explored in the dry run, * route retraced in the actual run\n");
        if(res.unexplored_exits > 0) std::printf("%d exit(s) were left untried\n", res.unexplored_exits);
    }

    if(args.has("--all")){
        std::printf("\n== every strategy on this maze ==\n");
        std::printf("%-14s %8s %7s %7s %5s %5s %7s %s\n", "strategy", "total", "dry s", "run s", "short", "sure", "drove m", "note");
        for(int i=0;i<nstrat;i++){
            ScoreBreakdown sb = playStrategy(truth, start, end, strats[i], setup);
            std::printf("%-14s %8.2f %7.2f %7.2f %5s %5s %7.2f %s\n", strats[i].name, sb.total, sb.dry_s, sb.run_s,
                        sb.is_shortest ? "yes" : "no", sb.guaranteed ? "yes" : "no", sb.walked_mm / 1000.0,
                        sb.dry_done ? "" : "dry run failed");
        }
    }
    return 0;
}
