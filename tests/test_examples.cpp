#include <cstdio>
#include "scoring.h"
#include "strategies.h"
#include "maze_gen.h"
#include "wall_follower.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

int main() {
    MazeGraph g;
    int s = g.addNode(NodeType::Start, 0, 0);
    int a = g.addNode(NodeType::Normal, 1000, 0);
    int e = g.addNode(NodeType::End, 1000, 1000);
    int d = g.addNode(NodeType::DeadEnd, 1000, -1000);
    g.addEdge(s, a, 1000);
    g.addEdge(a, e, 1000);
    g.addEdge(a, d, 1000);
    g.sortExitsByCompass();

    check("walker heading: north/east/south numbered 0/1/2",
          Walker(g, a).heading(0) == 0 && Walker(g, a).heading(1) == 2 && Walker(g, a).heading(2) == 3);
    check("walker heading: bad exit is -1", Walker(g, a).heading(7) == -1);

    {
        // facing east at S; at A left is north (dead end), then back, then south to E
        TimedWalker w(g, s, dryRunModel());
        MazeGraph known;
        ExploreResult r = wallFollowerExplore(w, known);
        check("wall follower: reaches End", r.end_found && r.at_end && w.type() == NodeType::End);
        check("wall follower: went left into the dead end first (4 moves)", r.moves == 4 && r.walked_mm == 4000);
        check("wall follower: map has 4 nodes, 3 edges", known.nodeCount() == 4 && known.edgeCount() == 3);
        Path p;
        shortestPath(known, r.start, r.end, p);
        check("wall follower: map still yields the 2000 mm route", p.found && p.total_mm == 2000);
    }

    {
        Strategy wall{"wall-left", "", ExploreConfig{}, wallFollowerExplore};
        AttemptSetup setup;
        int done = 0, shortest = 0;
        for(unsigned int seed = 1; seed <= 30; seed++){
            GridMaze m;
            MazeGraph truth;
            int st, en;
            generateGrid(m, 8, 8, 2, seed);
            buildGraph(m, 230, truth, st, en);
            ScoreBreakdown sb = playStrategy(truth, st, en, wall, setup);
            if(sb.dry_done && sb.run_done) done++;
            if(sb.is_shortest) shortest++;
            if(sb.guaranteed) done = -1000;   // it never proves anything
        }
        check("wall follower via playStrategy: finishes both runs on 30 mazes", done == 30);
        check("wall follower: sometimes lucky, never guaranteed", shortest > 0 && shortest < 30);
        std::printf("  (wall follower retraced a shortest route in %d of 30 mazes)\n", shortest);
    }

    return failures;
}
