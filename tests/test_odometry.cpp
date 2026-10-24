#include <cstdio>
#include "scoring.h"
#include "maze_gen.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

int main() {
    ExploreConfig full;
    full.order = Frontier::Nearest;

    {
        // perfect odometry must behave exactly like trusted ids
        bool same = true;
        for(unsigned int seed = 1; seed <= 15; seed++){
            GridMaze m;
            MazeGraph truth;
            int s, e;
            generateGrid(m, 8, 8, 3, seed);
            buildGraph(m, 230, truth, s, e);

            AttemptSetup ids;
            AttemptSetup odo;
            odo.robot.use_odometry = true;
            odo.robot.merge_mm = 100;
            ScoreBreakdown a = playAttempt(truth, s, e, full, ids);
            ScoreBreakdown b = playAttempt(truth, s, e, full, odo);
            if(a.walked_mm != b.walked_mm || a.total != b.total || a.known_nodes != b.known_nodes
               || !b.is_shortest || b.status != ExploreStatus::Ok) same = false;
        }
        check("perfect odometry: identical to trusted ids on 15 mazes", same);
    }

    {
        // realistic-ish drift: 2% scale error, 1% noise, 230 mm cells, merge at 100 mm
        bool all_good = true;
        int exact_maps = 0;
        for(unsigned int seed = 1; seed <= 20; seed++){
            GridMaze m;
            MazeGraph truth;
            int s, e;
            generateGrid(m, 8, 8, 3, seed);
            buildGraph(m, 230, truth, s, e);

            AttemptSetup odo;
            odo.robot.use_odometry = true;
            odo.robot.merge_mm = 100;
            odo.robot.odom.scale_err = 0.02;
            odo.robot.odom.noise = 0.01;
            odo.robot.odom.seed = seed;
            ScoreBreakdown sb = playAttempt(truth, s, e, full, odo);
            if(sb.status != ExploreStatus::Ok || !sb.is_shortest) all_good = false;
            if(sb.known_nodes == truth.nodeCount()) exact_maps++;
        }
        check("small drift: shortest route found on all 20 mazes", all_good);
        check("small drift: the map matched the maze node for node every time", exact_maps == 20);
    }

    {
        // heavy drift, tight merge radius: missed loop closures or confused places
        int clean = 0, duplicates = 0, lost = 0, wrong_route = 0;
        for(unsigned int seed = 1; seed <= 20; seed++){
            GridMaze m;
            MazeGraph truth;
            int s, e;
            generateGrid(m, 8, 8, 3, seed);
            buildGraph(m, 230, truth, s, e);

            AttemptSetup odo;
            odo.robot.use_odometry = true;
            odo.robot.merge_mm = 60;
            odo.robot.odom.scale_err = 0.25;
            odo.robot.odom.noise = 0.05;
            odo.robot.odom.seed = seed;
            ScoreBreakdown sb = playAttempt(truth, s, e, full, odo);
            if(sb.status != ExploreStatus::Ok) lost++;
            else if(sb.known_nodes != truth.nodeCount()) duplicates++;
            else if(!sb.is_shortest) wrong_route++;
            else clean++;
            if(sb.is_shortest && !sb.run_done) wrong_route = 999;   // cannot happen
        }
        std::printf("  (heavy drift over 20 mazes: clean %d, duplicated nodes %d, lost %d, wrong route %d)\n",
                    clean, duplicates, lost, wrong_route);
        check("heavy drift: the damage is visible in the results", clean < 20);
        check("heavy drift: 'shortest' is only ever claimed after a real retrace", wrong_route != 999);
    }

    {
        // two junctions closer than the merge radius: the robot thinks it never left A
        MazeGraph g;
        int s = g.addNode(NodeType::Start, 0, 0);
        int a = g.addNode(NodeType::Normal, 1000, 0);
        int b = g.addNode(NodeType::Normal, 1050, 0);
        int e = g.addNode(NodeType::End, 2050, 0);
        g.addEdge(s, a, 1000);
        g.addEdge(a, b, 50);
        g.addEdge(b, e, 1000);

        AttemptSetup odo;
        odo.robot.use_odometry = true;
        odo.robot.merge_mm = 100;
        ScoreBreakdown sb = playAttempt(g, s, e, full, odo);
        check("false merge: reported as LostPosition", sb.status == ExploreStatus::LostPosition);
        check("false merge: no Dry Run credit, score 0", !sb.dry_done && sb.total == 0);

        odo.robot.merge_mm = 40;
        sb = playAttempt(g, s, e, full, odo);
        check("tighter merge radius: the same maze works", sb.status == ExploreStatus::Ok && sb.is_shortest);
    }

    return failures;
}
