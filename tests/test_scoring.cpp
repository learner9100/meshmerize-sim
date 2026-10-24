#include <cstdio>
#include <cmath>
#include "scoring.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

static bool near(double a, double b){ return std::fabs(a - b) < 1e-6; }

int main() {
    AttemptSetup setup;   // placeholder robot: dry 500 mm/s + 300 ms, run 900 mm/s + 250 ms
    ExploreConfig first;
    first.stop_at_end = true;
    ExploreConfig full;

    {
        // one corridor, 1000 mm
        MazeGraph g;
        int s = g.addNode(NodeType::Start);
        int e = g.addNode(NodeType::End);
        g.addEdge(s, e, 1000);

        ScoreBreakdown sb = playAttempt(g, s, e, full, setup);
        check("corridor: dry 2.3 s, done", sb.dry_done && near(sb.dry_s, 2.3));
        check("corridor: B 30, L 5, C 177.7", near(sb.B, 30) && near(sb.L, 5) && near(sb.C, 177.7));
        check("corridor: run 1.361 s (1111 + 250 ms)", sb.run_done && near(sb.run_s, 1.361));
        check("corridor: shortest, S 30, T 148.639", sb.is_shortest && near(sb.S, 30) && near(sb.T, 148.639));
        check("corridor: total 391.339", near(sb.total, 391.339));
        check("corridor: proven complete", sb.guaranteed && sb.status == ExploreStatus::Ok);
        check("corridor: no checkpoints, no penalty", sb.A == 0 && sb.P == 0 && sb.checkpoints == 0);
    }

    // S-E 1000 first, then S-A 100, A-E 100. A is a checkpoint.
    MazeGraph g;
    int s = g.addNode(NodeType::Start);
    int e = g.addNode(NodeType::End);
    int a = g.addNode(NodeType::CheckPoint);
    g.addEdge(s, e, 1000);
    g.addEdge(s, a, 100);
    g.addEdge(a, e, 100);

    {
        ScoreBreakdown sb = playAttempt(g, s, e, first, setup);
        check("first-path: dry 2.3 s", sb.dry_done && near(sb.dry_s, 2.3));
        check("first-path: retraces 1000 mm fine but it is not the shortest",
              sb.run_done && !sb.is_shortest && sb.path_mm == 1000 && sb.true_mm == 200);
        check("first-path: S and T are lost", sb.S == 0 && sb.T == 0);
        check("first-path: misses the checkpoint", sb.checkpoints == 0 && sb.A == 0);
        check("first-path: total 30 + 177.7 + 5 = 212.7", near(sb.total, 212.7));
        check("first-path: not guaranteed", !sb.guaranteed);
    }
    {
        ScoreBreakdown sb = playAttempt(g, s, e, full, setup);
        check("full: dry 4.3 s", sb.dry_done && near(sb.dry_s, 4.3));
        check("full: run 0.722 s on the 200 mm route", sb.is_shortest && near(sb.run_s, 0.722) && sb.path_mm == 200);
        check("full: checkpoint A counted once, 25 points", sb.checkpoints == 1 && near(sb.A, 25));
        check("full: total 25 + 30 + 175.7 + 30 + 5 + 149.278 = 414.978", near(sb.total, 414.978));
        check("full: guaranteed", sb.guaranteed);
        check("full: 3 nodes, 3 edges known", sb.known_nodes == 3 && sb.known_edges == 3);
    }
    {
        AttemptSetup tight = setup;
        tight.rules.dry_limit_s = 3;      // the 4.3 s dry run overruns by 1.3 s
        ScoreBreakdown sb = playAttempt(g, s, e, full, tight);
        check("overrun: C clamps at 0", sb.C == 0 && sb.dry_done);
        check("overrun: run still fits in 150 - 1.3", sb.run_done && sb.is_shortest);
        check("overrun: total 25 + 30 + 0 + 30 + 5 + 149.278", near(sb.total, 239.278));

        tight.rules.dry_limit_s = 180;
        tight.rules.run_limit_s = 0;      // nothing fits: run fails, S and T gone
        sb = playAttempt(g, s, e, full, tight);
        check("run over limit: run_done false, S 0, T 0", !sb.run_done && sb.S == 0 && sb.T == 0);
        check("run over limit: dry points untouched", near(sb.B + sb.C + sb.L, 30 + 175.7 + 5));
    }
    {
        AttemptSetup pen = setup;
        pen.penalties = 10;
        ScoreBreakdown sb = playAttempt(g, s, e, full, pen);
        check("penalty: subtracted from total", near(sb.P, 10) && near(sb.total, 404.978));
    }
    {
        AttemptSetup timed = setup;
        timed.metric.by_time = true;
        timed.metric.model = setup.robot.run;
        ScoreBreakdown sb = playAttempt(g, s, e, full, timed);
        check("time metric: same answer when turns do not matter", sb.is_shortest && sb.path_mm == 200);
    }
    {
        // End cut off from Start: nothing can be scored
        MazeGraph h;
        int hs = h.addNode(NodeType::Start);
        int he = h.addNode(NodeType::End);
        ScoreBreakdown sb = playAttempt(h, hs, he, full, setup);
        check("unreachable End: everything stays 0", sb.total == 0 && !sb.dry_done && !sb.run_done);
    }
    {
        // Start is a dead end leading to a dead end: End exists but is not connected
        MazeGraph h;
        int hs = h.addNode(NodeType::Start);
        int hd = h.addNode(NodeType::DeadEnd);
        int he = h.addNode(NodeType::End);
        h.addEdge(hs, hd, 300);
        ScoreBreakdown sb = playAttempt(h, hs, he, full, setup);
        check("End elsewhere: no path on truth, score 0", sb.total == 0 && sb.true_mm == 0);
    }

    return failures;
}
