#include <cstdio>
#include "timed_walker.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

int main() {
    // S(0,0) - A(1000,0) - B(1000,1000) - E(2000,1000); dead end D(1000,2000) below B
    MazeGraph g;
    int s = g.addNode(NodeType::Start, 0, 0);
    int a = g.addNode(NodeType::Normal, 1000, 0);
    int b = g.addNode(NodeType::Normal, 1000, 1000);
    int e = g.addNode(NodeType::End, 2000, 1000);
    int d = g.addNode(NodeType::DeadEnd, 1000, 2000);
    g.addEdge(s, a, 1000);
    g.addEdge(a, b, 1000);
    g.addEdge(b, e, 1000);
    g.addEdge(b, d, 1000);

    CostModel m;
    m.mm_per_s = 500;
    m.node_ms = 300;
    m.turn_ms = 400;
    m.uturn_ms = 1000;

    TimedWalker w(g, s, m);
    check("start: clock at 0, start visited", w.elapsedMs() == 0 && w.visited(s) && w.traceLen() == 1);
    check("start: ids are trusted by default", w.here() == s && w.trueHere() == s);
    check("start: position comes from the maze", w.xMm() == 0 && w.yMm() == 0);

    check("bad exit: refused", w.go(9) == -1);
    check("bad exit: clock untouched", w.elapsedMs() == 0 && w.totalMm() == 0);

    w.go(0);
    check("S>A straight: 2000 + 300 ms", w.elapsedMs() == 2300);
    w.go(1);
    check("A>B corner: + 2700 ms", w.elapsedMs() == 5000);
    w.go(2);
    check("B>D straight on: + 2300 ms", w.elapsedMs() == 7300 && w.here() == d);
    w.go(0);
    check("D>B u-turn: + 3300 ms", w.elapsedMs() == 10600 && w.here() == b);
    w.go(1);
    check("B>E corner: + 2700 ms", w.elapsedMs() == 13300 && w.type() == NodeType::End);
    check("drove 5000 mm in 6 trace entries", w.totalMm() == 5000 && w.traceLen() == 6);
    check("every node visited", w.visited(s) && w.visited(a) && w.visited(b) && w.visited(d) && w.visited(e));
    check("trace lists the visits in order",
          w.trace()[0] == s && w.trace()[1] == a && w.trace()[2] == b && w.trace()[3] == d && w.trace()[4] == b && w.trace()[5] == e);
    check("position follows the walker", w.xMm() == 2000 && w.yMm() == 1000);
    check("visited() rejects bad ids", !w.visited(-1) && !w.visited(MAX_NODES));

    {
        OdomParams op;
        op.scale_err = 0.10;
        TimedWalker o(g, s, m);
        o.useOdometry(op);
        check("odometry: here() is no longer available", o.here() == -1 && o.trueHere() == s);
        o.go(0);
        check("odometry: 10% scale error shows 1100 mm east", o.xMm() == 1100 && o.yMm() == 0);
        o.go(1);
        check("odometry: error keeps accumulating", o.xMm() == 1100 && o.yMm() == 1100);
        check("odometry: the clock is unaffected", o.elapsedMs() == 5000);
    }

    {
        OdomParams op;
        op.noise = 0.05;
        op.seed = 9;
        TimedWalker o1(g, s, m), o2(g, s, m);
        o1.useOdometry(op);
        o2.useOdometry(op);
        o1.go(0); o1.go(1); o1.go(2);
        o2.go(0); o2.go(1); o2.go(2);
        check("odometry noise: same seed, same drift", o1.xMm() == o2.xMm() && o1.yMm() == o2.yMm());
        const int ex = 1000, ey = 2000;
        const int dx = o1.xMm() - ex, dy = o1.yMm() - ey;
        check("odometry noise: drifted but by less than 5% of 3000 mm per axis",
              (dx != 0 || dy != 0) && dx > -150 && dx < 150 && dy > -150 && dy < 150);
        op.seed = 10;
        TimedWalker o3(g, s, m);
        o3.useOdometry(op);
        o3.go(0); o3.go(1); o3.go(2);
        check("odometry noise: other seed, other drift", o3.xMm() != o1.xMm() || o3.yMm() != o1.yMm());
    }

    return failures;
}
