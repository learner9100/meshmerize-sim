#include <cstdio>
#include "walker.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

int main() {
    // Loop maze: S-A, A-B, B-S form a loop; B-E leaves it.
    MazeGraph g;
    int s = g.addNode(NodeType::Start);
    int a = g.addNode(NodeType::Normal);
    int b = g.addNode(NodeType::Normal);
    int e = g.addNode(NodeType::End);
    g.addEdge(s, a, 100);
    g.addEdge(a, b, 100);
    g.addEdge(b, s, 100);
    g.addEdge(b, e, 100);

    Walker w(g, s);
    check("start: 2 exits", w.exitCount() == 2);
    check("start: no back exit", w.backExit() == -1);
    check("invalid exit returns -1", w.go(5) == -1 && w.go(-1) == -1);
    check("invalid exit does not move", w.here() == s && w.totalMm() == 0);

    check("go(0) returns 100", w.go(0) == 100);
    check("now at A", w.here() == a);
    check("A: back exit is 0", w.backExit() == 0);

    w.go(1);
    check("now at B with 3 exits", w.here() == b && w.exitCount() == 3);

    w.go(2);
    check("reached End", w.type() == NodeType::End && w.here() == e);
    check("total distance is 300", w.totalMm() == 300);

    return failures;
}