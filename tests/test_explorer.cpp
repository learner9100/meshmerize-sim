#include <cstdio>
#include "solver.h"
#include "explorer.h"
#include "walker.h"

static int failures=0;

static void check(const char* name, bool ok){
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if(!ok) failures++;
}

// every go() fails
struct BrokenWalker{
    int here() const { return 0; }
    NodeType type() const { return NodeType::Start; }
    int exitCount() const { return 1; }
    int backExit() const { return -1; }
    int go(int){ return -1; }
    int totalMm() const { return 0; }
    int xMm() const { return 0; }
    int yMm() const { return 0; }
};

int main(){
    // loop maze: S-A, A-B, B-S form a ring, B-E leaves it, all 100 mm
    MazeGraph truth;
    int s = truth.addNode(NodeType::Start);
    int a = truth.addNode(NodeType::Normal);
    int b = truth.addNode(NodeType::Normal);
    int e = truth.addNode(NodeType::End);
    truth.addEdge(s, a, 100);
    truth.addEdge(a, b, 100);
    truth.addEdge(b, s, 100);
    truth.addEdge(b, e, 100);

    {
        // S>A>B>S closes the ring, then S>B>E
        MazeGraph known;
        Walker w(truth, s);
        ExploreResult r = explore(w, known, false);
        check("loop full: status ok and End found", r.status == ExploreStatus::Ok && r.end_found);
        check("loop full: 4 nodes, 4 edges", known.nodeCount() == 4 && known.edgeCount() == 4);
        check("loop full: walked 500 mm", r.walked_mm == 500);
        check("loop full: 5 moves", r.moves == 5);
        check("loop full: finishes standing on End", r.at_end && w.here() == e);
        check("loop full: nothing left unexplored", r.complete && r.unexplored_exits == 0);
        check("loop full: estimate 5 x (200+300) ms", r.elapsed_ms == 2500);

        Path p;
        shortestPath(known, r.start, r.end, p);
        check("loop full: shortest known is 200", p.found && p.total_mm == 200);

        int cmds[MAX_NODES];
        check("loop full: path gives 2 commands", pathToCommands(known, p, cmds, MAX_NODES) == 2);
        Walker replay(truth, s);
        replay.go(cmds[0]);
        replay.go(cmds[1]);
        check("loop full: replaying commands reaches End in 200 mm",
              replay.here() == e && replay.totalMm() == 200);
    }

    {
        MazeGraph known;
        Walker w(truth, s);
        ExploreResult r = explore(w, known, true);
        check("loop early stop: found End", r.end_found);
        check("loop early stop: stopped on End", r.at_end && w.here() == e);
        check("loop early stop: End was the last place, so map is complete", r.complete);
    }

    {
        // S-J 100, J-D 50 (dead end), J-E 100
        MazeGraph t2;
        int s2 = t2.addNode(NodeType::Start);
        int j2 = t2.addNode(NodeType::Normal);
        int d2 = t2.addNode(NodeType::DeadEnd);
        int e2 = t2.addNode(NodeType::End);
        t2.addEdge(s2, j2, 100);
        t2.addEdge(j2, d2, 50);
        t2.addEdge(j2, e2, 100);

        MazeGraph known;
        Walker w(t2, s2);
        ExploreResult r = explore(w, known, false);
        check("dead end: 4 nodes, 3 edges", known.nodeCount() == 4 && known.edgeCount() == 3);
        check("dead end: walked 100+50+50+100", r.walked_mm == 300);
        check("dead end: at End", r.at_end && w.here() == e2);
        Path p;
        shortestPath(known, r.start, r.end, p);
        check("dead end: shortest is 200", p.found && p.total_mm == 200);
    }

    // S-E 1000 is tried first, S-A 100 + A-E 100 is the real shortest
    MazeGraph t3;
    int s3 = t3.addNode(NodeType::Start);
    int e3 = t3.addNode(NodeType::End);
    int a3 = t3.addNode(NodeType::Normal);
    t3.addEdge(s3, e3, 1000);
    t3.addEdge(s3, a3, 100);
    t3.addEdge(a3, e3, 100);

    {
        MazeGraph known;
        Walker w(t3, s3);
        ExploreResult r = explore(w, known, true);
        Path p;
        shortestPath(known, r.start, r.end, p);
        check("first-path: stops after 1000 mm", r.walked_mm == 1000 && r.at_end);
        check("first-path: believes 1000 is shortest", p.total_mm == 1000);
        check("first-path: knows it left two exits untried (S-A and A-E)", !r.complete && r.unexplored_exits == 2);
    }
    {
        MazeGraph known;
        Walker w(t3, s3);
        ExploreResult r = explore(w, known, false);
        Path p;
        shortestPath(known, r.start, r.end, p);
        check("full: finds the 200 mm route", p.total_mm == 200);
        check("full: walked 1000+100+100 then 200 back to End", r.walked_mm == 1400 && r.at_end);
    }

    {
        // same maze plus a 5 m dead end at S
        MazeGraph t4 = t3;
        int d4 = t4.addNode(NodeType::DeadEnd);
        t4.addEdge(s3, d4, 5000);

        ExploreConfig cfg;
        cfg.limit_ms = 6000;
        cfg.margin_ms = 1000;
        MazeGraph known;
        Walker w(t4, s3);
        ExploreResult r = explore(w, known, cfg);
        check("guard: skipped the dead end", r.walked_mm == 1400 && known.nodeCount() == 3);
        check("guard: still ends at End", r.at_end && w.here() == e3);
        check("guard: admits the map is not complete", !r.complete && r.unexplored_exits == 1);
        check("guard: estimated 4300 ms, under the 6000 limit", r.elapsed_ms == 4300);

        cfg.limit_ms = 0;
        MazeGraph known2;
        Walker w2(t4, s3);
        ExploreResult r2 = explore(w2, known2, cfg);
        check("no guard: drives the dead end both ways", r2.walked_mm == 11400 && r2.complete);
    }

    {
        // S(0,0), E(1000,0) joined by a 2000 mm detour. A(0,1000) off S, B(0,2000) off A.
        MazeGraph t5;
        int s5 = t5.addNode(NodeType::Start, 0, 0);
        int e5 = t5.addNode(NodeType::End, 1000, 0);
        int a5 = t5.addNode(NodeType::Normal, 0, 1000);
        int b5 = t5.addNode(NodeType::DeadEnd, 0, 2000);
        t5.addEdge(s5, e5, 2000);
        t5.addEdge(s5, a5, 1000);
        t5.addEdge(a5, b5, 1000);

        ExploreConfig cfg;
        cfg.prune = true;
        MazeGraph known;
        Walker w(t5, s5);
        ExploreResult r = explore(w, known, cfg);
        // bound at S: 0 + 1000 < 2000, worth a look; at A: 1000 + 1414 >= 2000, not
        check("prune: visits A but not B", known.nodeCount() == 3);
        check("prune: calls the map complete anyway", r.complete && r.unexplored_exits == 1);
        check("prune: walked 2000+2000+1000 then 3000 back", r.walked_mm == 8000 && r.at_end);

        cfg.prune = false;
        MazeGraph known2;
        Walker w2(t5, s5);
        ExploreResult r2 = explore(w2, known2, cfg);
        check("no prune: visits B as well", known2.nodeCount() == 4 && r2.complete);

        // move E so the corridor already equals the straight line
        t5.setPos(e5, 2000, 0);
        cfg.prune = true;
        MazeGraph known3;
        Walker w3(t5, s5);
        ExploreResult r3 = explore(w3, known3, cfg);
        check("prune: corridor already matches the straight line, explores nothing else",
              known3.nodeCount() == 2 && r3.walked_mm == 2000 && r3.complete);
    }

    {
        // ring S-A-B-S (100 each), S-E 1000 is End, B-D 3000 a dead end
        MazeGraph t6;
        int s6 = t6.addNode(NodeType::Start);
        int a6 = t6.addNode(NodeType::Normal);
        int b6 = t6.addNode(NodeType::Normal);
        int e6 = t6.addNode(NodeType::End);
        int d6 = t6.addNode(NodeType::DeadEnd);
        t6.addEdge(s6, a6, 100);
        t6.addEdge(s6, e6, 1000);
        t6.addEdge(a6, b6, 100);
        t6.addEdge(b6, s6, 100);
        t6.addEdge(b6, d6, 3000);

        ExploreConfig deep;
        ExploreConfig near;
        near.order = Frontier::Nearest;
        MazeGraph k1, k2;
        Walker w1(t6, s6), w2(t6, s6);
        ExploreResult r1 = explore(w1, k1, deep);
        ExploreResult r2 = explore(w2, k2, near);
        check("orders: both map 5 nodes, 5 edges",
              k1.nodeCount() == 5 && k1.edgeCount() == 5 && k2.nodeCount() == 5 && k2.edgeCount() == 5);
        check("orders: both finish at End, complete", r1.at_end && r2.at_end && r1.complete && r2.complete);
        check("orders: deepest drives 300 + 100 + 3000 + 3100 + 1000 = 7500", r1.walked_mm == 7500);
        check("orders: nearest drives 300 + 1000 + 1100 + 3000 + 4100 = 9500", r2.walked_mm == 9500);
    }

    {
        MazeGraph known;
        for(int i=0;i<MAX_NODES-1;i++) known.addNode(NodeType::Normal);
        Walker w(truth, s);
        ExploreResult r = explore(w, known, false);
        check("graph full: status says GraphFull", r.status == ExploreStatus::GraphFull);
        check("graph full: not reported complete or at End", !r.complete && !r.at_end);
    }

    {
        MazeGraph known;
        BrokenWalker bw;
        ExploreResult r = explore(bw, known, false);
        check("broken walker: status says WalkerError", r.status == ExploreStatus::WalkerError);
        check("broken walker: no End claimed", !r.end_found && !r.at_end);
    }

    return failures;
}
