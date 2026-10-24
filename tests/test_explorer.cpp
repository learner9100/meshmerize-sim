#include<cstdio>
#include "solver.h"
#include "explorer.h"
#include "walker.h"

static int failures=0;

static void check(const char* name, bool ok){
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if(!ok) failures++;
}

int main(){
    //loop maze
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
        MazeGraph known;
        Walker w(truth, s);
        ExploreResult r = explore(w, known, false);
        check("loop full: finished and found end", r.end_found);
        check("loop full: four nodes 4 edges", known.nodeCount() == 4 && known.edgeCount() == 4);
        check("loop full: walked 800 mm", r.walked_mm == 800);
        check("loop full: ended back at start", w.here()==s);

        Path p;
        shortestPath(known, r.start, r.end, p);
        check("loop full: shortest is 200", p.found && p.total_mm == 200);
    }

    {
        MazeGraph known;
        Walker w(truth, s);
        ExploreResult r = explore(w, known, true);
        check("loop early stop: found End", r.end_found);
        check("loop early stop: walked 500 mm", r.walked_mm == 500);
        check("loop early stop: stopped on End", w.here() == e);
    }

    {
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
        Path p;
        shortestPath(known, r.start, r.end, p);
        check("dead end: shortest is 200", p.found && p.total_mm == 200);
    }
    return failures;
}