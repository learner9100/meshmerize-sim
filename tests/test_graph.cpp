#include <cstdio>
#include "maze_graph.h"

static int failures=0;

static void check(const char* name, bool ok){
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

int main(){
    MazeGraph g;
    int s = g.addNode(NodeType::Start);
    int j = g.addNode(NodeType::Normal);
    int e = g.addNode(NodeType::End);
    check("Ids are 0,1,2", s == 0 && j == 1 && e == 2);

    int e0 = g.addEdge(s, j, 300);
    int e1 = g.addEdge(j, e, 450);

    check("edge indices are 0,1", e0 == 0 && e1 == 1);
    check("3 nodes, 2 edges", g.nodeCount() == 3 && g.edgeCount() == 2);
    check("junction has 2 exits", g.node(j).exit_count == 2);
    check("start has 1 exit", g.node(s).exit_count == 1);
    check("otherEnd forward", g.otherEnd(e0, s) == j);
    check("otherEnd backward", g.otherEnd(e0, j) == s);
    check("otherEnd rejects non-endpoint", g.otherEnd(e0, e) == -1);
    check("rejects unknown node", g.addEdge(0, 9, 100) == -1);
    check("rejects zero-length edge", g.addEdge(s, e, 0) == -1);
    check("rejects self-loop", g.addEdge(j, j, 100) == -1);
    check("rejected edge are not counted", g.edgeCount() == 2);

    MazeGraph h;
    int hub=h.addNode(NodeType::Normal);
    int accepted = 0;

    for (int i=0; i<MAX_EXITS; i++){
        int n=h.addNode(NodeType::DeadEnd);
        if (h.addEdge(hub, n, 100) >= 0) accepted++;
    }

    int extra1 = h.addNode(NodeType::DeadEnd);
    check("hub accepted MAX_EXITS edges", accepted == MAX_EXITS);
    check("rejects extra edge beyond MAX_EXITS", h.addEdge(hub, extra1, 100) == -1);
    check("failed edge is not counted", h.edgeCount() == MAX_EXITS && h.node(extra1).exit_count==0);
    
    int extra2 = h.addNode(NodeType::DeadEnd);
    check("full hub as 2nd end is rejected", h.addEdge(extra2, hub, 100) == -1);
    check("and extra2 is left untouched", h.node(extra2).exit_count == 0);
    
    return failures;
}