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

    // positions, exit commands and slot lookup
    MazeGraph p;
    int p0 = p.addNode(NodeType::Start, 0, 0);
    int p1 = p.addNode(NodeType::End, 230, 460);
    check("node without position reads 0,0", g.node(s).x_mm == 0 && g.node(s).y_mm == 0);
    check("addNode with position stores it", p.node(p1).x_mm == 230 && p.node(p1).y_mm == 460);
    check("setPos moves a node", p.setPos(p0, 10, 20) && p.node(p0).x_mm == 10 && p.node(p0).y_mm == 20);
    check("setPos rejects unknown node", !p.setPos(7, 0, 0));
    int pe = p.addEdge(p0, p1, 500, 3, 1);
    check("exit_cmd stored per end", p.node(p0).exit_cmd[0] == 3 && p.node(p1).exit_cmd[0] == 1);
    check("plain addEdge leaves exit_cmd at -1", g.node(s).exit_cmd[0] == -1);
    check("exitSlot finds the slot", p.exitSlot(p0, pe) == 0 && p.exitSlot(p1, pe) == 0);
    check("exitSlot rejects edges not at the node", g.exitSlot(s, e1) == -1);
    check("exitSlot rejects bad node", p.exitSlot(42, pe) == -1);
    check("findNode locates End", p.findNode(NodeType::End) == p1);
    check("findNode returns -1 when absent", p.findNode(NodeType::CheckPoint) == -1);
    check("setType changes a node", p.setType(p1, NodeType::CheckPoint) && p.findNode(NodeType::CheckPoint) == p1);

    // compass ordering of exits: added W, S, E, N, expected N, E, S, W afterwards
    MazeGraph c;
    int hub2 = c.addNode(NodeType::Normal, 1000, 1000);
    int west = c.addNode(NodeType::DeadEnd, 0, 1000);
    int south = c.addNode(NodeType::DeadEnd, 1000, 2000);
    int east = c.addNode(NodeType::DeadEnd, 2000, 1000);
    int north = c.addNode(NodeType::DeadEnd, 1000, 0);
    int ew = c.addEdge(hub2, west, 1000, 10, 0);
    int es = c.addEdge(hub2, south, 1000, 11, 0);
    int ee = c.addEdge(hub2, east, 1000, 12, 0);
    int en = c.addEdge(hub2, north, 1000, 13, 0);
    check("before sorting exits follow insertion order", c.node(hub2).exit_edge[0] == ew && c.node(hub2).exit_edge[3] == en);
    c.sortExitsByCompass();
    const Node& hn = c.node(hub2);
    check("after sorting exits are N, E, S, W",
          hn.exit_edge[0] == en && hn.exit_edge[1] == ee && hn.exit_edge[2] == es && hn.exit_edge[3] == ew);
    check("exit_cmd moves together with its edge",
          hn.exit_cmd[0] == 13 && hn.exit_cmd[1] == 12 && hn.exit_cmd[2] == 11 && hn.exit_cmd[3] == 10);
    check("single-exit nodes are untouched", c.node(west).exit_edge[0] == ew && c.node(west).exit_count == 1);

    return failures;
}