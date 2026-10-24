#include <cstdio>
#include "solver.h"

static int failures=0;

static void check(const char* name, bool ok){
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if(!ok) failures++;
}

static bool pathIsConsistent(const MazeGraph& g, const Path& p){
    if(p.edge_count != p.node_count - 1) return false;
    for(int i=0;i<p.edge_count;i++){
        if(g.otherEnd(p.edges[i], p.nodes[i]) != p.nodes[i+1]) return false;
    }
    return true;

}

int main(){
    //1. simple chain
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);
        int j=g.addNode(NodeType::Normal);
        int e=g.addNode(NodeType::End);
        g.addEdge(s, j, 300);
        g.addEdge(j, e, 450);

        Path p;
        bool ok = shortestPath(g, s, e, p);
        check("Chain found", ok && p.found);
        check("chain: total_mm", p.total_mm == 750);
        check("chain: nodes 0,1,2", p.node_count == 3 && p.nodes[0] == 0 && p.nodes[1] == 1 && p.nodes[2] == 2);
        check("chain: path is consistent", pathIsConsistent(g, p));
    }

    //2. Fewer edges is not always shorter
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);
        int a=g.addNode(NodeType::Normal);
        int b=g.addNode(NodeType::Normal);
        int e=g.addNode(NodeType::End);
        g.addEdge(s, e, 1000);
        g.addEdge(s, a, 100);
        g.addEdge(a, b, 200);
        g.addEdge(b, e, 600);
        Path p;
        shortestPath(g, s, e, p);
        check("Fewer edges: total_mm", p.total_mm == 900);
        check("weights: takes 4 node", p.node_count == 4);
        check("Fewer edges: path is consistent", pathIsConsistent(g, p));
    }

    //2. loop must not confuse algo
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);
        int a=g.addNode(NodeType::Normal);
        int b=g.addNode(NodeType::Normal);
        int e=g.addNode(NodeType::End);
        g.addEdge(s, a, 100);
        g.addEdge(a, b, 100);
        g.addEdge(b, s, 100);
        g.addEdge(b, e, 100);

        Path p;
        shortestPath(g, s, e, p);
        check("Loop: total_mm", p.total_mm == 200);
        check("Loop: goes S->B->E", p.node_count == 3 && p.nodes[1]==b);
        check("Loop: path is consistent", pathIsConsistent(g, p));
    }

    //4. Goal is not reachable
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);
        int e=g.addNode(NodeType::End);

        Path p;
        check("Unreachable: returns false", !shortestPath(g, s, e, p));
        check("Unreachable: path not found", !p.found);
    }

    //5. Start equals goal
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);

        Path p;
        bool ok = shortestPath(g, s, s, p);
        check("Start equals goal: total_mm==0", ok && p.total_mm == 0);
        check("Start equals goal: one node no edges", p.node_count == 1 && p.edge_count == 0);
    }

    //6. Bad ids
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);

        Path p;
        check("Bad ids: returns false", !shortestPath(g, s, 999, p));
        check("Negative start: returns false", !shortestPath(g, -1, s, p));
    }

    //7. Equal length: prefer fewer turns
    {
        // S(0,0) to E(2000,2000). Route A: via X(2000,0), one corner.
        // Route B: via Y(0,1000), Z(2000,1000), two corners. Both 4000 mm.
        MazeGraph g;
        int s=g.addNode(NodeType::Start, 0, 0);
        int e=g.addNode(NodeType::End, 2000, 2000);
        int y=g.addNode(NodeType::Normal, 0, 1000);
        int z=g.addNode(NodeType::Normal, 2000, 1000);
        int x=g.addNode(NodeType::Normal, 2000, 0);
        g.addEdge(s, y, 1000);
        g.addEdge(y, z, 2000);
        g.addEdge(z, e, 1000);
        g.addEdge(s, x, 2000);
        g.addEdge(x, e, 2000);

        Path p;
        shortestPath(g, s, e, p);
        check("Tie: total is 4000", p.total_mm == 4000);
        check("Tie: picks the one-corner route via X", p.node_count == 3 && p.nodes[1] == x);
        check("Tie: reports 1 turn", p.turns == 1);
        check("Tie: path is consistent", pathIsConsistent(g, p));
    }

    //8. By time, turns cost; by distance they do not
    {
        // S(0,0), E(3000,0). A: one meandering corridor of 3300 mm.
        // B: via P(0,-100), Q(3000,-100): 100 + 3000 + 100 = 3200 mm, two corners.
        MazeGraph g;
        int s=g.addNode(NodeType::Start, 0, 0);
        int e=g.addNode(NodeType::End, 3000, 0);
        int pp=g.addNode(NodeType::Normal, 0, -100);
        int q=g.addNode(NodeType::Normal, 3000, -100);
        g.addEdge(s, e, 3300);
        g.addEdge(s, pp, 100);
        g.addEdge(pp, q, 3000);
        g.addEdge(q, e, 100);

        Path p;
        shortestPath(g, s, e, p);
        check("Distance: picks 3200 route with 2 turns", p.total_mm == 3200 && p.turns == 2);

        Metric m;
        m.by_time = true;
        m.model.mm_per_s = 500;
        m.model.node_ms = 300;
        m.model.turn_ms = 400;
        shortestPath(g, s, e, m, p);
        check("Time: picks the straight 3300 route", p.total_mm == 3300 && p.turns == 0);
        check("Time: cost is 6600 + 300 ms", p.cost == 6900);
        check("Time: pathTimeMs agrees with cost", pathTimeMs(g, p, m.model) == p.cost);

        m.model.node_ms = 0;
        m.model.turn_ms = 0;
        shortestPath(g, s, e, m, p);
        check("Time without turn cost: back to 3200 route", p.total_mm == 3200 && p.cost == 6400);
    }

    //9. Commands for the robot
    {
        MazeGraph g;
        int s=g.addNode(NodeType::Start);
        int j=g.addNode(NodeType::Normal);
        int e=g.addNode(NodeType::End);
        g.addEdge(s, j, 300, 2, 0);
        g.addEdge(j, e, 450, 1, 0);
        Path p;
        shortestPath(g, s, e, p);
        int cmds[MAX_NODES];
        int n = pathToCommands(g, p, cmds, MAX_NODES);
        check("Commands: one per edge", n == 2);
        check("Commands: taken from the departure side", cmds[0] == 2 && cmds[1] == 1);
        check("Commands: too small buffer is refused", pathToCommands(g, p, cmds, 1) == -1);

        MazeGraph h;
        int hs=h.addNode(NodeType::Start);
        int he=h.addNode(NodeType::End);
        h.addEdge(hs, he, 100);
        shortestPath(h, hs, he, p);
        check("Commands: unknown exit_cmd gives -1", pathToCommands(h, p, cmds, MAX_NODES) == -1);
    }

    return failures;
}