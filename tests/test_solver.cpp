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

    return failures;
}