#include <cstdio>
#include "maze_gen.h"
#include "solver.h"

static int failures=0;

static void check(const char* name, bool ok){
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if(!ok) failures++;
}

static int totalLength(const MazeGraph& g){
    int sum = 0;
    for(int i=0;i<g.edgeCount();i++) sum += g.edge(i).length_mm;
    return sum;
}

static bool sameGraph(const MazeGraph& a, const MazeGraph& b){
    if(a.nodeCount() != b.nodeCount() || a.edgeCount() != b.edgeCount()) return false;
    for(int i=0;i<a.nodeCount();i++){
        if(a.node(i).type != b.node(i).type || a.node(i).x_mm != b.node(i).x_mm || a.node(i).y_mm != b.node(i).y_mm) return false;
    }
    for(int i=0;i<a.edgeCount();i++){
        if(a.edge(i).a != b.edge(i).a || a.edge(i).b != b.edge(i).b || a.edge(i).length_mm != b.edge(i).length_mm) return false;
    }
    return true;
}

// every graph node's exits must match the degree of its cell
static bool degreesMatch(const GridMaze& m, const MazeGraph& g, int cell_mm){
    for(int i=0;i<g.nodeCount();i++){
        const Node& n = g.node(i);
        if(n.exit_count != m.degree(n.x_mm / cell_mm, n.y_mm / cell_mm)) return false;
    }
    return true;
}

int main(){
    const int cell = 230;

    {
        GridMaze m;
        m.width = 4; m.height = 1;
        m.carve(0, 0, 1);
        m.carve(1, 0, 1);
        m.carve(2, 0, 1);
        MazeGraph g;
        int s, e;
        check("corridor: builds", buildGraph(m, cell, g, s, e));
        check("corridor: 2 nodes, 1 edge", g.nodeCount() == 2 && g.edgeCount() == 1);
        check("corridor: one 690 mm edge", g.edge(0).length_mm == 690);
        check("corridor: Start at 0,0 and End at 690,0",
              g.node(s).x_mm == 0 && g.node(s).y_mm == 0 && g.node(e).x_mm == 690 && g.node(e).y_mm == 0);
        check("corridor: 3 passages", m.passageCount() == 3);
    }

    {
        GridMaze m;
        m.width = 2; m.height = 2;
        m.carve(0, 0, 1);
        m.carve(0, 0, 2);
        m.carve(1, 0, 2);
        m.carve(0, 1, 1);
        MazeGraph g;
        int s, e;
        buildGraph(m, cell, g, s, e);
        check("ring: 4 nodes, 4 edges", g.nodeCount() == 4 && g.edgeCount() == 4);
        check("ring: every node has 2 exits", degreesMatch(m, g, cell));
        Path p;
        shortestPath(g, s, e, p);
        check("ring: shortest Start to End is 460", p.found && p.total_mm == 460);
        check("ring: corner cells are not straight", !m.straightCell(0, 0) && !m.straightCell(1, 1));
    }

    {
        GridMaze m;
        check("rejects 1 wide", !generateGrid(m, 1, 5, 0, 1));
        check("rejects 1 high", !generateGrid(m, 5, 1, 0, 1));
        check("rejects wider than MAX_GRID", !generateGrid(m, MAX_GRID + 1, 5, 0, 1));
        check("rejects negative openings", !generateGrid(m, 5, 5, -1, 1));
        // 5x5 has 40 inner walls, 24 are carved, 16 remain
        check("accepts all 16 remaining walls", generateGrid(m, 5, 5, 16, 1));
        check("rejects 17 openings", !generateGrid(m, 5, 5, 17, 1));
        check("5x5 fully open has 40 passages", generateGrid(m, 5, 5, 16, 1) && m.passageCount() == 40);
    }

    {
        bool perfect_ok = true, reachable = true, degrees_ok = true, size_ok = true;
        for(unsigned int seed = 1; seed <= 40; seed++){
            GridMaze m;
            MazeGraph g;
            int s, e;
            if(!generateGrid(m, 10, 10, 0, seed) || !buildGraph(m, cell, g, s, e)){ size_ok = false; continue; }
            if(g.edgeCount() != g.nodeCount() - 1) perfect_ok = false;
            if(totalLength(g) != 99 * cell) perfect_ok = false;
            if(m.passageCount() != 99) perfect_ok = false;
            if(!degreesMatch(m, g, cell)) degrees_ok = false;
            Path p;
            if(!shortestPath(g, s, e, p)) reachable = false;
            if(g.node(s).type != NodeType::Start || g.node(e).type != NodeType::End) reachable = false;
            if(g.node(e).x_mm != 9 * cell || g.node(e).y_mm != 9 * cell) reachable = false;
        }
        check("perfect 10x10: generates and fits the graph for 40 seeds", size_ok);
        check("perfect 10x10: edges = nodes - 1 and total length = 99 cells", perfect_ok);
        check("perfect 10x10: node exits match cell degrees", degrees_ok);
        check("perfect 10x10: End at bottom-right and reachable", reachable);
    }

    {
        bool loops_ok = true;
        for(int k = 1; k <= 8; k++){
            for(unsigned int seed = 1; seed <= 10; seed++){
                GridMaze m;
                MazeGraph g;
                int s, e;
                if(!generateGrid(m, 10, 10, k, seed) || !buildGraph(m, cell, g, s, e)){ loops_ok = false; continue; }
                if(g.edgeCount() - g.nodeCount() + 1 != k) loops_ok = false;
                if(totalLength(g) != (99 + k) * cell) loops_ok = false;
            }
        }
        check("k openings: edges - nodes + 1 = k and length = (99 + k) cells", loops_ok);
    }

    {
        GridMaze m1, m2, m3;
        MazeGraph g1, g2, g3;
        int s, e;
        generateGrid(m1, 10, 10, 3, 77); buildGraph(m1, cell, g1, s, e);
        generateGrid(m2, 10, 10, 3, 77); buildGraph(m2, cell, g2, s, e);
        generateGrid(m3, 10, 10, 3, 78); buildGraph(m3, cell, g3, s, e);
        check("same seed gives the same graph", sameGraph(g1, g2));
        check("next seed gives a different graph", !sameGraph(g1, g3));
    }

    {
        GridMaze m;
        generateGrid(m, 10, 10, 2, 5);
        check("checkpoints: rejects more than cells - 2", !placeCheckpoints(m, 99, 5));
        check("checkpoints: rejects negative", !placeCheckpoints(m, -1, 5));
        check("checkpoints: accepts 5", placeCheckpoints(m, 5, 5));
        int count = 0;
        bool corner_free = true;
        for(int y=0;y<10;y++) for(int x=0;x<10;x++){
            if(!m.checkpoint[y][x]) continue;
            count++;
            if((x == 0 && y == 0) || (x == 9 && y == 9)) corner_free = false;
        }
        check("checkpoints: exactly 5 cells marked", count == 5);
        check("checkpoints: never on Start or End", corner_free);

        MazeGraph g;
        int s, e;
        buildGraph(m, cell, g, s, e);
        int cp_nodes = 0;
        for(int i=0;i<g.nodeCount();i++) if(g.node(i).type == NodeType::CheckPoint) cp_nodes++;
        check("checkpoints: every one becomes a graph node", cp_nodes == 5);
        check("checkpoints: graph still has 101 passages of length", totalLength(g) == 101 * cell);

        placeCheckpoints(m, 0, 5);
        count = 0;
        for(int y=0;y<10;y++) for(int x=0;x<10;x++) if(m.checkpoint[y][x]) count++;
        check("checkpoints: placing 0 clears old ones", count == 0);
    }

    {
        GridMaze m;
        MazeGraph g;
        int s, e;
        check("buildGraph rejects cell_mm 0", generateGrid(m, 4, 4, 0, 1) && !buildGraph(m, 0, g, s, e));
    }

    return failures;
}
