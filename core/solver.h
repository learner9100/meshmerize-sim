#pragma once
#include "maze_graph.h"
#include "cost.h"

constexpr int INF_MM=0x3FFFFFFF;

// What "shortest" means. Distance is the default and is what we assume the
// judges check. by_time asks "which route would the robot drive fastest"
// with the given speed and turn costs.
struct Metric{
    bool by_time = false;
    CostModel model;
};

struct Path{
    bool found = false;
    int total_mm = 0;
    int cost = 0;        // mm, or ms when found with Metric::by_time
    int turns = 0;       // corners and u-turns along the way
    int node_count = 0;
    int nodes[MAX_NODES]={};
    int edge_count = 0;
    int edges[MAX_NODES]={};
};

// Dijkstra over (node, exit we arrived through) so a turn can be priced by
// looking at where we came from. Ties on cost are broken by fewer turns.
// About 17 KB of scratch: on the ESP32 keep one of these static.
class PathFinder{
    private:
        static constexpr int SLOTS = MAX_EXITS + 1;   // +1 for "arrived from nowhere"
        static constexpr int STATES = MAX_NODES * SLOTS;

        int cost_[STATES];
        int turns_[STATES];
        int prev_[STATES];
        bool done_[STATES];

        static int stateOf(int node, int via_slot){ return node * SLOTS + via_slot + 1; }

    public:
        bool find(const MazeGraph& g, int start, int goal, const Metric& m, Path& out){
            out = Path{};
            const int n = g.nodeCount();
            if (start < 0 || start >= n || goal < 0 || goal >= n) return false;

            const int total = n * SLOTS;
            for(int i=0;i<total;i++){
                cost_[i] = INF_MM;
                turns_[i] = 0;
                prev_[i] = -1;
                done_[i] = false;
            }
            const int s0 = stateOf(start, -1);
            cost_[s0] = 0;

            int goal_state = -1;
            while(true){
                int u = -1;
                for(int i=0;i<total;i++){
                    if(done_[i] || cost_[i] >= INF_MM) continue;
                    if(u == -1 || cost_[i] < cost_[u] ||
                       (cost_[i] == cost_[u] && turns_[i] < turns_[u])) u = i;
                }
                if(u == -1) break;
                done_[u] = true;

                const int here = u / SLOTS;
                const int via = u % SLOTS - 1;
                if(here == goal){ goal_state = u; break; }

                const Node& nd = g.node(here);
                int in_dx = 0, in_dy = 0;
                if(via >= 0){
                    const Node& from = g.node(g.otherEnd(nd.exit_edge[via], here));
                    in_dx = nd.x_mm - from.x_mm;
                    in_dy = nd.y_mm - from.y_mm;
                }

                for(int k=0;k<nd.exit_count;k++){
                    const int ei = nd.exit_edge[k];
                    const int v = g.otherEnd(ei, here);
                    const Node& nv = g.node(v);
                    const Turn t = turnKind(in_dx, in_dy, nv.x_mm - nd.x_mm, nv.y_mm - nd.y_mm);
                    const int len = g.edge(ei).length_mm;
                    const int step = m.by_time ? moveCostMs(m.model, len, t) : len;

                    const int sv = stateOf(v, g.exitSlot(v, ei));
                    if(done_[sv]) continue;
                    const int nc = cost_[u] + step;
                    const int nt = turns_[u] + (t == Turn::Straight ? 0 : 1);
                    if(nc < cost_[sv] || (nc == cost_[sv] && nt < turns_[sv])){
                        cost_[sv] = nc;
                        turns_[sv] = nt;
                        prev_[sv] = u;
                    }
                }
            }

            if(goal_state < 0) return false;

            int count = 1;
            for(int s = goal_state; s != s0; s = prev_[s]) count++;
            if(count > MAX_NODES) return false;

            out.node_count = count;
            out.edge_count = count - 1;
            int s = goal_state;
            for(int i = count - 1; i >= 0; i--){
                const int node = s / SLOTS;
                const int via = s % SLOTS - 1;
                out.nodes[i] = node;
                if(i > 0){
                    out.edges[i-1] = g.node(node).exit_edge[via];
                    out.total_mm += g.edge(out.edges[i-1]).length_mm;
                }
                s = prev_[s];
            }
            out.cost = cost_[goal_state];
            out.turns = turns_[goal_state];
            out.found = true;
            return true;
        }
};

inline bool shortestPath(const MazeGraph& g, int start, int goal, const Metric& m, Path& out){
    PathFinder pf;
    return pf.find(g, start, goal, m, out);
}

inline bool shortestPath(const MazeGraph& g, int start, int goal, Path& out){
    return shortestPath(g, start, goal, Metric{}, out);
}

// How long the robot needs to drive `p`, turns included.
inline int pathTimeMs(const MazeGraph& g, const Path& p, const CostModel& m){
    int ms = 0;
    int in_dx = 0, in_dy = 0;
    for(int i=0;i<p.edge_count;i++){
        const Node& a = g.node(p.nodes[i]);
        const Node& b = g.node(p.nodes[i+1]);
        const int out_dx = b.x_mm - a.x_mm;
        const int out_dy = b.y_mm - a.y_mm;
        ms += moveCostMs(m, g.edge(p.edges[i]).length_mm, turnKind(in_dx, in_dy, out_dx, out_dy));
        in_dx = out_dx;
        in_dy = out_dy;
    }
    return ms;
}

// Turn a path into the go() arguments the robot must issue, one per edge.
// Returns the count, or -1 if an edge has no recorded command.
inline int pathToCommands(const MazeGraph& g, const Path& p, int* cmds, int max_cmds){
    if(p.edge_count > max_cmds) return -1;
    for(int i=0;i<p.edge_count;i++){
        const int slot = g.exitSlot(p.nodes[i], p.edges[i]);
        if(slot < 0) return -1;
        const int cmd = g.node(p.nodes[i]).exit_cmd[slot];
        if(cmd < 0) return -1;
        cmds[i] = cmd;
    }
    return p.edge_count;
}
