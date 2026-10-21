#pragma once
#include "maze_graph.h"

constexpr int INF_MM=0x3FFFFFFF;

struct Path{
    bool found = false;
    int total_mm = 0;
    int node_count = 0;
    int nodes[MAX_NODES]={};
    int edge_count = 0;
    int edges[MAX_NODES]={};
};

inline bool shortestPath(const MazeGraph& g, int start, int goal, Path& out){
    out = Path{};
    const int n = g.nodeCount();

    if (start < 0 || start >= n || goal < 0 || goal >= n) return false;

    int dist[MAX_NODES];
    int prev_node[MAX_NODES];
    int prev_edge[MAX_NODES];
    bool done[MAX_NODES];

    for(int i = 0;i < n;i++){
        dist[i] = INF_MM;
        prev_node[i] = -1;
        prev_edge[i] = -1;
        done[i] = false;

    }
    dist[start] = 0;

    for(int round =0; round < n; round++){
        int u = -1;
        for(int i=0; i<n; i++){
            if (!done[i] && dist[i] < INF_MM && (u == -1 || dist[i] < dist[u])) {
                u = i;
            }
        }
        if (u == -1) break;
        done[u] = true;
        if(u == goal) break;

        const Node& nd = g.node(u);
        for(int k=0;k<nd.exit_count;k++){
            const int ei=nd.exit_edge[k];
            const int v = g.otherEnd(ei, u);
            const int candidate = dist[u] + g.edge(ei).length_mm;
            if(!done[v] && candidate < dist[v]){
                dist[v] = candidate;
                prev_node[v] = u;
                prev_edge[v] = ei;
            }
        }
    }

    if(!done[goal]) return false;

    int count = 1;
    for(int v = goal; v!=start; v = prev_node[v]) count++;

    out.node_count = count;
    out.edge_count = count - 1;

    int v = goal;
    for(int i = count - 1; i >= 0; i--){
        out.nodes[i] = v;
        if(i > 0) out.edges[i-1] = prev_edge[v];
        v = prev_node[v];
    }
    out.total_mm = dist[goal];
    out.found = true;
    return true;
}