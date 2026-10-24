#pragma once
#include "maze_graph.h"
#include "explorer.h"
#include "timed_walker.h"

// Left-hand wall following plugged into the simulator as a custom strategy.
// Prefers left, then straight, right, back. Needs no map to reach End but
// records one anyway so the Actual Run can be replayed and scored.
inline ExploreResult wallFollowerExplore(TimedWalker& w, MazeGraph& known){
    ExploreResult res;
    static int to_known[MAX_NODES];
    static bool used[MAX_NODES][MAX_EXITS];
    for(int i=0;i<MAX_NODES;i++){
        to_known[i] = -1;
        for(int j=0;j<MAX_EXITS;j++) used[i][j] = false;
    }

    int at = known.addNode(w.type(), w.xMm(), w.yMm());
    if(at < 0 || w.here() < 0){ res.status = w.here() < 0 ? ExploreStatus::WalkerError : ExploreStatus::GraphFull; return res; }
    to_known[w.here()] = at;
    res.start = at;

    int heading = 1;   // facing east at the start
    for(int moves = 0; moves < 4000; moves++){
        if(w.type() == NodeType::End){
            res.end_found = true;
            res.end = at;
            res.at_end = true;
            break;
        }
        const int back = w.backExit();
        const int prefs[4] = {(heading + 3) % 4, heading, (heading + 1) % 4, (heading + 2) % 4};
        int k = -1;
        for(int p=0; p<4 && k<0; p++){
            for(int j=0;j<w.exitCount();j++){
                if(w.heading(j) == prefs[p] && (j != back || p == 3)){ k = j; break; }
            }
        }
        if(k < 0) k = back;
        if(k < 0){ res.status = ExploreStatus::WalkerError; break; }

        heading = w.heading(k);
        const int from = w.here();
        const int len = w.go(k);
        if(len < 0){ res.status = ExploreStatus::WalkerError; break; }
        const int arrive_back = w.backExit();

        int kv = to_known[w.here()];
        if(kv < 0){
            kv = known.addNode(w.type(), w.xMm(), w.yMm());
            if(kv < 0){ res.status = ExploreStatus::GraphFull; break; }
            to_known[w.here()] = kv;
        }
        if(!used[from][k]){
            used[from][k] = true;
            used[w.here()][arrive_back] = true;
            if(known.addEdge(at, kv, len, k, arrive_back) < 0){ res.status = ExploreStatus::GraphFull; break; }
        }
        at = kv;
        res.moves++;
    }
    res.walked_mm = w.totalMm();
    res.elapsed_ms = w.elapsedMs();
    return res;
}
