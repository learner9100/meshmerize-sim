#pragma once
#include "maze_graph.h"

struct ExploreResult{
    bool end_found=false;
    int start=-1;
    int end=-1;
    int walked_mm=0;

};

//DFS exploration
template<class W>

ExploreResult explore(W& w, MazeGraph& known, bool stop_at_end){
    ExploreResult res;
    int to_known[MAX_NODES];
    int parent_node[MAX_NODES];
    int parent_exit[MAX_NODES];
    bool tried[MAX_NODES][MAX_EXITS];

    for(int i=0; i<MAX_NODES; i++){
        to_known[i] = -1;
        parent_node[i] = -1;
        parent_exit[i] = -1;
        for(int j=0; j<MAX_EXITS; j++){
            tried[i][j] = false;
        }
    }

    int at = known.addNode(w.type());
    if(at<0) return res;
    to_known[w.here()]=at;
    res.start = at;

    while(true){
        if(stop_at_end && res.end_found) break;

        int k=-1;
        for(int j=0;j<w.exitCount(); j++){
            if(!tried[at][j]){
                k = j;
                break;
            }
        }

        if(k>=0){
            tried[at][k] = true;
            const int len = w.go(k);
            const int back = w.backExit();
            const int v = w.here();

            if (to_known[v] < 0){
                const int kv = known.addNode(w.type());
                if(kv<0) break;
                to_known[v] = kv;
                parent_node[kv] = at;
                parent_exit[kv] = back;
                tried[kv][back] = true;

                if (known.addEdge(at, kv, len)<0) break;
                if(w.type() == NodeType::End){
                    res.end_found = true;
                    res.end = kv;
                }
                at=kv;
            }
            else{
                const int kv = to_known[v];
                if (known.addEdge(at, kv, len)<0) break;
                tried[kv][back] = true;
                w.go(back);
            }
        }
        else{
            if(parent_node[at]<0) break;
            const int p = parent_node[at];
            w.go(parent_exit[at]);
            at=p;

        }
    }
    res.walked_mm = w.totalMm();
    return res;
}