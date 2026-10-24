#pragma once
#include "maze_graph.h"

class Walker{
    private:
        const MazeGraph& truth_;
        int at_;
        int back_exit=-1;
        int total_mm=0;
    
    public:
        Walker(const MazeGraph& truth, int start): truth_(truth), at_(start) {}

        int here() const {return at_;}
        NodeType type() const {return truth_.node(at_).type;}
        int exitCount() const {return truth_.node(at_).exit_count;}
        int backExit() const {return back_exit;}
        int totalMm() const {return total_mm;}
        int xMm() const {return truth_.node(at_).x_mm;}
        int yMm() const {return truth_.node(at_).y_mm;}

        int go(int k){
            if(k < 0 || k >= MAX_EXITS || k >= exitCount()) return -1;
            const int ei = truth_.node(at_).exit_edge[k];
            const int next = truth_.otherEnd(ei, at_);
            const int len = truth_.edge(ei).length_mm;

            const Node& nn = truth_.node(next);
            back_exit=-1;
            for(int j=0;j<nn.exit_count;j++){
                if(nn.exit_edge[j] == ei) back_exit = j;
            }
            at_ = next;
            total_mm += len;
            return len;
        } 
};