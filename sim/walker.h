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

        // Compass direction of exit k: 0 N, 1 E, 2 S, 3 W, -1 if unknown or
        // diagonal. Optional extension, the core explorer does not need it.
        int heading(int k) const {
            if(k < 0 || k >= MAX_EXITS || k >= exitCount()) return -1;
            const Node& a = truth_.node(at_);
            const Node& b = truth_.node(truth_.otherEnd(a.exit_edge[k], at_));
            const int dx = b.x_mm - a.x_mm;
            const int dy = b.y_mm - a.y_mm;
            if(dx == 0 && dy < 0) return 0;
            if(dx > 0 && dy == 0) return 1;
            if(dx == 0 && dy > 0) return 2;
            if(dx < 0 && dy == 0) return 3;
            return -1;
        }

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