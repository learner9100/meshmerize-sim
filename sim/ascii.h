#pragma once
#include <cstdio>
#include "maze_gen.h"
#include "solver.h"

struct Overlay{
    char mark[MAX_GRID][MAX_GRID];
    Overlay(){ clear(); }
    void clear(){
        for(int y=0;y<MAX_GRID;y++)
            for(int x=0;x<MAX_GRID;x++) mark[y][x] = ' ';
    }
};

// corridors between nodes are straight, so just step from a towards b
inline void markCorridor(const MazeGraph& g, int a, int b, int cell_mm, Overlay& o, char c, bool keep_existing){
    int x = g.node(a).x_mm / cell_mm;
    int y = g.node(a).y_mm / cell_mm;
    const int bx = g.node(b).x_mm / cell_mm;
    const int by = g.node(b).y_mm / cell_mm;
    const int dx = bx > x ? 1 : (bx < x ? -1 : 0);
    const int dy = by > y ? 1 : (by < y ? -1 : 0);
    while(true){
        if(x >= 0 && y >= 0 && x < MAX_GRID && y < MAX_GRID){
            if(!keep_existing || o.mark[y][x] == ' ') o.mark[y][x] = c;
        }
        if(x == bx && y == by) break;
        x += dx;
        y += dy;
    }
}

inline void markPath(const MazeGraph& g, const Path& p, int cell_mm, Overlay& o, char c = '*'){
    if(!p.found) return;
    for(int i=0;i<p.edge_count;i++) markCorridor(g, p.nodes[i], p.nodes[i+1], cell_mm, o, c, false);
    if(p.node_count == 1) markCorridor(g, p.nodes[0], p.nodes[0], cell_mm, o, c, false);
}

inline void markTrace(const MazeGraph& g, const int* trace, int len, int cell_mm, Overlay& o, char c = '.'){
    for(int i=0;i+1<len;i++) markCorridor(g, trace[i], trace[i+1], cell_mm, o, c, true);
}

inline void printMaze(const GridMaze& m, const Overlay* o = nullptr){
    for(int y=0; y<m.height; y++){
        for(int x=0; x<m.width; x++) std::printf(m.open[y][x][0] ? "+   " : "+---");
        std::printf("+\n");
        for(int x=0; x<m.width; x++){
            char what = ' ';
            if(x == 0 && y == 0) what = 'S';
            else if(x == m.width-1 && y == m.height-1) what = 'E';
            else if(m.checkpoint[y][x]) what = 'C';
            const char mark = o ? o->mark[y][x] : ' ';
            std::printf("%c%c%c ", m.open[y][x][3] ? ' ' : '|', mark, what);
        }
        std::printf("|\n");
    }
    for(int x=0; x<m.width; x++) std::printf("+---");
    std::printf("+\n");
}
