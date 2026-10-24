#pragma once
#include "maze_graph.h"
#include "rng.h"

constexpr int MAX_GRID = 20;

// 0 north, 1 east, 2 south, 3 west
constexpr int DIR_DX[4] = {0, 1, 0, -1};
constexpr int DIR_DY[4] = {-1, 0, 1, 0};
inline int opposite(int d){ return (d + 2) % 4; }

struct GridMaze{
    int width = 0;
    int height = 0;
    bool open[MAX_GRID][MAX_GRID][4] = {};        // [y][x][dir]
    bool checkpoint[MAX_GRID][MAX_GRID] = {};

    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }

    bool passage(int x, int y, int d) const { return inside(x, y) && open[y][x][d]; }

    int degree(int x, int y) const {
        int n = 0;
        for(int d=0; d<4; d++) if(open[y][x][d]) n++;
        return n;
    }

    bool straightCell(int x, int y) const {
        if(degree(x, y) != 2) return false;
        return (open[y][x][0] && open[y][x][2]) || (open[y][x][1] && open[y][x][3]);
    }

    int passageCount() const {
        int n = 0;
        for(int y=0; y<height; y++)
            for(int x=0; x<width; x++){
                if(open[y][x][1]) n++;
                if(open[y][x][2]) n++;
            }
        return n;
    }

    void carve(int x, int y, int d){
        open[y][x][d] = true;
        open[y + DIR_DY[d]][x + DIR_DX[d]][opposite(d)] = true;
    }
};

// Randomised DFS gives a perfect maze; every extra opening then joins two
// already-connected cells, so each one adds exactly one loop.
inline bool generateGrid(GridMaze& m, int w, int h, int extra_openings, unsigned int seed){
    if(w < 2 || h < 2 || w > MAX_GRID || h > MAX_GRID || extra_openings < 0) return false;

    m = GridMaze{};
    m.width = w;
    m.height = h;
    Rng rng(seed);

    bool seen[MAX_GRID][MAX_GRID] = {};
    int stack_x[MAX_GRID * MAX_GRID];
    int stack_y[MAX_GRID * MAX_GRID];
    int top = 0;
    stack_x[0] = 0;
    stack_y[0] = 0;
    seen[0][0] = true;

    while(top >= 0){
        const int x = stack_x[top];
        const int y = stack_y[top];
        int options[4];
        int n = 0;
        for(int d=0; d<4; d++){
            const int nx = x + DIR_DX[d];
            const int ny = y + DIR_DY[d];
            if(m.inside(nx, ny) && !seen[ny][nx]) options[n++] = d;
        }
        if(n == 0){ top--; continue; }
        const int d = options[rng.below(n)];
        m.carve(x, y, d);
        const int nx = x + DIR_DX[d];
        const int ny = y + DIR_DY[d];
        seen[ny][nx] = true;
        top++;
        stack_x[top] = nx;
        stack_y[top] = ny;
    }

    // closed inner walls, each once as (cell, east|south)
    int walls[MAX_GRID * MAX_GRID * 2];
    int wall_count = 0;
    for(int y=0; y<h; y++)
        for(int x=0; x<w; x++){
            if(x + 1 < w && !m.open[y][x][1]) walls[wall_count++] = (y * w + x) * 4 + 1;
            if(y + 1 < h && !m.open[y][x][2]) walls[wall_count++] = (y * w + x) * 4 + 2;
        }
    if(extra_openings > wall_count) return false;

    for(int i=0; i<extra_openings; i++){
        const int pick = rng.below(wall_count);
        const int code = walls[pick];
        walls[pick] = walls[--wall_count];
        const int cell = code / 4;
        m.carve(cell % w, cell / w, code % 4);
    }
    return true;
}

// Real checkpoint placement is unknown; random cells are a stand-in.
inline bool placeCheckpoints(GridMaze& m, int count, unsigned int seed){
    const int cells = m.width * m.height;
    if(count < 0 || count > cells - 2) return false;
    for(int y=0; y<m.height; y++)
        for(int x=0; x<m.width; x++) m.checkpoint[y][x] = false;

    Rng rng(seed ^ 0xC0FFEEu);
    int placed = 0;
    while(placed < count){
        const int c = rng.below(cells);
        const int x = c % m.width;
        const int y = c / m.width;
        const bool corner = (x == 0 && y == 0) || (x == m.width - 1 && y == m.height - 1);
        if(corner || m.checkpoint[y][x]) continue;
        m.checkpoint[y][x] = true;
        placed++;
    }
    return true;
}

inline bool isNodeCell(const GridMaze& m, int x, int y){
    if(x == 0 && y == 0) return true;
    if(x == m.width - 1 && y == m.height - 1) return true;
    if(m.checkpoint[y][x]) return true;
    return !m.straightCell(x, y);
}

// Start top-left, End bottom-right. Straight cells fold into one long edge.
inline bool buildGraph(const GridMaze& m, int cell_mm, MazeGraph& g, int& start, int& end){
    g = MazeGraph{};
    start = end = -1;
    if(cell_mm <= 0 || m.width < 1 || m.height < 1 || m.width * m.height < 2) return false;

    int id_of[MAX_GRID][MAX_GRID];
    for(int y=0; y<m.height; y++)
        for(int x=0; x<m.width; x++){
            id_of[y][x] = -1;
            if(!isNodeCell(m, x, y)) continue;
            NodeType t = NodeType::Normal;
            if(x == 0 && y == 0) t = NodeType::Start;
            else if(x == m.width - 1 && y == m.height - 1) t = NodeType::End;
            else if(m.checkpoint[y][x]) t = NodeType::CheckPoint;
            else if(m.degree(x, y) == 1) t = NodeType::DeadEnd;
            const int id = g.addNode(t, x * cell_mm, y * cell_mm);
            if(id < 0) return false;
            id_of[y][x] = id;
            if(t == NodeType::Start) start = id;
            if(t == NodeType::End) end = id;
        }

    for(int y=0; y<m.height; y++)
        for(int x=0; x<m.width; x++){
            const int a = id_of[y][x];
            if(a < 0) continue;
            for(int d=0; d<4; d++){
                if(!m.open[y][x][d]) continue;
                int cx = x, cy = y, steps = 0;
                do{
                    cx += DIR_DX[d];
                    cy += DIR_DY[d];
                    steps++;
                }while(id_of[cy][cx] < 0);
                const int b = id_of[cy][cx];
                if(a < b && g.addEdge(a, b, steps * cell_mm) < 0) return false;   // each corridor once
            }
        }
    g.sortExitsByCompass();
    return true;
}
