#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "cost.h"

// placeholders, nobody has measured this robot yet (README: Calibration)
inline CostModel dryRunModel(){
    CostModel m;
    m.mm_per_s = 500;
    m.node_ms  = 300;
    m.turn_ms  = 400;
    m.uturn_ms = 1000;
    return m;
}

inline CostModel actualRunModel(){
    CostModel m;
    m.mm_per_s = 900;
    m.node_ms  = 250;
    m.turn_ms  = 300;
    m.uturn_ms = 800;
    return m;
}

struct OdomParams{
    double scale_err = 0.0;   // 0.02 = believes it drove 2% further
    double noise = 0.0;       // random error per move, fraction of distance
    unsigned int seed = 1;
};

struct RobotConfig{
    CostModel dry = dryRunModel();
    CostModel run = actualRunModel();
    OdomParams odom;
    bool use_odometry = false;
    int merge_mm = 100;
};

// "key = value" lines, # comments. Returns 0, the bad line number, -1 if
// the file is missing, -2 for nonsense values.
inline int loadRobotConfig(const char* path, RobotConfig& cfg){
    FILE* f = std::fopen(path, "r");
    if(!f) return -1;

    char line[256];
    int lineno = 0;
    while(std::fgets(line, sizeof line, f)){
        lineno++;
        char* hash = std::strchr(line, '#');
        if(hash) *hash = 0;

        char key[64];
        double value;
        if(std::sscanf(line, " %63[A-Za-z_.] = %lf", key, &value) != 2){
            bool blank = true;
            for(const char* c = line; *c; c++) if(*c > ' ') blank = false;
            if(blank) continue;
            std::fclose(f);
            return lineno;
        }

        const int iv = (int)value;
        if     (!std::strcmp(key, "dry.mm_per_s"))   cfg.dry.mm_per_s = iv;
        else if(!std::strcmp(key, "dry.node_ms"))    cfg.dry.node_ms = iv;
        else if(!std::strcmp(key, "dry.turn_ms"))    cfg.dry.turn_ms = iv;
        else if(!std::strcmp(key, "dry.uturn_ms"))   cfg.dry.uturn_ms = iv;
        else if(!std::strcmp(key, "run.mm_per_s"))   cfg.run.mm_per_s = iv;
        else if(!std::strcmp(key, "run.node_ms"))    cfg.run.node_ms = iv;
        else if(!std::strcmp(key, "run.turn_ms"))    cfg.run.turn_ms = iv;
        else if(!std::strcmp(key, "run.uturn_ms"))   cfg.run.uturn_ms = iv;
        else if(!std::strcmp(key, "odom.scale_err")) cfg.odom.scale_err = value;
        else if(!std::strcmp(key, "odom.noise"))     cfg.odom.noise = value;
        else if(!std::strcmp(key, "odom.enabled"))   cfg.use_odometry = iv != 0;
        else if(!std::strcmp(key, "odom.merge_mm"))  cfg.merge_mm = iv;
        else { std::fclose(f); return lineno; }
    }
    std::fclose(f);
    if(cfg.dry.mm_per_s <= 0 || cfg.run.mm_per_s <= 0) return -2;
    return 0;
}
