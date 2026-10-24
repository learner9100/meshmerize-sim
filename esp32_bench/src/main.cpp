// Runs the explorer and the solver on generated mazes on the chip and
// prints compute time and memory use. WiFi and Bluetooth are switched off
// because they are not allowed at the competition.
//
// UNVERIFIED: written without a board at hand. Expect to fix small things.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include "maze_gen.h"
#include "solver.h"
#include "explorer.h"
#include "walker.h"

// Big objects must be global or static on the ESP32, never locals: the
// Arduino loop task has only a few KB of stack.
static GridMaze maze;
static MazeGraph truth;
static MazeGraph known;
static Explorer explorer;
static PathFinder finder;
static Path path;

static void benchSeed(unsigned int seed){
    int start, end;
    if(!generateGrid(maze, 10, 10, 3, seed) || !buildGraph(maze, 230, truth, start, end)){
        Serial.printf("seed %u: could not build maze\n", seed);
        return;
    }

    ExploreConfig cfg;
    cfg.order = Frontier::Nearest;
    cfg.prune = true;
    cfg.manhattan = true;

    known = MazeGraph{};
    Walker w(truth, start);
    const int64_t t0 = esp_timer_get_time();
    const ExploreResult res = explorer.run(w, known, cfg);
    const int64_t t1 = esp_timer_get_time();
    const bool ok = finder.find(known, res.start, res.end, Metric{}, path);
    const int64_t t2 = esp_timer_get_time();

    Serial.printf("seed %u: %d nodes, explore %lld us (%d moves), solve %lld us, route %d mm %s\n",
                  seed, known.nodeCount(), (long long)(t1 - t0), res.moves, (long long)(t2 - t1),
                  path.total_mm, ok && res.status == ExploreStatus::Ok ? "ok" : "FAILED");
}

void setup(){
    Serial.begin(115200);
    WiFi.mode(WIFI_OFF);
    btStop();
    delay(1000);

    Serial.printf("sizes: MazeGraph %u, Explorer %u, PathFinder %u, Path %u bytes\n",
                  (unsigned)sizeof(MazeGraph), (unsigned)sizeof(Explorer),
                  (unsigned)sizeof(PathFinder), (unsigned)sizeof(Path));
    Serial.printf("free heap before: %u\n", ESP.getFreeHeap());

    for(unsigned int seed = 1; seed <= 5; seed++) benchSeed(seed);

    Serial.printf("free heap after: %u, loop task stack high-water mark: %u words\n",
                  ESP.getFreeHeap(), (unsigned)uxTaskGetStackHighWaterMark(NULL));
}

void loop(){
    delay(1000);
}
