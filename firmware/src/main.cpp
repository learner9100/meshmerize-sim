// Meshmerize robot firmware.
//
//   boot        -> serial menu for hardware tests (any time, USB connected)
//   short press -> Dry Run (explore, stop on End, red LED on); pressed again -> Actual Run
//   long press  -> Actual Run from the route stored in flash (after a reset between runs)
//
// The maze logic is the same code the simulator runs (../core). Hardware
// details are in config.h, sensors.h, motors.h and drive.h.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <Preferences.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "drive.h"
#include "robot_walker.h"
#include "explorer.h"
#include "solver.h"
#include "menu.h"

static IrArray ir;
static Motors motors;
static Encoders encoders;
static Driver driver(ir, motors, encoders);

// big objects stay global: the loop task stack is small
static MazeGraph known;
static Explorer explorer;
static PathFinder finder;
static Path path;
static int route[MAX_NODES];
static int route_len = 0;
static bool have_map = false;

static Menu menu(ir, motors, encoders, driver, known, explorer, finder, path);

static ExploreConfig policyConfig(){
    ExploreConfig c;
    c.trust_ids = false;
    c.merge_mm = MERGE_MM;
    c.model.mm_per_s = DRY_MM_PER_S;
    c.model.node_ms = DRY_NODE_MS;
    c.model.turn_ms = DRY_TURN_MS;
    c.model.uturn_ms = DRY_UTURN_MS;
    switch(POLICY){
        case Policy::FirstPath:    c.stop_at_end = true; break;
        case Policy::DfsFull:      break;
        case Policy::NearestFull:  c.order = Frontier::Nearest; break;
        case Policy::Prune:        c.order = Frontier::Nearest; c.prune = true; break;
        case Policy::PruneGrid:    c.order = Frontier::Nearest; c.prune = true; c.manhattan = true; break;
        case Policy::PruneGuard:   c.order = Frontier::Nearest; c.prune = true; c.manhattan = true;
                                   c.limit_ms = DRY_LIMIT_MS; c.margin_ms = DRY_MARGIN_MS; break;
        case Policy::NearestGuard: c.order = Frontier::Nearest;
                                   c.limit_ms = DRY_LIMIT_MS; c.margin_ms = DRY_MARGIN_MS; break;
    }
    return c;
}

static void saveRoute(){
    Preferences p;
    p.begin("maze", false);
    p.putInt("len", route_len);
    p.putBytes("route", route, route_len * sizeof(int));
    p.end();
}

static bool loadRoute(){
    Preferences p;
    p.begin("maze", true);
    route_len = p.getInt("len", 0);
    const bool ok = route_len > 0 && route_len <= MAX_NODES
                 && p.getBytes("route", route, route_len * sizeof(int)) == route_len * sizeof(int);
    p.end();
    if(!ok) route_len = 0;
    return ok;
}

static void waitButtonRelease(){
    while(digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(50);
}

static void blink(int times){
    for(int i=0;i<times;i++){
        digitalWrite(LED_STATUS_PIN, HIGH); delay(120);
        digitalWrite(LED_STATUS_PIN, LOW);  delay(120);
    }
}

static void dryRun(){
    Serial.println("DRY RUN: exploring");
    digitalWrite(LED_RED_PIN, LOW);
    driver.clearAbort();
    waitButtonRelease();
    delay(1000);

    RobotWalker robot(driver, SPEED_DRY, START_HEADING, START_EXITS);
    known = MazeGraph{};
    const unsigned long t0 = millis();
    const ExploreResult r = explorer.run(robot, known, policyConfig());
    const unsigned long dry_ms = millis() - t0;
    driver.stop();

    Serial.printf("status %d, End %s, at End %s, complete %s, %d nodes, %d edges, drove %d mm in %lu ms\n",
                  (int)r.status, r.end_found ? "found" : "NOT found", r.at_end ? "yes" : "no",
                  r.complete ? "yes" : "no", known.nodeCount(), known.edgeCount(), r.walked_mm, dry_ms);

    have_map = r.status == ExploreStatus::Ok && r.end_found && robot.type() == NodeType::End;
    if(!have_map){
        Serial.println("dry run failed; no route saved");
        blink(5);
        return;
    }
    digitalWrite(LED_RED_PIN, HIGH);

    if(!finder.find(known, r.start, r.end, Metric{}, path)){
        Serial.println("no route in the map?!");
        return;
    }
    route_len = 0;
    for(int i=0;i<path.edge_count;i++){
        const Node& a = known.node(path.nodes[i]);
        const Node& b = known.node(path.nodes[i+1]);
        const int dx = b.x_mm - a.x_mm, dy = b.y_mm - a.y_mm;
        // map positions come from the robot's own odometry, so the dominant axis is the heading
        route[route_len++] = abs(dx) > abs(dy) ? (dx > 0 ? 1 : 3) : (dy > 0 ? 2 : 0);
    }
    saveRoute();
    Serial.printf("route: %d mm, %d turns, %d moves, saved to flash\n", path.total_mm, path.turns, route_len);
}

static void actualRun(){
    if(route_len == 0 && !loadRoute()){
        Serial.println("no route stored. do a Dry Run first");
        blink(3);
        return;
    }
    Serial.printf("ACTUAL RUN: %d moves\n", route_len);
    digitalWrite(LED_RED_PIN, LOW);
    driver.clearAbort();
    waitButtonRelease();
    delay(1000);

    // the route is the compass heading to take at each node
    RobotWalker robot(driver, SPEED_RUN, START_HEADING, START_EXITS);
    const unsigned long t0 = millis();
    for(int i=0;i<route_len;i++){
        if(robot.goHeading(route[i]) < 0){
            Serial.printf("move %d failed\n", i);
            driver.stop();
            blink(5);
            return;
        }
    }
    driver.stop();
    Serial.printf("done in %lu ms, %s\n", millis() - t0,
                  robot.type() == NodeType::End ? "standing on End" : "but this does not look like End");
    if(robot.type() == NodeType::End) digitalWrite(LED_RED_PIN, HIGH);
}

static void printMap(){
    if(!have_map){ Serial.println("no map yet"); return; }
    for(int i=0;i<known.nodeCount();i++){
        const Node& n = known.node(i);
        Serial.printf("node %3d type %d at (%6d,%6d) exits:", i, (int)n.type, n.x_mm, n.y_mm);
        for(int k=0;k<n.exit_count;k++){
            const Edge& e = known.edge(n.exit_edge[k]);
            Serial.printf(" ->%d (%dmm, cmd %d)", known.otherEnd(n.exit_edge[k], i), e.length_mm, n.exit_cmd[k]);
        }
        Serial.println();
    }
    Serial.print("route:");
    for(int i=0;i<route_len;i++) Serial.printf(" %c", "NESW"[route[i]]);
    Serial.println();
}

void setup(){
    Serial.begin(115200);
    WiFi.mode(WIFI_OFF);
    btStop();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_STATUS_PIN, OUTPUT);
    if(BUZZER_PIN >= 0) pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);

    ir.begin();
    motors.begin();
    encoders.begin();
    delay(300);

    Serial.println();
    Serial.println("meshmerize robot ready. WiFi and Bluetooth are off.");
    if(loadRoute()) Serial.printf("a route with %d moves is stored from an earlier Dry Run (long-press to drive it)\n", route_len);
    menu.show();
    blink(2);
}

// 0 = not pressed, 1 = short press, 2 = held for 2 s
static int readButton(){
    if(digitalRead(BUTTON_PIN) != LOW) return 0;
    const unsigned long t0 = millis();
    while(digitalRead(BUTTON_PIN) == LOW){
        if(millis() - t0 > 2000){ digitalWrite(LED_STATUS_PIN, HIGH); }
        delay(10);
    }
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(50);
    return millis() - t0 > 2000 ? 2 : 1;
}

void loop(){
    const char key = menu.poll();
    const int button = readButton();

    if(key == 'm'){ printMap(); menu.show(); }
    else if(key == 'd' || (button == 1 && !have_map)){ dryRun(); menu.show(); }
    else if(key == 'a' || (button == 1 && have_map) || button == 2){ actualRun(); menu.show(); }

    delay(20);
}
