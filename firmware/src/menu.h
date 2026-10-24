#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "drive.h"
#include "maze_gen.h"
#include "explorer.h"
#include "solver.h"
#include "walker.h"

// Serial test menu for bringing the hardware up, one subsystem at a time.
// Connect at 115200 and press a key. Any key stops a running test.
class Menu{
    private:
        IrArray& ir_;
        Motors& motors_;
        Encoders& enc_;
        Driver& drive_;
        MazeGraph& known_;
        Explorer& explorer_;
        PathFinder& finder_;
        Path& path_;

        static bool keyPressed(){
            if(!Serial.available()) return false;
            while(Serial.available()) Serial.read();
            return true;
        }

        void irLive(bool normalised){
            Serial.println(normalised ? "normalised 0..1000 (1000 = line), position at the end" : "raw ADC values");
            while(!keyPressed()){
                ir_.read();
                for(int i=0;i<IR_COUNT;i++) Serial.printf("%5d", normalised ? ir_.norm(i) : ir_.raw(i));
                if(normalised) Serial.printf("   pos %5d  lit %d%s%s", ir_.linePosition(), ir_.litCount(),
                                             ir_.leftBranch() ? " L" : "", ir_.rightBranch() ? " R" : "");
                Serial.println();
                delay(100);
            }
        }

        void irCalibrate(){
            Serial.println("sweep the sensor row across the line for 6 seconds...");
            ir_.resetCalibration();
            const unsigned long t0 = millis();
            while(millis() - t0 < 6000){ ir_.calibrateStep(); delay(2); }
            ir_.saveCalibration();
            Serial.println("saved. check with option 3 that the line reads ~1000 and the floor ~0");
        }

        void motorTest(){
            Serial.println("left forward 2 s, right forward 2 s, both 2 s. watch wheel direction and encoder counts");
            const int tests[3][2] = {{300, 0}, {0, 300}, {300, 300}};
            for(int t=0; t<3; t++){
                enc_.reset();
                motors_.set(tests[t][0], tests[t][1]);
                delay(2000);
                motors_.stop();
                Serial.printf("  L=%d R=%d  ->  counts L %ld  R %ld  (both should be positive; equal when both run)\n",
                              tests[t][0], tests[t][1], enc_.left(), enc_.right());
                delay(500);
            }
            Serial.println("wheel backwards? flip M_*_REVERSED. count negative? flip E_*_REVERSED.");
        }

        void encoderTest(){
            Serial.println("push the robot exactly 1000 mm forward by hand, then press a key");
            enc_.reset();
            while(!keyPressed()){
                Serial.printf("  L %ld  R %ld  (%.0f mm with current config)\r", enc_.left(), enc_.right(), enc_.mm());
                delay(200);
            }
            const float avg = (enc_.left() + enc_.right()) / 2.0f;
            if(avg <= 0){ Serial.println("\n  no counts. check wiring and E_*_REVERSED"); return; }
            const float mm_per_count = 1000.0f / avg;
            Serial.printf("\n  mm per count = %.4f  ->  set COUNTS_PER_WHEEL_REV = %.1f (for a %.1f mm wheel)\n",
                          mm_per_count, WHEEL_DIAMETER_MM * PI / mm_per_count / 2, WHEEL_DIAMETER_MM);
        }

        void lineTest(){
            Serial.println("line following for 8 s at SPEED_DRY. tune LINE_KP / LINE_KD until it is smooth");
            drive_.clearAbort();
            const unsigned long t0 = millis();
            while(millis() - t0 < 8000 && !keyPressed()){
                NodeInfo n = drive_.followUntilNode(SPEED_DRY);
                Serial.printf("  node after %d mm: %s%s%s%s%s\n", n.segment_mm,
                              n.left ? "L " : "", n.straight ? "S " : "", n.right ? "R " : "",
                              n.dead_end ? "dead-end " : "", n.end_mark ? "END " : "");
                if(n.dead_end || n.end_mark) break;
            }
            drive_.stop();
        }

        void turnTest(){
            Serial.println("left 90, right 90, about-face. adjust WHEEL_BASE_MM / TURN_OVERSHOOT_DEG until it lands on the line");
            drive_.clearAbort();
            drive_.turn(-1); delay(700);
            drive_.turn(1);  delay(700);
            drive_.turn(2);  delay(700);
            drive_.turn(2);
        }

        void bench(){
            // the truth maze only exists for this test, so it can live on the heap
            GridMaze* maze = new GridMaze;
            MazeGraph* truth = new MazeGraph;
            Serial.printf("sizes: MazeGraph %u, Explorer %u, PathFinder %u bytes; free heap %u\n",
                          (unsigned)sizeof(MazeGraph), (unsigned)sizeof(Explorer), (unsigned)sizeof(PathFinder), (unsigned)ESP.getFreeHeap());
            for(unsigned int seed = 1; seed <= 5; seed++){
                int s, e;
                if(!generateGrid(*maze, 10, 10, 3, seed) || !buildGraph(*maze, CELL_MM, *truth, s, e)) continue;
                ExploreConfig cfg;
                cfg.order = Frontier::Nearest;
                cfg.prune = true;
                cfg.manhattan = true;
                known_ = MazeGraph{};
                Walker w(*truth, s);
                const int64_t t0 = esp_timer_get_time();
                ExploreResult r = explorer_.run(w, known_, cfg);
                const int64_t t1 = esp_timer_get_time();
                finder_.find(known_, r.start, r.end, Metric{}, path_);
                const int64_t t2 = esp_timer_get_time();
                Serial.printf("  seed %u: %d nodes, explore %lld us, solve %lld us, %d mm\n",
                              seed, known_.nodeCount(), (long long)(t1 - t0), (long long)(t2 - t1), path_.total_mm);
            }
            Serial.printf("stack high-water mark: %u words\n", (unsigned)uxTaskGetStackHighWaterMark(NULL));
            delete truth;
            delete maze;
        }

    public:
        Menu(IrArray& ir, Motors& m, Encoders& e, Driver& d,
             MazeGraph& known, Explorer& ex, PathFinder& pf, Path& p)
            : ir_(ir), motors_(m), enc_(e), drive_(d), known_(known), explorer_(ex), finder_(pf), path_(p) {}

        void show(){
            Serial.println();
            Serial.println("=== meshmerize robot: test menu ===");
            Serial.println(" 1  IR raw values");
            Serial.println(" 2  IR calibrate (sweep over the line) and save");
            Serial.println(" 3  IR normalised + line position + branch flags");
            Serial.println(" 4  motor direction and encoder sign test");
            Serial.println(" 5  encoder scale: push 1 m by hand");
            Serial.println(" 6  line following until a node");
            Serial.println(" 7  turn test");
            Serial.println(" 8  maze logic benchmark on the chip (wipes the stored map)");
            Serial.println(" d  Dry Run      (or short-press the button)");
            Serial.println(" a  Actual Run   (or short-press again; long-press drives the route stored in flash)");
            Serial.println(" m  print the map from the last Dry Run");
            Serial.print("> ");
        }

        // returns the key for d/a/m so main can act on it, 0 otherwise
        char poll(){
            if(!Serial.available()) return 0;
            const char c = Serial.read();
            while(Serial.available()) Serial.read();
            Serial.println(c);
            switch(c){
                case '1': irLive(false); break;
                case '2': irCalibrate(); break;
                case '3': irLive(true); break;
                case '4': motorTest(); break;
                case '5': encoderTest(); break;
                case '6': lineTest(); break;
                case '7': turnTest(); break;
                case '8': bench(); break;
                case 'd': case 'a': case 'm': return c;
                default: break;
            }
            show();
            return 0;
        }
};
