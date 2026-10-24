#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

struct NodeInfo{
    bool left = false;
    bool straight = false;
    bool right = false;
    bool dead_end = false;
    bool end_mark = false;
    int segment_mm = 0;        // distance driven since the previous node
};

// Low-level driving: follow the line to the next node, turn, go straight.
// Everything the maze logic needs from the hardware goes through here.
class Driver{
    private:
        IrArray& ir_;
        Motors& motors_;
        Encoders& enc_;
        float integral_ = 0;
        float last_err_ = 0;
        bool stop_flag_ = false;

        static int clampSpeed(int v){
            if(v > PWM_MAX) return PWM_MAX;
            if(v < -PWM_MAX) return -PWM_MAX;
            return v;
        }

        void followStep(int base){
            const float err = ir_.linePosition() / 1000.0f;
            integral_ += err;
            const float corr = LINE_KP * err * base + LINE_KI * integral_ + LINE_KD * (err - last_err_) * base;
            last_err_ = err;
            motors_.set(clampSpeed(base + (int)corr), clampSpeed(base - (int)corr));
        }

        // drive straight by encoders for `mm`, keeping both wheels in step
        void straight(int mm, int speed){
            const float start = enc_.mm();
            while(enc_.mm() - start < mm && !aborted()){
                const float diff = enc_.leftMm() - enc_.rightMm();
                const int corr = (int)(diff * 8);
                motors_.set(clampSpeed(speed - corr), clampSpeed(speed + corr));
                delay(LOOP_MS);
            }
        }

        bool aborted(){
            if(digitalRead(BUTTON_PIN) == LOW) stop_flag_ = true;
            return stop_flag_;
        }

    public:
        Driver(IrArray& ir, Motors& m, Encoders& e) : ir_(ir), motors_(m), enc_(e) {}

        bool wasAborted() const { return stop_flag_; }
        void clearAbort(){ stop_flag_ = false; }

        // Follows the line until something happens: a branch to a side (then
        // rolls forward so the wheels sit on the junction), a dead end or
        // the End marker. Returns what it saw.
        NodeInfo followUntilNode(int base){
            NodeInfo n;
            integral_ = 0;
            last_err_ = 0;
            const float seg_start = enc_.mm();
            float lit_since = -1, dark_since = -1, left_since = -1, right_since = -1;

            while(!aborted()){
                ir_.read();
                const float here = enc_.mm();

                if(ir_.allLit()){
                    if(lit_since < 0) lit_since = here;
                    if(here - lit_since > END_MARK_MM){ n.end_mark = true; break; }
                } else lit_since = -1;

                if(ir_.allDark()){
                    if(dark_since < 0) dark_since = here;
                    if(here - dark_since > DEAD_END_MM){ n.dead_end = true; break; }
                } else dark_since = -1;

                if(ir_.leftBranch()){ if(left_since < 0) left_since = here; } else left_since = -1;
                if(ir_.rightBranch()){ if(right_since < 0) right_since = here; } else right_since = -1;

                const bool left_ok = left_since >= 0 && here - left_since > NODE_CONFIRM_MM;
                const bool right_ok = right_since >= 0 && here - right_since > NODE_CONFIRM_MM;
                if(left_ok || right_ok){
                    n.left = left_ok;
                    n.right = right_ok;
                    // roll the axle onto the junction, watching for the other side and for a straight exit
                    const float roll_start = enc_.mm();
                    while(enc_.mm() - roll_start < IR_TO_AXLE_MM && !aborted()){
                        ir_.read();
                        if(ir_.leftBranch()) n.left = true;
                        if(ir_.rightBranch()) n.right = true;
                        if(ir_.allLit() && enc_.mm() - roll_start > IR_TO_AXLE_MM / 2) n.end_mark = true;
                        motors_.set(base, base);
                        delay(LOOP_MS);
                    }
                    ir_.read();
                    n.straight = ir_.centreLit() && !ir_.allLit();
                    break;
                }

                followStep(base);
                delay(LOOP_MS);
            }
            motors_.brake();
            delay(TURN_SETTLE_MS);
            motors_.stop();
            if(n.dead_end || n.end_mark) straight(IR_TO_AXLE_MM, base / 2);   // axle onto the spot too
            n.segment_mm = (int)(enc_.mm() - seg_start + 0.5f);
            return n;
        }

        // quarter turns: +1 right, -1 left, +2 or -2 about-face
        void turn(int quarters){
            if(quarters == 0) return;
            const int dir = quarters > 0 ? 1 : -1;
            const float arc = WHEEL_BASE_MM * PI / 4 * abs(quarters) * (90 + TURN_OVERSHOOT_DEG) / 90.0f;
            const float l0 = enc_.leftMm();
            while(fabsf(enc_.leftMm() - l0) < arc && !aborted()){
                motors_.set(SPEED_TURN * dir, -SPEED_TURN * dir);
                delay(LOOP_MS);
            }
            // keep creeping until the centre sees the line again, within reason
            const float l1 = enc_.leftMm();
            while(!aborted()){
                ir_.read();
                if(ir_.centreLit()) break;
                if(fabsf(enc_.leftMm() - l1) > WHEEL_BASE_MM * PI / 8) break;
                motors_.set(SPEED_TURN * dir * 2 / 3, -SPEED_TURN * dir * 2 / 3);
                delay(LOOP_MS);
            }
            motors_.brake();
            delay(TURN_SETTLE_MS);
            motors_.stop();
        }

        void forward(int mm, int speed){ straight(mm, speed); motors_.stop(); }

        void stop(){ motors_.stop(); }
};
