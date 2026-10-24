#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

constexpr int NO_LINE = -32768;

// The IR array. Raw readings are normalised against a stored calibration so
// 0 = darkest seen, 1000 = brightest seen, with 1000 always meaning "line".
class IrArray{
    private:
        int raw_[IR_COUNT];
        int lo_[IR_COUNT];
        int hi_[IR_COUNT];
        int norm_[IR_COUNT];
        int last_pos_ = 0;

        int readOne(int i) const {
            if(IR_ANALOG) return analogRead(IR_PIN[i]);
            return digitalRead(IR_PIN[i]) ? 4095 : 0;
        }

    public:
        void begin(){
            for(int i=0;i<IR_COUNT;i++){
                pinMode(IR_PIN[i], INPUT);
                lo_[i] = 4095;
                hi_[i] = 0;
                raw_[i] = norm_[i] = 0;
            }
            if(IR_EMITTER_PIN >= 0){
                pinMode(IR_EMITTER_PIN, OUTPUT);
                digitalWrite(IR_EMITTER_PIN, HIGH);
            }
            if(IR_ANALOG){
                analogReadResolution(12);
                analogSetAttenuation(ADC_11db);
            }
            loadCalibration();
        }

        void read(){
            for(int i=0;i<IR_COUNT;i++){
                raw_[i] = readOne(i);
                int span = hi_[i] - lo_[i];
                if(span < 50) span = 50;
                long v = (long)(raw_[i] - lo_[i]) * 1000 / span;
                if(v < 0) v = 0;
                if(v > 1000) v = 1000;
                norm_[i] = IR_LINE_READS_HIGH ? (int)v : 1000 - (int)v;
            }
        }

        // call repeatedly while sweeping the array across the line
        void calibrateStep(){
            for(int i=0;i<IR_COUNT;i++){
                const int v = readOne(i);
                if(v < lo_[i]) lo_[i] = v;
                if(v > hi_[i]) hi_[i] = v;
            }
        }

        void resetCalibration(){
            for(int i=0;i<IR_COUNT;i++){ lo_[i] = 4095; hi_[i] = 0; }
        }

        void saveCalibration(){
            Preferences p;
            p.begin("ir", false);
            p.putBytes("lo", lo_, sizeof lo_);
            p.putBytes("hi", hi_, sizeof hi_);
            p.end();
        }

        bool loadCalibration(){
            Preferences p;
            p.begin("ir", true);
            const bool ok = p.getBytes("lo", lo_, sizeof lo_) == sizeof lo_
                         && p.getBytes("hi", hi_, sizeof hi_) == sizeof hi_;
            p.end();
            if(!ok) resetCalibration();
            return ok;
        }

        int raw(int i) const { return raw_[i]; }
        int norm(int i) const { return norm_[i]; }
        bool onLine(int i) const { return norm_[i] >= IR_ON_LINE; }

        int litCount() const {
            int n = 0;
            for(int i=0;i<IR_COUNT;i++) if(onLine(i)) n++;
            return n;
        }
        bool allDark() const { return litCount() == 0; }
        bool allLit() const { return litCount() == IR_COUNT; }

        bool leftBranch() const {
            for(int i=0;i<NODE_SIDE_SENSORS;i++) if(!onLine(i)) return false;
            return true;
        }
        bool rightBranch() const {
            for(int i=IR_COUNT-NODE_SIDE_SENSORS;i<IR_COUNT;i++) if(!onLine(i)) return false;
            return true;
        }
        bool centreLit() const {
            return onLine(IR_COUNT/2 - 1) || onLine(IR_COUNT/2);
        }

        // weighted position of the line, -1000 (far left) .. 1000 (far right).
        // When the line is lost, returns the side it was last seen on.
        int linePosition(){
            long sum = 0, weight = 0;
            for(int i=0;i<IR_COUNT;i++){
                const long w = norm_[i];
                sum += w * (i * 2000L / (IR_COUNT - 1) - 1000);
                weight += w;
            }
            if(weight < 200) return last_pos_ < 0 ? -1000 : 1000;
            last_pos_ = (int)(sum / weight);
            return last_pos_;
        }

        void print(Stream& s, bool normalised) const {
            for(int i=0;i<IR_COUNT;i++) s.printf("%5d", normalised ? norm_[i] : raw_[i]);
            s.println();
        }
};
