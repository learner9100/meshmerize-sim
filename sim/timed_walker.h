#pragma once
#include <cmath>
#include "walker.h"
#include "cost.h"
#include "rng.h"
#include "robot_model.h"

constexpr int MAX_TRACE = 4096;

// Walker plus stopwatch, visit trace and optional odometry drift.
class TimedWalker{
    private:
        Walker w_;
        const MazeGraph& truth_;
        CostModel model_;
        int elapsed_ms = 0;
        int in_dx_ = 0, in_dy_ = 0;
        bool visited_[MAX_NODES] = {};
        int trace_[MAX_TRACE];
        int trace_len_ = 0;

        bool odom_ = false;
        OdomParams op_;
        Rng rng_;
        double rep_x_ = 0, rep_y_ = 0;

        void note(int id){
            visited_[id] = true;
            if(trace_len_ < MAX_TRACE) trace_[trace_len_++] = id;
        }

        double unit(){ return (double)(rng_.next() & 0xFFFF) / 32767.5 - 1.0; }

    public:
        TimedWalker(const MazeGraph& truth, int start, const CostModel& model)
            : w_(truth, start), truth_(truth), model_(model), rng_(1) {
            note(start);
            rep_x_ = truth.node(start).x_mm;
            rep_y_ = truth.node(start).y_mm;
        }

        // from now on here() is -1 and positions drift
        void useOdometry(const OdomParams& p){
            odom_ = true;
            op_ = p;
            rng_.reseed(p.seed);
        }

        int here() const { return odom_ ? -1 : w_.here(); }
        int trueHere() const { return w_.here(); }
        NodeType type() const { return w_.type(); }
        int exitCount() const { return w_.exitCount(); }
        int backExit() const { return w_.backExit(); }
        int totalMm() const { return w_.totalMm(); }
        int xMm() const { return odom_ ? (int)std::lround(rep_x_) : w_.xMm(); }
        int yMm() const { return odom_ ? (int)std::lround(rep_y_) : w_.yMm(); }
        int heading(int k) const { return w_.heading(k); }

        int go(int k){
            const Node& from = truth_.node(w_.here());
            const int len = w_.go(k);
            if(len < 0) return -1;
            const Node& to = truth_.node(w_.here());
            const int dx = to.x_mm - from.x_mm;
            const int dy = to.y_mm - from.y_mm;

            elapsed_ms += moveCostMs(model_, len, turnKind(in_dx_, in_dy_, dx, dy));
            in_dx_ = dx;
            in_dy_ = dy;
            note(w_.here());

            if(odom_){
                rep_x_ += dx * (1.0 + op_.scale_err) + len * op_.noise * unit();
                rep_y_ += dy * (1.0 + op_.scale_err) + len * op_.noise * unit();
            }
            return len;
        }

        int elapsedMs() const { return elapsed_ms; }
        bool visited(int id) const { return id >= 0 && id < MAX_NODES && visited_[id]; }
        const int* trace() const { return trace_; }
        int traceLen() const { return trace_len_; }
        const CostModel& model() const { return model_; }
};
