#pragma once
#include "maze_graph.h"
#include "maze_gen.h"
#include "drive.h"

// The real robot seen through the Walker interface the explorer expects.
// Exits are numbered by compass heading (N, E, S, W), so a node looks the
// same every time the robot arrives there, whichever way it came from.
class RobotWalker{
    private:
        Driver& drive_;
        int speed_;
        int heading_;
        int x_ = 0, y_ = 0;
        int total_ = 0;
        NodeType type_ = NodeType::Start;
        int exits_[4];
        int exit_count_ = 0;
        int back_ = -1;
        bool failed_ = false;

        void setNode(const NodeInfo& n, bool first){
            bool has[4] = {false, false, false, false};
            if(n.left)     has[(heading_ + 3) % 4] = true;
            if(n.straight) has[heading_] = true;
            if(n.right)    has[(heading_ + 1) % 4] = true;
            if(!first)     has[opposite(heading_)] = true;

            exit_count_ = 0;
            back_ = -1;
            for(int h=0; h<4; h++){
                if(!has[h]) continue;
                if(!first && h == opposite(heading_)) back_ = exit_count_;
                exits_[exit_count_++] = h;
            }
            if(first) type_ = NodeType::Start;
            else if(n.end_mark) type_ = NodeType::End;
            else if(exit_count_ == 1) type_ = NodeType::DeadEnd;
            else type_ = NodeType::Normal;
        }

    public:
        // start_exits: bit 0 left, bit 1 straight, bit 2 right as seen from the robot at Start
        RobotWalker(Driver& d, int speed, int start_heading, int start_exits)
            : drive_(d), speed_(speed), heading_(start_heading) {
            NodeInfo n;
            n.left = start_exits & 1;
            n.straight = start_exits & 2;
            n.right = start_exits & 4;
            setNode(n, true);
        }

        void setSpeed(int s){ speed_ = s; }

        int here() const { return -1; }           // the robot does not know node ids, only positions
        NodeType type() const { return type_; }
        int exitCount() const { return exit_count_; }
        int backExit() const { return back_; }
        int totalMm() const { return total_; }
        int xMm() const { return x_; }
        int yMm() const { return y_; }
        int heading(int k) const { return k >= 0 && k < exit_count_ ? exits_[k] : -1; }
        int facing() const { return heading_; }
        bool failed() const { return failed_; }

        int go(int k){
            if(k < 0 || k >= exit_count_ || failed_) return -1;
            return goHeading(exits_[k]);
        }

        // turn to an absolute heading and follow the line to the next node,
        // whether or not that exit was noticed on arrival
        int goHeading(int target){
            if(target < 0 || target > 3 || failed_) return -1;
            int q = (target - heading_ + 4) % 4;   // 0 straight, 1 right, 2 back, 3 left
            if(q == 3) q = -1;
            drive_.turn(q);
            heading_ = target;

            const NodeInfo n = drive_.followUntilNode(speed_);
            if(drive_.wasAborted()){ failed_ = true; return -1; }

            x_ += DIR_DX[heading_] * n.segment_mm;
            y_ += DIR_DY[heading_] * n.segment_mm;
            total_ += n.segment_mm;
            setNode(n, false);
            return n.segment_mm > 0 ? n.segment_mm : 1;
        }
};
