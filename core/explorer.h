#pragma once
#include "maze_graph.h"
#include "cost.h"
#include "solver.h"

enum class ExploreStatus {
    Ok,
    GraphFull,      // known map ran out of nodes/edges/exits
    WalkerError,    // go() refused a command or here() was unusable
    LostPosition    // map and reality disagree (only possible without trusted ids)
};

// Which unexplored exit to go to next when the current node has none left.
enum class Frontier {
    Deepest,   // classic depth-first backtracking
    Nearest    // least driving from where we stand
};

struct ExploreConfig{
    bool stop_at_end = false;          // leave the moment End is seen
    Frontier order = Frontier::Deepest;

    // Skip branches whose lower bound cannot beat the best known route.
    // Needs node positions. Manhattan is tighter but only valid for mazes
    // with 90 degree corridors; the straight line is always valid.
    bool prune = false;
    bool manhattan = false;

    // Time guard: stop exploring when the estimated finish would pass
    // limit_ms - margin_ms. 0 = no guard. Only active once End is known.
    int limit_ms = 0;
    int margin_ms = 20000;
    CostModel model;                   // the robot's belief about its own speed

    // false = recognise places by position like a real robot has to.
    bool trust_ids = true;
    int merge_mm = 100;                // position mode: this close is "same node"
};

struct ExploreResult{
    ExploreStatus status = ExploreStatus::Ok;
    bool end_found = false;
    bool at_end = false;               // standing at End when done (as far as the map knows)
    bool complete = false;             // no unexplored exit could still hold a shorter route
    int start = -1;
    int end = -1;
    int walked_mm = 0;
    int moves = 0;
    int elapsed_ms = 0;                // explorer's own estimate from cfg.model
    int end_found_ms = -1;             // estimate when End was first seen, -1 if never
    int unexplored_exits = 0;
};

// Builds the known map while driving a Walker. Roughly 15 KB of state, so on
// the ESP32 declare one Explorer static and reuse it.
class Explorer{
    private:
        struct Tree{
            int dist[MAX_NODES];
            int prev[MAX_NODES];
            int via[MAX_NODES];        // edge used to reach the node
            int hops[MAX_NODES];
        };

        enum class Pick { Found, Nothing, AllPruned, OutOfTime };

        ExploreConfig cfg_;
        ExploreResult res_;
        MazeGraph* known_ = nullptr;

        int to_known_[MAX_NODES];      // trusted-id mode: walker id -> known id
        int exits_[MAX_NODES];         // walker exit count per known node
        bool tried_[MAX_NODES][MAX_EXITS];
        bool done_[MAX_NODES];
        Tree from_at_;
        Tree from_start_;
        Tree from_end_;

        int at_ = -1;
        int in_dx_ = 0, in_dy_ = 0;
        int corr_x_ = 0, corr_y_ = 0;  // position mode: drift fixed at each loop closure
        int new_x_ = 0, new_y_ = 0;    // where the last unrecognised arrival was

        void reset(const ExploreConfig& cfg, MazeGraph& known){
            cfg_ = cfg;
            res_ = ExploreResult{};
            known_ = &known;
            for(int i=0;i<MAX_NODES;i++){
                to_known_[i] = -1;
                exits_[i] = 0;
                for(int j=0;j<MAX_EXITS;j++) tried_[i][j] = false;
            }
            at_ = -1;
            in_dx_ = in_dy_ = 0;
            corr_x_ = corr_y_ = 0;
        }

        void dijkstra(int from, Tree& t){
            const int n = known_->nodeCount();
            for(int i=0;i<n;i++){
                t.dist[i] = INF_MM;
                t.prev[i] = -1;
                t.via[i] = -1;
                t.hops[i] = 0;
                done_[i] = false;
            }
            t.dist[from] = 0;
            while(true){
                int u = -1;
                for(int i=0;i<n;i++){
                    if(!done_[i] && t.dist[i] < INF_MM && (u == -1 || t.dist[i] < t.dist[u])) u = i;
                }
                if(u == -1) break;
                done_[u] = true;
                const Node& nd = known_->node(u);
                for(int k=0;k<nd.exit_count;k++){
                    const int ei = nd.exit_edge[k];
                    const int v = known_->otherEnd(ei, u);
                    const int cand = t.dist[u] + known_->edge(ei).length_mm;
                    if(done_[v]) continue;
                    if(cand < t.dist[v] || (cand == t.dist[v] && t.hops[u] + 1 < t.hops[v])){
                        t.dist[v] = cand;
                        t.prev[v] = u;
                        t.via[v] = ei;
                        t.hops[v] = t.hops[u] + 1;
                    }
                }
            }
        }

        // Pessimistic on purpose: every node on the way is priced as a corner.
        int estimateMs(const Tree& t, int u) const {
            const long drive = (long)t.dist[u] * 1000 / cfg_.model.mm_per_s;
            return (int)drive + t.hops[u] * (cfg_.model.node_ms + cfg_.model.turn_ms);
        }

        int firstUntried(int u) const {
            for(int j=0;j<exits_[u];j++){
                if(!tried_[u][j]) return j;
            }
            return -1;
        }

        void account(int len, int from, int to){
            const Node& a = known_->node(from);
            const Node& b = known_->node(to);
            const int out_dx = b.x_mm - a.x_mm;
            const int out_dy = b.y_mm - a.y_mm;
            res_.elapsed_ms += moveCostMs(cfg_.model, len, turnKind(in_dx_, in_dy_, out_dx, out_dy));
            in_dx_ = out_dx;
            in_dy_ = out_dy;
            res_.moves++;
        }

        // Which known node is the walker standing on? -1 means "never seen".
        template<class W>
        int recognise(W& w){
            if(cfg_.trust_ids){
                const int id = w.here();
                if(id < 0 || id >= MAX_NODES){ res_.status = ExploreStatus::WalkerError; return -1; }
                new_x_ = w.xMm();
                new_y_ = w.yMm();
                return to_known_[id];
            }
            new_x_ = w.xMm() + corr_x_;
            new_y_ = w.yMm() + corr_y_;
            const long r2 = (long)cfg_.merge_mm * cfg_.merge_mm;
            int best = -1;
            long best_d2 = 0;
            for(int i=0;i<known_->nodeCount();i++){
                const Node& nd = known_->node(i);
                const long dx = nd.x_mm - new_x_;
                const long dy = nd.y_mm - new_y_;
                const long d2 = dx*dx + dy*dy;
                if(d2 <= r2 && (best < 0 || d2 < best_d2)){ best = i; best_d2 = d2; }
            }
            if(best >= 0){
                corr_x_ += known_->node(best).x_mm - new_x_;
                corr_y_ += known_->node(best).y_mm - new_y_;
            }
            return best;
        }

        template<class W>
        int addPlace(W& w){
            const int id = known_->addNode(w.type(), new_x_, new_y_);
            if(id < 0){ res_.status = ExploreStatus::GraphFull; return -1; }
            if(cfg_.trust_ids) to_known_[w.here()] = id;
            exits_[id] = w.exitCount();
            if(w.type() == NodeType::End && !res_.end_found){
                res_.end_found = true;
                res_.end = id;
            }
            return id;
        }

        // Drive through exit k of the current node and record what we find.
        template<class W>
        bool step(W& w, int k){
            tried_[at_][k] = true;
            const int len = w.go(k);
            if(len < 0){ res_.status = ExploreStatus::WalkerError; return false; }
            const int back = w.backExit();
            if(back < 0 || back >= MAX_EXITS){ res_.status = ExploreStatus::WalkerError; return false; }

            int kv = recognise(w);
            if(res_.status != ExploreStatus::Ok) return false;
            if(kv < 0){
                kv = addPlace(w);
                if(kv < 0) return false;
            }
            else if(kv == at_){
                res_.status = ExploreStatus::LostPosition;
                return false;
            }
            if(known_->addEdge(at_, kv, len, k, back) < 0){
                res_.status = ExploreStatus::GraphFull;
                return false;
            }
            tried_[kv][back] = true;
            account(len, at_, kv);
            if(res_.end_found && res_.end_found_ms < 0) res_.end_found_ms = res_.elapsed_ms;
            at_ = kv;
            return true;
        }

        // Follow known corridors to `target` along the shortest known route.
        template<class W>
        bool driveTo(W& w, int target){
            if(target == at_) return true;
            dijkstra(at_, from_at_);
            if(from_at_.dist[target] >= INF_MM){ res_.status = ExploreStatus::LostPosition; return false; }

            int route[MAX_NODES];
            int count = 0;
            for(int v = target; v != at_; v = from_at_.prev[v]) route[count++] = v;

            for(int i = count - 1; i >= 0; i--){
                const int next = route[i];
                const int slot = known_->exitSlot(at_, from_at_.via[next]);
                const int cmd = known_->node(at_).exit_cmd[slot];
                const int len = w.go(cmd);
                if(len < 0){ res_.status = ExploreStatus::WalkerError; return false; }
                const int seen = recognise(w);
                if(res_.status != ExploreStatus::Ok) return false;
                if(seen != next){ res_.status = ExploreStatus::LostPosition; return false; }
                const int back = w.backExit();
                if(back >= 0 && back < MAX_EXITS) tried_[seen][back] = true;
                account(len, at_, seen);
                at_ = seen;
            }
            return true;
        }

        Pick pickFrontier(int& u_out, int& k_out){
            const int n = known_->nodeCount();
            dijkstra(at_, from_at_);

            const bool pruning = cfg_.prune && res_.end_found;
            const bool guarding = cfg_.limit_ms > 0 && res_.end_found;
            int best_known = INF_MM;
            if(pruning){
                dijkstra(res_.start, from_start_);
                best_known = from_start_.dist[res_.end];
            }
            if(guarding) dijkstra(res_.end, from_end_);

            int best = -1;
            long best_key = 0;
            bool any = false, timed_out = false;
            for(int u=0;u<n;u++){
                const int k = firstUntried(u);
                if(k < 0) continue;
                any = true;
                if(from_at_.dist[u] >= INF_MM) continue;

                if(pruning){
                    const Node& nu = known_->node(u);
                    const Node& ne = known_->node(res_.end);
                    const int rest = cfg_.manhattan ? manhattanMm(nu, ne) : straightMm(nu, ne);
                    if(from_start_.dist[u] + rest >= best_known) continue;
                }
                if(guarding){
                    const long est = (long)res_.elapsed_ms + estimateMs(from_at_, u)
                                   + estimateMs(from_end_, u) + cfg_.margin_ms;
                    if(est > cfg_.limit_ms){ timed_out = true; continue; }
                }

                // Nearest: least driving, deeper node on ties. Deepest: highest id.
                const long key = cfg_.order == Frontier::Nearest
                               ? (long)from_at_.dist[u] * MAX_NODES + (MAX_NODES - 1 - u)
                               : (long)(MAX_NODES - 1 - u);
                if(best < 0 || key < best_key){ best = u; best_key = key; k_out = k; }
            }
            if(best >= 0){ u_out = best; return Pick::Found; }
            if(!any) return Pick::Nothing;
            return timed_out ? Pick::OutOfTime : Pick::AllPruned;
        }

        int countUnexplored() const {
            int c = 0;
            for(int u=0;u<known_->nodeCount();u++){
                for(int j=0;j<exits_[u];j++) if(!tried_[u][j]) c++;
            }
            return c;
        }

    public:
        template<class W>
        ExploreResult run(W& w, MazeGraph& known, const ExploreConfig& cfg){
            reset(cfg, known);

            if(recognise(w) >= 0 || res_.status != ExploreStatus::Ok){
                // A fresh exploration must start on a fresh map.
                if(res_.status == ExploreStatus::Ok) res_.status = ExploreStatus::WalkerError;
                return res_;
            }
            at_ = addPlace(w);
            if(at_ < 0) return res_;
            res_.start = at_;

            bool settled = false;
            while(res_.status == ExploreStatus::Ok){
                if(cfg_.stop_at_end && res_.end_found) break;

                int u = -1, k = -1;
                const Pick p = pickFrontier(u, k);
                if(p != Pick::Found){
                    settled = (p != Pick::OutOfTime);
                    break;
                }
                if(!driveTo(w, u)) break;
                if(!step(w, k)) break;
            }

            if(res_.status == ExploreStatus::Ok && res_.end_found && at_ != res_.end){
                driveTo(w, res_.end);
            }

            res_.unexplored_exits = countUnexplored();
            res_.complete = res_.status == ExploreStatus::Ok && res_.end_found
                          && (settled || res_.unexplored_exits == 0);
            res_.at_end = res_.status == ExploreStatus::Ok && res_.end_found && at_ == res_.end;
            res_.walked_mm = w.totalMm();
            return res_;
        }
};

template<class W>
ExploreResult explore(W& w, MazeGraph& known, const ExploreConfig& cfg){
    Explorer ex;
    return ex.run(w, known, cfg);
}

template<class W>
ExploreResult explore(W& w, MazeGraph& known, bool stop_at_end){
    ExploreConfig cfg;
    cfg.stop_at_end = stop_at_end;
    return explore(w, known, cfg);
}
