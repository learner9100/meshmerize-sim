#pragma once

constexpr int MAX_NODES=256;
constexpr int MAX_EDGES=512;
constexpr int MAX_EXITS=4;

enum class NodeType {
    Normal,
    Start,
    End,
    CheckPoint,
    DeadEnd
};
struct Edge{
    int a;
    int b;
    int length_mm;
};

struct Node{
    NodeType type = NodeType::Normal;
    int exit_count = 0;
    int exit_edge[MAX_EXITS]={};
    int exit_cmd[MAX_EXITS]={-1,-1,-1,-1};   // what to pass to go() for this exit
    int x_mm = 0;
    int y_mm = 0;
};

class MazeGraph{
    private:
        Node nodes_[MAX_NODES];
        Edge edges_[MAX_EDGES];
        int node_count_ = 0;
        int edge_count_ = 0;

        bool validNode(int id) const {return id >=0 && id<node_count_;}

        // N=0 E=2 S=4 W=6, diagonals in between
        int compassKey(int from, int edge_idx) const {
            const Node& a = nodes_[from];
            const Node& b = nodes_[otherEnd(edge_idx, from)];
            const int dx = b.x_mm - a.x_mm;
            const int dy = b.y_mm - a.y_mm;
            if(dx == 0 && dy < 0) return 0;
            if(dx > 0 && dy == 0) return 2;
            if(dx == 0 && dy > 0) return 4;
            if(dx < 0 && dy == 0) return 6;
            if(dx > 0) return dy < 0 ? 1 : 3;
            if(dx < 0) return dy > 0 ? 5 : 7;
            return 8;
        }


    public:
        int addNode(NodeType type){
            if (node_count_ >= MAX_NODES) return -1;
            nodes_[node_count_]=Node{};
            nodes_[node_count_].type = type;
            return node_count_++;
        }

        int addNode(NodeType type, int x_mm, int y_mm){
            const int id = addNode(type);
            if(id >= 0) setPos(id, x_mm, y_mm);
            return id;
        }

        bool setPos(int id, int x_mm, int y_mm){
            if(!validNode(id)) return false;
            nodes_[id].x_mm = x_mm;
            nodes_[id].y_mm = y_mm;
            return true;
        }

        bool setType(int id, NodeType type){
            if(!validNode(id)) return false;
            nodes_[id].type = type;
            return true;
        }

        int addEdge(int a, int b, int length_mm){
            return addEdge(a, b, length_mm, -1, -1);
        }

        int addEdge(int a, int b, int length_mm, int cmd_a, int cmd_b){
            if(!validNode(a) || !validNode(b)) return -1;
            if (a==b) return -1;
            if (length_mm <= 0) return -1;
            if (edge_count_ >= MAX_EDGES) return -1;
            if (nodes_[a].exit_count >=MAX_EXITS) return -1;
            if (nodes_[b].exit_count >=MAX_EXITS) return -1;

            const int idx = edge_count_++;
            edges_[idx]=Edge{a, b, length_mm};
            const int ka = nodes_[a].exit_count++;
            const int kb = nodes_[b].exit_count++;
            nodes_[a].exit_edge[ka] = idx;
            nodes_[b].exit_edge[kb] = idx;
            nodes_[a].exit_cmd[ka] = cmd_a;
            nodes_[b].exit_cmd[kb] = cmd_b;
            return idx;
        }

        int otherEnd(int edge_idx, int from_node) const {
            if (edge_idx < 0 || edge_idx >= edge_count_) return -1;
            const Edge& e = edges_[edge_idx];
            if (e.a == from_node) return e.b;
            if (e.b == from_node) return e.a;
            return -1;
        }

        int exitSlot(int node, int edge_idx) const {
            if(!validNode(node)) return -1;
            const Node& nd = nodes_[node];
            for(int k=0;k<nd.exit_count;k++){
                if(nd.exit_edge[k] == edge_idx) return k;
            }
            return -1;
        }

        int findNode(NodeType type) const {
            for(int i=0;i<node_count_;i++){
                if(nodes_[i].type == type) return i;
            }
            return -1;
        }

        // needs positions; gives the robot a stable exit numbering
        void sortExitsByCompass(){
            for(int i=0;i<node_count_;i++){
                Node& nd = nodes_[i];
                for(int k=1;k<nd.exit_count;k++){
                    const int ek = nd.exit_edge[k];
                    const int ck = nd.exit_cmd[k];
                    const int keyk = compassKey(i, ek);
                    int j = k - 1;
                    while(j >= 0 && compassKey(i, nd.exit_edge[j]) > keyk){
                        nd.exit_edge[j+1] = nd.exit_edge[j];
                        nd.exit_cmd[j+1] = nd.exit_cmd[j];
                        j--;
                    }
                    nd.exit_edge[j+1] = ek;
                    nd.exit_cmd[j+1] = ck;
                }
            }
        }

        int nodeCount() const {return node_count_;}
        int edgeCount() const {return edge_count_;}
        const Node& node(int id) const {return nodes_[id];}
        const Edge& edge(int id) const {return edges_[id];}


};
