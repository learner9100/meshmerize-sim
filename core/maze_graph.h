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
};

class MazeGraph{
    private:
        Node nodes_[MAX_NODES];
        Edge edges_[MAX_EDGES];
        int node_count_ = 0;
        int edge_count_ = 0;

        bool validNode(int id) const {return id >=0 && id<node_count_;}


    public:
        int addNode(NodeType type){
            if (node_count_ >= MAX_NODES) return -1;
            nodes_[node_count_]=Node{};
            nodes_[node_count_].type = type;
            return node_count_++;
        }

        int addEdge(int a, int b, int length_mm){
            if(!validNode(a) || !validNode(b)) return -1;
            if (a==b) return -1;
            if (length_mm <= 0) return -1;
            if (edge_count_ >= MAX_EDGES) return -1;
            if (nodes_[a].exit_count >=MAX_EXITS) return -1;
            if (nodes_[b].exit_count >=MAX_EXITS) return -1;

            const int idx = edge_count_++;
            edges_[idx]=Edge{a, b, length_mm};
            nodes_[a].exit_edge[nodes_[a].exit_count++] = idx;
            nodes_[b].exit_edge[nodes_[b].exit_count++] = idx;
            return idx;
        }

        int otherEnd(int edge_idx, int from_node) const {
            if (edge_idx < 0 || edge_idx >= edge_count_) return -1;
            const Edge& e = edges_[edge_idx];
            if (e.a == from_node) return e.b;
            if (e.b == from_node) return e.a;
            return -1;
        }

        int nodeCount() const {return node_count_;}
        int edgeCount() const {return edge_count_;}
        const Node& node(int id) const {return nodes_[id];}
        const Edge& edge(int id) const {return edges_[id];}


};
