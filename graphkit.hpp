#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <map>
#include <list>
#include <queue>
#include <stack>
#include <set>
#include <string>
#include <limits>
#include <stdexcept>
#include <algorithm>

namespace gk {

enum class GraphType {
    DIRECTED,
    UNDIRECTED
};

template <typename T>
struct TraversalResult {
    std::vector<T> order;
    std::map<T, T> parent;
};

template <typename T>
struct MSTResult {
    long long totalCost;
    std::vector<std::pair<T, T>> edges;
};

template <typename T>
class Graph {
private:
    GraphType graphType;
    std::map<T, std::vector<std::pair<T, int>>> adj;
    bool hasNegativeWeights = false;

    long long dijkstra(T start, T end) const {
        std::map<T, long long> dist;
        for(const auto& pair : adj) {
            dist[pair.first] = std::numeric_limits<long long>::max();
        }
        dist[start] = 0;
        std::priority_queue<std::pair<long long, T>, std::vector<std::pair<long long, T>>, std::greater<std::pair<long long, T>>> pq;
        pq.push({0, start});

        while(!pq.empty()) {
            long long d = pq.top().first;
            T u = pq.top().second;
            pq.pop();

            if(d > dist[u]) continue;
            if(u == end) return d;

            if(adj.find(u) != adj.end()) {
                for(const auto& edge : adj.at(u)) {
                    T v = edge.first;
                    int wt = edge.second;

                    if(dist[u] + wt < dist[v]) {
                        dist[v] = dist[u] + wt;
                        pq.push({dist[v], v});
                    }
                }
            }
        }
        return -1;
    }

    long long bellmanFord(T start, T end) const {
        std::map<T, long long> dist;
        for (const auto& pair : adj) {
            dist[pair.first] = std::numeric_limits<long long>::max();
        }
        dist[start] = 0;

        size_t V = adj.size();
        for (size_t i = 1; i <= V - 1; i++) {
            for (const auto& pair : adj) {
                T u = pair.first;
                for (const auto& edge : pair.second) {
                    T v = edge.first;
                    int weight = edge.second;
                    if (dist[u] != std::numeric_limits<long long>::max() && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }

        for (const auto& pair : adj) {
            T u = pair.first;
            for (const auto& edge : pair.second) {
                T v = edge.first;
                int weight = edge.second;
                if (dist[u] != std::numeric_limits<long long>::max() && dist[u] + weight < dist[v]) {
                    throw std::runtime_error("Negative Weight Cycle Detected");
                }
            }
        }

        if (dist[end] == std::numeric_limits<long long>::max()) return -1;
        return dist[end];
    }

    std::size_t countUndirectedComponents() const {
        std::size_t count = 0;
        std::set<T> visited;

        for (const auto& pair : adj) {
            T startNode = pair.first;
            
            if (visited.find(startNode) == visited.end()) {
                count++;
                
                std::queue<T> q;
                q.push(startNode);
                visited.insert(startNode);

                while (!q.empty()) {
                    T u = q.front();
                    q.pop();

                    if (adj.find(u) != adj.end()) {
                        for (const auto& edge : adj.at(u)) {
                            T v = edge.first;
                            if (visited.find(v) == visited.end()) {
                                visited.insert(v);
                                q.push(v);
                            }
                        }
                    }
                }
            }
        }
        return count;
    }

    std::size_t countStronglyConnectedComponents() const {
        std::stack<T> stack;
        std::set<T> visited;

        auto fillOrder = [&](auto&& self, T u) -> void {
            visited.insert(u);
            if (adj.find(u) != adj.end()) {
                for (const auto& edge : adj.at(u)) {
                    T v = edge.first;
                    if (visited.find(v) == visited.end()) {
                        self(self, v);
                    }
                }
            }
            stack.push(u);
        };

        for (const auto& pair : adj) {
            if (visited.find(pair.first) == visited.end()) {
                fillOrder(fillOrder, pair.first);
            }
        }

        std::map<T, std::vector<T>> adjRev;
        for (const auto& pair : adj) {
            T u = pair.first;
            for (const auto& edge : pair.second) {
                T v = edge.first;
                adjRev[v].push_back(u);
            }
        }

        auto dfsRev = [&](auto&& self, T u) -> void {
            visited.insert(u);
            if (adjRev.find(u) != adjRev.end()) {
                for (const T& v : adjRev[u]) {
                    if (visited.find(v) == visited.end()) {
                        self(self, v);
                    }
                }
            }
        };

        visited.clear();
        std::size_t count = 0;

        while (!stack.empty()) {
            T u = stack.top();
            stack.pop();

            if (visited.find(u) == visited.end()) {
                count++;
                dfsRev(dfsRev, u);
            }
        }
        return count;
    }

public:
    explicit Graph(GraphType gType = GraphType::DIRECTED) 
        : graphType(gType) {}

    void addEdge(T u, T v, int w = 1) {
        if (adj.find(u) == adj.end()) adj[u] = {};
        if (adj.find(v) == adj.end()) adj[v] = {};

        adj[u].push_back({v, w});

        if (graphType == GraphType::UNDIRECTED) {
            adj[v].push_back({u, w});
        }

        if(w < 0) hasNegativeWeights = true;
    }

    [[nodiscard]] TraversalResult<T> bfs(T start) const {
        if (adj.find(start) == adj.end()) {
            throw std::out_of_range("Start node not found in graph");
        }

        std::vector<T> order;
        std::map<T, T> parent;
        std::set<T> visited;
        std::queue<T> q;

        visited.insert(start);
        q.push(start);
        parent[start] = T{}; 

        while (!q.empty()) {
            T u = q.front();
            q.pop();
            order.push_back(u);

            if (adj.find(u) != adj.end()) {
                for (const auto& edge : adj.at(u)) {
                    T v = edge.first;
                    if (visited.find(v) == visited.end()) {
                        visited.insert(v);
                        parent[v] = u;
                        q.push(v);
                    }
                }
            }
        }
        return {order, parent};
    }

    [[nodiscard]] TraversalResult<T> dfs(T start) const {
        if (adj.find(start) == adj.end()) {
            throw std::out_of_range("Start node not found in graph");
        }

        std::vector<T> order;
        std::map<T, T> parent;
        std::set<T> visited;
        std::stack<T> s;

        s.push(start);
        parent[start] = T{};

        while (!s.empty()) {
            T u = s.top();
            s.pop();

            if (visited.find(u) == visited.end()) {
                visited.insert(u);
                order.push_back(u);

                if (adj.find(u) != adj.end()) {
                    const auto& neighbors = adj.at(u);
                    for (auto it = neighbors.rbegin(); it != neighbors.rend(); ++it) {
                        T v = it->first;
                        if (visited.find(v) == visited.end()) {
                            parent[v] = u;
                            s.push(v);
                        }
                    }
                }
            }
        }
        return {order, parent};
    }

    [[nodiscard]] bool hasCycle() const {
        if (graphType == GraphType::UNDIRECTED) {
            std::set<T> visited;
            std::map<T, T> parent;
            std::queue<T> q;

            for (const auto& pair : adj) {
                T startNode = pair.first;
                if (visited.find(startNode) != visited.end()) continue;

                visited.insert(startNode);
                q.push(startNode);
                parent[startNode] = T{};

                while (!q.empty()) {
                    T u = q.front();
                    q.pop();

                    for (const auto& edge : adj.at(u)) {
                        T v = edge.first;

                        if (visited.find(v) == visited.end()) {
                            visited.insert(v);
                            parent[v] = u;
                            q.push(v);
                        } else if (parent[u] != v) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        std::map<T, int> state; 
        std::stack<T> st;

        for (const auto& pair : adj) {
            T startNode = pair.first;
            if (state[startNode] != 0) continue;

            st.push(startNode);

            while (!st.empty()) {
                T u = st.top();

                if (state[u] == 0) {
                    state[u] = 1;
                }

                bool pushed = false;
                if (adj.find(u) != adj.end()) {
                    for (const auto& edge : adj.at(u)) {
                        T v = edge.first;
                        if (state[v] == 1) return true;
                        if (state[v] == 0) {
                            st.push(v);
                            pushed = true;
                            break;
                        }
                    }
                }

                if (!pushed) {
                    state[u] = 2;
                    st.pop();
                }
            }
        }
        return false;
    }

    [[nodiscard]] std::vector<T> getNeighbors(T u) const {
        if (adj.find(u) == adj.end()) {
            throw std::out_of_range("Node not found");
        }

        std::vector<T> neighbors;
        for (const auto& edge : adj.at(u)) {
            neighbors.push_back(edge.first);
        }
        return neighbors;
    }

    [[nodiscard]] int indegree(T u) const {
        if (adj.find(u) == adj.end()) {
            throw std::out_of_range("Node not found");
        }

        int inDeg = 0;
        for (const auto& pair : adj) {
            for (const auto& edge : pair.second) {
                if (edge.first == u) {
                    inDeg++;
                }
            }
        }
        return inDeg;
    }

    [[nodiscard]] int outdegree(T u) const {
        if (adj.find(u) == adj.end()) {
            throw std::out_of_range("Node not found");
        }
        return static_cast<int>(adj.at(u).size());
    }

    [[nodiscard]] std::vector<T> topologicalSort() const {
        if (graphType == GraphType::UNDIRECTED) {
            throw std::logic_error("Topological sort is not defined for undirected graphs");
        }

        if (hasCycle()) {
            throw std::logic_error("Graph contains a cycle; topological sort not possible");
        }

        std::vector<T> topo;
        std::map<T, int> inDeg;

        for (const auto& pair : adj) {
            inDeg[pair.first] = 0;
        }

        for (const auto& pair : adj) {
            for (const auto& edge : pair.second) {
                inDeg[edge.first]++;
            }
        }

        std::queue<T> q;
        for (const auto& pair : inDeg) {
            if (pair.second == 0) {
                q.push(pair.first);
            }
        }

        while (!q.empty()) {
            T u = q.front();
            q.pop();
            topo.push_back(u);

            if (adj.find(u) != adj.end()) {
                for (const auto& edge : adj.at(u)) {
                    T v = edge.first;
                    inDeg[v]--;
                    if (inDeg[v] == 0) {
                        q.push(v);
                    }
                }
            }
        }
        return topo;
    }
    
    [[nodiscard]] std::size_t size() const {
        return adj.size();
    }

    [[nodiscard]] long long getShortestPath(T start, T end) const {
        if (hasNegativeWeights) {
            return bellmanFord(start, end);
        } else {
            return dijkstra(start, end);
        }
    }

    [[nodiscard]] std::size_t countComponents() const {
        if (graphType == GraphType::UNDIRECTED) {
            return countUndirectedComponents();
        } else {
            return countStronglyConnectedComponents();
        }
    }

    [[nodiscard]] std::map<T, std::map<T, long long>> getAllPairsShortestPaths() const {
        std::map<T, std::map<T, long long>> dist;
        long long INF = std::numeric_limits<long long>::max();

        for (const auto& p1 : adj) {
            for (const auto& p2 : adj) {
                if (p1.first == p2.first) {
                    dist[p1.first][p2.first] = 0;
                } else {
                    dist[p1.first][p2.first] = INF;
                }
            }
        }

        for (const auto& pair : adj) {
            T u = pair.first;
            for (const auto& edge : pair.second) {
                dist[u][edge.first] = edge.second;
            }
        }

        for (const auto& k_pair : adj) {
            T k = k_pair.first;
            for (const auto& i_pair : adj) {
                T i = i_pair.first;
                for (const auto& j_pair : adj) {
                    T j = j_pair.first;

                    long long d_ik = dist[i][k];
                    long long d_kj = dist[k][j];

                    if (d_ik != INF && d_kj != INF && d_ik + d_kj < dist[i][j]) {
                        dist[i][j] = d_ik + d_kj;
                    }
                }
            }
        }
        return dist;
    }

    [[nodiscard]] MSTResult<T> getMinimumSpanningTree() const {
        MSTResult<T> result;
        result.totalCost = 0;
        
        if (adj.empty()) return result;

        std::map<T, long long> key;
        std::map<T, T> parent;
        std::map<T, bool> inMST;

        for (const auto& pair : adj) {
            key[pair.first] = std::numeric_limits<long long>::max();
            inMST[pair.first] = false;
        }

        T startNode = adj.begin()->first;
        key[startNode] = 0;
        
        std::priority_queue<std::pair<long long, T>, std::vector<std::pair<long long, T>>, std::greater<std::pair<long long, T>>> pq;
        pq.push({0, startNode});

        while (!pq.empty()) {
            long long weight = pq.top().first;
            T u = pq.top().second;
            pq.pop();

            if (inMST[u]) continue;
            inMST[u] = true;

            if (u != startNode) {
                result.totalCost += weight;
                result.edges.push_back({parent[u], u});
            }

            if (adj.find(u) != adj.end()) {
                for (const auto& edge : adj.at(u)) {
                    T v = edge.first;
                    int w = edge.second;

                    if (!inMST[v] && w < key[v]) {
                        key[v] = w;
                        pq.push({key[v], v});
                        parent[v] = u;
                    }
                }
            }
        }
        return result;
    }

    [[nodiscard]] GraphType getType() const { return graphType; }

    [[nodiscard]] const std::map<T, std::vector<std::pair<T, int>>>& getAdjacencyList() const { 
        return adj; 
    }
};
}

#endif