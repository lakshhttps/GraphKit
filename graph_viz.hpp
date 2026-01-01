#ifndef GRAPH_VIZ_HPP
#define GRAPH_VIZ_HPP

#include <string>
#include <sstream>
#include <set>
#include <vector>
#include <iostream>
#include "graphkit.hpp" 

template <typename T>
std::string toStr(const T& value) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}

namespace gk {

template<typename T>
std::string generateGraphDOT(const Graph<T>& g) {
    bool isDirected = (g.getType() == GraphType::DIRECTED);
    std::string dot = isDirected ? "digraph G {\n" : "graph G {\n";
    std::string arrow = isDirected ? " -> " : " -- ";
    
    dot += "  node [shape=circle, style=filled, color=lightblue, fontname=\"Helvetica\"];\n";
    dot += "  edge [fontname=\"Helvetica\", fontsize=10];\n\n";
    
    std::set<std::pair<T, T>> processed; 
    const auto& adjList = g.getAdjacencyList();

    for (const auto& pair : adjList) {
        T u = pair.first;
        if (adjList.at(u).empty()) {
             dot += "  \"" + toStr(u) + "\";\n";
        }
        for (const auto& edge : pair.second) {
            T v = edge.first;
            int w = edge.second;
            if (!isDirected) {
                if (processed.count({v, u})) continue;
                processed.insert({u, v});
            }
            dot += "  \"" + toStr(u) + "\"" + arrow + "\"" + toStr(v) + "\" [label=\"" + std::to_string(w) + "\"];\n";
        }
    }
    dot += "}\n";
    return dot;
}

template<typename T>
std::string generateMSTDOT(const MSTResult<T>& mstResult) {
    std::string dot = "graph MST {\n";
    dot += "  label=\"Minimum Spanning Tree (Total Cost: " + std::to_string(mstResult.totalCost) + ")\";\n";
    dot += "  labelloc=\"t\";\n";
    dot += "  node [shape=circle, style=filled, color=lightgreen, fontname=\"Helvetica\"];\n";
    dot += "  edge [color=red, penwidth=2.0, fontname=\"Helvetica\"];\n\n";

    for (const auto& edge : mstResult.edges) {
        dot += "  \"" + toStr(edge.first) + "\" -- \"" + toStr(edge.second) + "\";\n";
    }
    dot += "}\n";
    return dot;
}

template<typename T>
void printGraph(const Graph<T>& g) {
    std::cout << "\n=== GRAPH VISUALIZATION CODE ===" << std::endl;
    std::cout << generateGraphDOT(g) << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << ">> Paste above code at: https://dreampuf.github.io/GraphvizOnline/" << std::endl;
    std::cout << "================================\n" << std::endl;
}

template<typename T>
void printMST(const MSTResult<T>& mstResult) {
    std::cout << "\n=== MST VISUALIZATION CODE ===" << std::endl;
    std::cout << generateMSTDOT(mstResult) << std::endl;
    std::cout << "==============================" << std::endl;
    std::cout << ">> Paste above code at: https://dreampuf.github.io/GraphvizOnline/" << std::endl;
    std::cout << "==============================\n" << std::endl;
}

}

#endif