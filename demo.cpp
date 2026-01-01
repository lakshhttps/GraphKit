#include <iostream>
#include <string>
#include <vector>
#include "graphkit.hpp"
#include "graph_viz.hpp"

using namespace gk;

// --- Scenario 1: City Navigation (Undirected Graph) ---
void runCityDemo() {
    std::cout << "\n============================================\n";
    std::cout << "   DEMO 1: CITY NAVIGATION SYSTEM \n";
    std::cout << "============================================\n";

    // 1. Create a map
    Graph<std::string> cityMap(GraphType::UNDIRECTED);
    
    std::cout << "Building the road network...\n";
    cityMap.addEdge("Jalandhar", "Ludhiana", 60);
    cityMap.addEdge("Jalandhar", "Amritsar", 80);
    cityMap.addEdge("Ludhiana", "Chandigarh", 100);
    cityMap.addEdge("Amritsar", "Chandigarh", 220);
    cityMap.addEdge("Ludhiana", "Delhi", 310);
    cityMap.addEdge("Chandigarh", "Delhi", 250);

    // 2. Calculate Distances
    std::cout << "\nFinding shortest distance from Jalandhar to Delhi...\n";
    long long distance = cityMap.getShortestPath("Jalandhar", "Delhi");
    std::cout << ">> Result: " << distance << " km\n";

    // 3. Find the Minimum Spanning Tree
    std::cout << "\nCalculating cheapest way to connect all cities...\n";
    auto mst = cityMap.getMinimumSpanningTree();
    std::cout << ">> Total Wire Length Needed: " << mst.totalCost << " km\n";

    // 4. Visualizations
    std::cout << "\n--- Visualizing the Network ---";
    printGraph(cityMap);
    
    std::cout << "\n--- Visualizing the MST (Optimized Network) ---";
    printMST(mst);
}

// --- Scenario 2: Course Prerequisites (Directed Graph) ---
void runCourseDemo() {
    std::cout << "\n\n============================================\n";
    std::cout << "   DEMO 2: UNIVERSITY COURSE PLANNER \n";
    std::cout << "============================================\n";

    Graph<std::string> courses(GraphType::DIRECTED);

    std::cout << "Adding course prerequisites...\n";
    courses.addEdge("Calculus", "Physics");
    courses.addEdge("Calculus", "Linear Algebra");
    courses.addEdge("Physics", "Electronics");
    courses.addEdge("Linear Algebra", "AI");
    courses.addEdge("AI", "Robotics");

    // 1. Analyze Dependencies
    std::cout << "\nCounting courses that depend on Calculus directly...\n";
    std::cout << ">> Result: " << courses.outdegree("Calculus") << " courses\n";

    // 2. Generate Study Plan (Topological Sort)
    std::cout << "\nDetermining the correct order to study these subjects...\n";
    std::vector<std::string> studyPlan = courses.topologicalSort();
    
    std::cout << ">> Recommended Order: \n   ";
    for (const auto& subject : studyPlan) {
        std::cout << subject << " -> ";
    }

    // 3. Visualization
    std::cout << "\n--- Visualizing the Course Map ---";
    printGraph(courses);
}

int main() {
    runCityDemo();
    runCourseDemo();

    std::cout << "\n\nPress Enter to exit...";
    std::cin.get(); 
    return 0;
}