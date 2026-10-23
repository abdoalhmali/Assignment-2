// find_route.cpp
// CS 456 - Assignment 2: Uninformed Search
//
// This program finds a route between two cities using one of these algorithms:
//   BFS - Breadth First Search
//   DFS - Depth First Search
//   IDT - Iterative Deepening Search
//   UCS - Uniform Cost Search
//   ALL - runs the four algorithms and prints a comparison table
//
// Usage:   find_route algorithm input_filename origin_city destination_city
// Example: find_route UCS input1.txt Bremen Frankfurt

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <algorithm>
#include <chrono>
#include <sstream>
using namespace std;

// A road that leaves a city.
struct Road {
    string toCity;
    int lengthKm;
};

// A node of the search tree. We remember its parent so we can
// rebuild the route when we reach the goal.
struct SearchNode {
    string city;
    int parentIndex;   // position of the parent in the nodes vector (-1 = start)
    int costSoFar;     // total km from the origin to this city
};

// Everything we want to know after running one algorithm.
struct SearchResult {
    string algorithmName;
    bool found = false;
    int totalDistance = 0;
    vector<string> cities;      // cities on the route, in order
    vector<int> legLengths;     // km of each road on the route
    int expandedNodes = 0;      // nodes we took out of the frontier and expanded
    int generatedNodes = 0;     // nodes we created and added to the frontier
    int maxFrontierSize = 0;    // biggest size of the frontier (memory use)
    double timeMicroseconds = 0;
};

// The road map: city name -> list of roads leaving it.
map<string, vector<Road>> roadMap;

// ---------------------------------------------------------------
// Reading the input file
// ---------------------------------------------------------------

// Reads the file. Every line is: city1 city2 distance.
// We stop when we see the word "END" (from "END OF INPUT").
bool loadRoadMap(const string& filename) {
    ifstream file(filename);
    if (!file) return false;

    string cityA, cityB;
    int km;
    while (file >> cityA) {
        if (cityA == "END") break;
        file >> cityB >> km;
        // Roads can be driven in both directions
        roadMap[cityA].push_back({cityB, km});
        roadMap[cityB].push_back({cityA, km});
    }
    return true;
}

// ---------------------------------------------------------------
// Helper: build the route from the goal node
// ---------------------------------------------------------------

// Goes from the goal node back to the start using parentIndex,
// then reverses the list to get the route from origin to goal.
void buildRoute(const vector<SearchNode>& nodes, int goalIndex, SearchResult& result) {
    vector<int> path;
    for (int i = goalIndex; i != -1; i = nodes[i].parentIndex) {
        path.push_back(i);
    }
    reverse(path.begin(), path.end());

    result.found = true;
    result.totalDistance = nodes[goalIndex].costSoFar;
    for (size_t i = 0; i < path.size(); i++) {
        result.cities.push_back(nodes[path[i]].city);
        if (i > 0) {
            result.legLengths.push_back(nodes[path[i]].costSoFar - nodes[path[i - 1]].costSoFar);
        }
    }
}

// ---------------------------------------------------------------
// Algorithm 1: Breadth First Search (uses a queue)
// Expands the shallowest node first. Here every road costs 1,
// so BFS finds the route with the fewest roads.
// ---------------------------------------------------------------
SearchResult breadthFirstSearch(const string& origin, const string& goal) {
    SearchResult result;
    result.algorithmName = "BFS";

    vector<SearchNode> nodes;
    queue<int> frontier;            // holds indexes of nodes
    set<string> visited;            // cities already added to the frontier

    nodes.push_back({origin, -1, 0});
    frontier.push(0);
    visited.insert(origin);
    result.generatedNodes = 1;
    result.maxFrontierSize = 1;

    while (!frontier.empty()) {
        int currentIndex = frontier.front();
        frontier.pop();
        SearchNode current = nodes[currentIndex];
        result.expandedNodes++;

        if (current.city == goal) {
            buildRoute(nodes, currentIndex, result);
            return result;
        }

        for (const Road& road : roadMap[current.city]) {
            if (visited.count(road.toCity)) continue;   // skip cities we already saw
            visited.insert(road.toCity);
            nodes.push_back({road.toCity, currentIndex, current.costSoFar + road.lengthKm});
            frontier.push((int)nodes.size() - 1);
            result.generatedNodes++;
        }
        result.maxFrontierSize = max(result.maxFrontierSize, (int)frontier.size());
    }
    return result;   // not found
}

// ---------------------------------------------------------------
// Algorithm 2: Depth First Search (uses a stack)
// Goes as deep as possible before going back.
// It finds A route, but not always the shortest one.
// ---------------------------------------------------------------
SearchResult depthFirstSearch(const string& origin, const string& goal) {
    SearchResult result;
    result.algorithmName = "DFS";

    vector<SearchNode> nodes;
    stack<int> frontier;
    set<string> expandedCities;     // cities we already expanded

    nodes.push_back({origin, -1, 0});
    frontier.push(0);
    result.generatedNodes = 1;
    result.maxFrontierSize = 1;

    while (!frontier.empty()) {
        int currentIndex = frontier.top();
        frontier.pop();
        SearchNode current = nodes[currentIndex];

        if (expandedCities.count(current.city)) continue;   // already done
        expandedCities.insert(current.city);
        result.expandedNodes++;

        if (current.city == goal) {
            buildRoute(nodes, currentIndex, result);
            return result;
        }

        for (const Road& road : roadMap[current.city]) {
            if (expandedCities.count(road.toCity)) continue;
            nodes.push_back({road.toCity, currentIndex, current.costSoFar + road.lengthKm});
            frontier.push((int)nodes.size() - 1);
            result.generatedNodes++;
        }
        result.maxFrontierSize = max(result.maxFrontierSize, (int)frontier.size());
    }
    return result;   // not found
}

// ---------------------------------------------------------------
// Algorithm 3: Iterative Deepening Search (IDT)
// Runs depth-limited DFS with limit 0, 1, 2, ... until the goal is found.
// It uses little memory like DFS and finds the shallowest route like BFS.
// ---------------------------------------------------------------

// Recursive depth-limited search.
// pathCities / pathLegs hold the route we are currently exploring.
bool depthLimitedSearch(const string& city, const string& goal, int depthLimit,
                        int distanceSoFar, vector<string>& pathCities,
                        vector<int>& pathLegs, SearchResult& result) {
    result.expandedNodes++;
    if (city == goal) {
        result.totalDistance = distanceSoFar;
        return true;
    }
    if (depthLimit == 0) return false;   // cannot go deeper

    for (const Road& road : roadMap[city]) {
        // do not visit a city that is already on the current path (avoids loops)
        if (find(pathCities.begin(), pathCities.end(), road.toCity) != pathCities.end()) continue;

        result.generatedNodes++;
        pathCities.push_back(road.toCity);
        pathLegs.push_back(road.lengthKm);
        result.maxFrontierSize = max(result.maxFrontierSize, (int)pathCities.size());

        if (depthLimitedSearch(road.toCity, goal, depthLimit - 1, distanceSoFar + road.lengthKm,
                               pathCities, pathLegs, result)) {
            return true;
        }
        pathCities.pop_back();   // go back (backtrack)
        pathLegs.pop_back();
    }
    return false;
}

SearchResult iterativeDeepeningSearch(const string& origin, const string& goal) {
    SearchResult result;
    result.algorithmName = "IDT";
    result.generatedNodes = 1;

    // The longest possible route without repeating a city has (number of cities - 1) roads
    int maxDepth = (int)roadMap.size() - 1;

    for (int depthLimit = 0; depthLimit <= maxDepth; depthLimit++) {
        vector<string> pathCities = {origin};
        vector<int> pathLegs;
        if (depthLimitedSearch(origin, goal, depthLimit, 0, pathCities, pathLegs, result)) {
            result.found = true;
            result.cities = pathCities;
            result.legLengths = pathLegs;
            return result;
        }
    }
    return result;   // not found
}

// ---------------------------------------------------------------
// Algorithm 4: Uniform Cost Search (uses a priority queue)
// Always expands the node with the smallest distance from the origin,
// so the first time we reach the goal the route is the shortest one.
// ---------------------------------------------------------------
SearchResult uniformCostSearch(const string& origin, const string& goal) {
    SearchResult result;
    result.algorithmName = "UCS";

    vector<SearchNode> nodes;
    // pair = (cost, node index); greater<> puts the smallest cost on top
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> frontier;
    map<string, int> bestCost;      // best distance found so far to each city
    set<string> expandedCities;

    nodes.push_back({origin, -1, 0});
    frontier.push({0, 0});
    bestCost[origin] = 0;
    result.generatedNodes = 1;
    result.maxFrontierSize = 1;

    while (!frontier.empty()) {
        int currentIndex = frontier.top().second;
        frontier.pop();
        SearchNode current = nodes[currentIndex];

        if (expandedCities.count(current.city)) continue;   // old, worse entry
        expandedCities.insert(current.city);
        result.expandedNodes++;

        if (current.city == goal) {
            buildRoute(nodes, currentIndex, result);
            return result;
        }

        for (const Road& road : roadMap[current.city]) {
            int newCost = current.costSoFar + road.lengthKm;
            // only add the city if this way is better than what we already know
            if (bestCost.count(road.toCity) && bestCost[road.toCity] <= newCost) continue;
            bestCost[road.toCity] = newCost;
            nodes.push_back({road.toCity, currentIndex, newCost});
            frontier.push({newCost, (int)nodes.size() - 1});
            result.generatedNodes++;
        }
        result.maxFrontierSize = max(result.maxFrontierSize, (int)frontier.size());
    }
    return result;   // not found
}

// ---------------------------------------------------------------
// Running an algorithm by name (and measuring the time)
// ---------------------------------------------------------------
SearchResult runAlgorithm(const string& algorithm, const string& origin, const string& goal) {
    SearchResult result;

    // If a city is not in the file, there is no route
    if (!roadMap.count(origin) || !roadMap.count(goal)) {
        result.algorithmName = algorithm;
        return result;
    }

    auto startTime = chrono::high_resolution_clock::now();

    if (algorithm == "BFS")      result = breadthFirstSearch(origin, goal);
    else if (algorithm == "DFS") result = depthFirstSearch(origin, goal);
    else if (algorithm == "IDT") result = iterativeDeepeningSearch(origin, goal);
    else if (algorithm == "UCS") result = uniformCostSearch(origin, goal);

    auto endTime = chrono::high_resolution_clock::now();
    result.timeMicroseconds = chrono::duration<double, micro>(endTime - startTime).count();
    return result;
}

// ---------------------------------------------------------------
// Printing
// ---------------------------------------------------------------

// Prints the result in the format required by the assignment.
void printResult(ostream& out, const SearchResult& result) {
    if (!result.found) {
        out << "distance: infinity" << endl;
        out << "route:" << endl;
        out << "none" << endl;
        return;
    }
    out << "distance: " << result.totalDistance << " km" << endl;
    out << "route:" << endl;
    for (size_t i = 0; i + 1 < result.cities.size(); i++) {
        out << result.cities[i] << " to " << result.cities[i + 1]
             << ", " << result.legLengths[i] << " km" << endl;
    }
}

// Prints a comparison table of all the results.
void printTable(ostream& out, const vector<SearchResult>& results) {
    out << left << setw(10) << "Algorithm"
         << right << setw(15) << "Distance (km)"
         << setw(9) << "Cities"
         << setw(10) << "Expanded"
         << setw(11) << "Generated"
         << setw(14) << "Max frontier"
         << setw(12) << "Time (us)" << endl;
    out << string(81, '-') << endl;

    for (const SearchResult& r : results) {
        out << left << setw(10) << r.algorithmName << right;
        if (r.found) {
            out << setw(15) << r.totalDistance << setw(9) << r.cities.size();
        } else {
            out << setw(15) << "infinity" << setw(9) << "-";
        }
        out << setw(10) << r.expandedNodes
             << setw(11) << r.generatedNodes
             << setw(14) << r.maxFrontierSize
             << setw(12) << fixed << setprecision(1) << r.timeMicroseconds << endl;
    }

    // Also show the route found by each algorithm
    out << endl << "Routes:" << endl;
    for (const SearchResult& r : results) {
        out << r.algorithmName << ": ";
        if (!r.found) {
            out << "none";
        } else {
            for (size_t i = 0; i < r.cities.size(); i++) {
                if (i > 0) out << " -> ";
                out << r.cities[i];
            }
        }
        out << endl;
    }
}

// ---------------------------------------------------------------
// main
// ---------------------------------------------------------------
int main(int argc, char* argv[]) {
    string algorithm, filename, origin, goal;
    bool askedInProgram = false;

    if (argc == 5) {
        algorithm = argv[1];
        filename = argv[2];
        origin = argv[3];
        goal = argv[4];
    } else if (argc == 1) {
        // The program was started without arguments (for example the Run button
        // of an IDE), so we ask the user for the four values.
        askedInProgram = true;
        cout << "Algorithm (BFS, DFS, IDT, UCS or ALL): "; cin >> algorithm;
        cout << "Input file name (example input1.txt): "; cin >> filename;
        cout << "Origin city: "; cin >> origin;
        cout << "Destination city: "; cin >> goal;
    } else {
        cerr << "Usage: find_route algorithm input_filename origin_city destination_city" << endl;
        return 1;
    }

    if (!loadRoadMap(filename)) {
        cerr << "Cannot open file: " << filename << endl;
        return 1;
    }

    // We build the whole output in a string first,
    // then we print it on the screen AND save it in output.txt
    ostringstream output;

    if (algorithm == "ALL") {
        vector<SearchResult> results;
        for (string name : {"BFS", "DFS", "IDT", "UCS"}) {
            results.push_back(runAlgorithm(name, origin, goal));
        }
        printTable(output, results);
    } else if (algorithm == "BFS" || algorithm == "DFS" ||
               algorithm == "IDT" || algorithm == "UCS") {
        SearchResult result = runAlgorithm(algorithm, origin, goal);
        printResult(output, result);
        output << "expanded nodes: " << result.expandedNodes << endl;
        output << "generated nodes: " << result.generatedNodes << endl;
    } else {
        cerr << "Unknown algorithm. Use BFS, DFS, IDT, UCS or ALL." << endl;
        return 1;
    }

    cout << output.str();

    ofstream outputFile("output.txt");
    outputFile << output.str();
    outputFile.close();
    cout << endl << "(the result was also saved in output.txt)" << endl;

    if (askedInProgram) {
        cout << "Press Enter to close...";
        cin.ignore();
        cin.get();
    }
    return 0;
}