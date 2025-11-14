#pragma once
#include "AStar_Node.hpp"
#include "AStar_Grid.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

/**
 * @brief A library for A* pathfinding algorithm
 */
namespace AStarLib
{
    /**
     * @brief Calculate Manhattan distance heuristic (no diagonal movement)
     */
    template<typename CoordType>
    CoordType calculateManhattanDistance(const std::tuple<CoordType, CoordType>& start, 
                                        const std::tuple<CoordType, CoordType>& end)
    {
        CoordType dx = std::abs(std::get<0>(end) - std::get<0>(start));
        CoordType dy = std::abs(std::get<1>(end) - std::get<1>(start));
        return dx + dy;
    }
    

    /**
     * @brief Helper function to convert coordinate tuple to string for hashing
     */
    template<typename CoordType>
    std::string coordToString(const std::tuple<CoordType, CoordType>& coord)
    {
        return std::to_string(std::get<0>(coord)) + "," + std::to_string(std::get<1>(coord));
    }
    

    /**
     * @brief Comparator for priority queue (min-heap based on fCost)
     */
    template<typename CoordType>
    struct NodeComparator 
    {
        AStar_Grid<CoordType>* grid;
        
        NodeComparator(AStar_Grid<CoordType>* g) : grid(g) {}
        
        bool operator()(const std::tuple<CoordType, CoordType>& a, 
                       const std::tuple<CoordType, CoordType>& b) const 
        {
            const auto& nodeA = grid->getNode(a);
            const auto& nodeB = grid->getNode(b);
            
            // If F costs are equal, prefer lower H cost (closer to target)
            if (nodeA.fCost == nodeB.fCost) 
            {
                return nodeA.hCost > nodeB.hCost; // Min-heap, so reverse comparison
            }
            return nodeA.fCost > nodeB.fCost; // Min-heap, so reverse comparison
        }
    };
    
    
    /**
     * @brief Find path using A* algorithm
     * 
     * @param grid The grid to search on
     * @param start Starting position
     * @param end Target position  
     * @return Vector of coordinates representing the path (empty if no path found)
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPath( AStar_Grid<CoordType>& grid,
                                                            const std::tuple<CoordType, CoordType>& start,
                                                            const std::tuple<CoordType, CoordType>& end)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        
        // Check if start and end are valid and walkable
        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end)) 
        {
            return path; // Empty path
        }
        
        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable()) 
        {
            return path; // Empty path
        }
        
        // If start and end are the same
        if (start == end) 
        {
            path.push_back(start);
            return path;
        }
        
        // Initialize start node
        auto& startNode = grid.getNode(start);
        startNode.gCost = 0;
        startNode.hCost = calculateManhattanDistance(start, end);
        startNode.calculateFCost();
        startNode.parent = std::make_tuple(-1, -1); // No parent
        
        // Priority queue for open set (nodes to be evaluated)
        std::priority_queue<    std::tuple<CoordType, CoordType>, 
                                std::vector< std::tuple<CoordType, CoordType> >, 
                                NodeComparator<CoordType>   > openSet{NodeComparator<CoordType>(&grid)};
        
        // Sets to track open and closed nodes
        std::unordered_set<std::string> openSetTracker;
        std::unordered_set<std::string> closedSet;
        
        openSet.push(start);
        openSetTracker.insert(coordToString(start));
        
        while (!openSet.empty()) 
        {
            // Get node with lowest fCost
            auto current = openSet.top();
            openSet.pop();
            
            std::string currentStr = coordToString(current);
            openSetTracker.erase(currentStr);
            closedSet.insert(currentStr);
            
            // Check if we reached the target
            if (current == end) 
            {
                // Reconstruct path
                auto pathNode = current;
                while (!(std::get<0>(grid.getNode(pathNode).parent) == -1 && 
                        std::get<1>(grid.getNode(pathNode).parent) == -1)) 
                {
                    path.push_back(pathNode);
                    pathNode = grid.getNode(pathNode).parent;
                }
                path.push_back(start);
                
                // Reverse to get path from start to end
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Check all neighbors
            auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors) 
            {
                std::string neighborStr = coordToString(neighbor);
                
                // Skip if neighbor is not walkable or already in closed set
                if (!grid.getNode(neighbor).getIsWalkable() || 
                    closedSet.find(neighborStr) != closedSet.end()) 
                {
                    continue;
                }
                
                // Calculate tentative gCost
                CoordType tentativeGCost = grid.getNode(current).gCost + 1; // Cost of 1 for each step
                
                auto& neighborNode = grid.getNode(neighbor);
                
                // If this path to neighbor is better than any previous one
                if (openSetTracker.find(neighborStr) == openSetTracker.end() || 
                    tentativeGCost < neighborNode.gCost) 
                {
                    // This path is the best until now
                    neighborNode.parent = current;
                    neighborNode.gCost = tentativeGCost;
                    neighborNode.hCost = calculateManhattanDistance(neighbor, end);
                    neighborNode.calculateFCost();
                    
                    // Add to open set if not already there
                    if (openSetTracker.find(neighborStr) == openSetTracker.end()) 
                    {
                        openSet.push(neighbor);
                        openSetTracker.insert(neighborStr);
                    }
                }
            }
        }
        
        // No path found
        return path; // Empty path
    }
    

    /**
     * @brief Convenience function to find path between two points on a simple grid
     * 
     * @param width Grid width
     * @param height Grid height
     * @param obstacles Set of obstacle coordinates
     * @param start Starting position
     * @param end Target position
     * @return Vector of coordinates representing the path
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathSimple(
        CoordType width, CoordType height,
        const std::vector<std::tuple<CoordType, CoordType>>& obstacles,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end)
    {
        AStar_Grid<CoordType> grid(width, height);
        
        // Set obstacles
        for (const auto& obstacle : obstacles) 
        {
            grid.setObstacle(std::get<0>(obstacle), std::get<1>(obstacle));
        }
        
        return findPath(grid, start, end);
    }
}