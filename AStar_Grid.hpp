#pragma once
#include "AStar_Node.hpp"
#include <vector>

/**
 * @brief Represents a 2D grid for A* pathfinding
 */
template<typename CoordType>
class AStar_Grid 
{
public:
    std::vector<std::vector<AStar_Node<CoordType>>> nodes;
    CoordType width;
    CoordType height;
    
    /**
     * @brief Constructor
     */
    AStar_Grid(CoordType w, CoordType h) : width(w), height(h) 
    {
        // Initialize the grid with walkable nodes
        nodes.resize(width);
        for (CoordType x = 0; x < width; ++x) 
        {
            nodes[x].resize(height);
            for (CoordType y = 0; y < height; ++y) 
            {
                nodes[x][y] = AStar_Node<CoordType>(x, y, true);
            }
        }
    }
    
    /**
     * @brief Get a node at the specified position
     */
    AStar_Node<CoordType>& getNode(CoordType x, CoordType y) 
    {
        return nodes[x][y];
    }
    
    /**
     * @brief Get a node at the specified position (const version)
     */
    const AStar_Node<CoordType>& getNode(CoordType x, CoordType y) const 
    {
        return nodes[x][y];
    }
    
    /**
     * @brief Get a node at the specified position using tuple
     */
    AStar_Node<CoordType>& getNode(const std::tuple<CoordType, CoordType>& pos) 
    {
        return nodes[std::get<0>(pos)][std::get<1>(pos)];
    }
    
    /**
     * @brief Check if coordinates are within the grid bounds
     */
    bool isWithinBounds(CoordType x, CoordType y) const 
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    
    /**
     * @brief Check if coordinates are within the grid bounds using tuple
     */
    bool isWithinBounds(const std::tuple<CoordType, CoordType>& pos) const 
    {
        return isWithinBounds(std::get<0>(pos), std::get<1>(pos));
    }
    
    /**
     * @brief Set a node as unwalkable (obstacle)
     */
    void setObstacle(CoordType x, CoordType y) 
    {
        if (isWithinBounds(x, y)) 
        {
            nodes[x][y].setWalkable(false);
        }
    }
    
    /**
     * @brief Set a node as walkable
     */
    void setWalkable(CoordType x, CoordType y) 
    {
        if (isWithinBounds(x, y)) 
        {
            nodes[x][y].setWalkable(true);
        }
    }
    
    /**
     * @brief Get neighboring nodes (4-directional: up, down, left, right)
     */
    std::vector<std::tuple<CoordType, CoordType>> getNeighbors(CoordType x, CoordType y) const 
    {
        std::vector<std::tuple<CoordType, CoordType>> neighbors;
        
        // 4-directional movement only (no diagonals)
        std::vector<std::tuple<CoordType, CoordType>> directions = {
            {0, 1},   // Up
            {0, -1},  // Down  
            {-1, 0},  // Left
            {1, 0}    // Right
        };
        
        for (const auto& [dx, dy] : directions) 
        {
            CoordType newX = x + dx;
            CoordType newY = y + dy;
            
            if (isWithinBounds(newX, newY)) 
            {
                neighbors.push_back(std::make_tuple(newX, newY));
            }
        }
        
        return neighbors;
    }
    
    /**
     * @brief Get neighboring nodes using tuple position
     */
    std::vector<std::tuple<CoordType, CoordType>> getNeighbors(const std::tuple<CoordType, CoordType>& pos) const 
    {
        return getNeighbors(std::get<0>(pos), std::get<1>(pos));
    }
};