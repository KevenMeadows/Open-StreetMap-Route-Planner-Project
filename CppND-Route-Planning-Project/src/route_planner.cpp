#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*end_node); // Distance method
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    //  Find neighbors method to populate node neighbors
    current_node->FindNeighbors();
    // For loop derived from the english in the TODO 4 section
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        // parent->neighbor = current_node;
        // Set parent 
        neighbor->parent = current_node;
        // cout << "This is parent" << neighbor->parent; test
        // Set h
        neighbor->h_value = CalculateHValue(neighbor); // Use CalculateHValue
        // cout << "h: " << neighbor->h_value; test
        // Set g
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor); // Distance method
        // cout << "g: " << neighbor->g_value; test
        // Add neighbor to open_list
        // open_list-> 
        // Not a pointer use push
        open_list.push_back(neighbor);
        // Set visited 
        neighbor->visited = true;
        // cout << "Visted? " << neighbor->visited; test
    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
RouteModel::Node *RoutePlanner::NextNode() 
{
    // Use sort method with autos so we don't need explicit type 
    std:: sort(open_list.begin(), open_list.end(), [](const auto & First, const auto & Second)
    {
        return First->h_value + First->g_value < Second->h_value + Second->g_value; // Sort according to sums
    });

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Sort points to both g and h to add them then compares 
    // them. Then, below, node is created to point lowest to
    // the next creating an order where lowest is added first 
    // then the next lowest and so on. Returns the lowest so
    // it's added first before the next lowest.
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    // Pointer to node in list with lowest sum
    RouteModel::Node *lowest_node = open_list.front(); 
    // Remove node from open_list
    open_list.erase(open_list.begin());
    // cout << "lowest: " << RouteModel->lowest_node; test
    // Return pointer
    return lowest_node;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> FoundPath;
    RouteModel:: Node parent;
    // TODO: Implement your solution here.

    // While loop that keeps going until there is a null pointer (the end)
    while (current_node->parent != nullptr)
    {
        // Current node add to path
        FoundPath.push_back(*current_node);

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Using push back will take the first element pointed 
        // to and add it to the end of the vector, allowing 
        // for the vector group to be ordered first to last.
        // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        parent = *(current_node->parent);
        // Add distance from parent node to distance
        distance += current_node->distance(parent);
        // cout << "Distance: " << distance; test
        // Set current to its parent
        current_node = current_node->parent;
    }
    // First should be start and last the end
    FoundPath.push_back(*current_node);
    // Needs to be reversed
    std:: reverse(std:: begin(FoundPath), std:: end(FoundPath));
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    // Return vector
    return FoundPath; 
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
void RoutePlanner::AStarSearch() 
{
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // Start node points to visited as true since revisiting it would be an extra step.
    start_node->visited = true;
    // Initialize list of open nodes
    open_list.push_back(start_node);
    // While the list of open nodes is non empty
    while(open_list.size() > 0)
    {
        // Get the next node as it's close to the end
        current_node = NextNode();
        // Check if destination was reached, if not continue
        if(current_node->distance(*end_node) == 0)
        {
            // Construct the path
            m_Model.path = ConstructFinalPath(current_node); // Call ConstructFinalPath method
            // Return for exiting
            return;
        }
        else
        {
            // Expand search for current neighboring nodes
            AddNeighbors(current_node);
        }
    }
}