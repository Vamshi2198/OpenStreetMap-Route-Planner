#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Using the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes found in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);


}


// The CalculateHValue method uses distance method from Node objects to determine the distance to another node.
// Her distance to the end_node is taken to calculate h value.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);

}


// The AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
// Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

// For each node in current_node.neighbors, set the parent, the h_value, the g_value.
  for (auto node : current_node->neighbors) {
    node->parent = current_node;
    node->g_value = current_node->g_value + current_node->distance(*node);
// Use CalculateHValue below to implement the h-Value calculation.
    node->h_value = CalculateHValue(node);
// For each node in current_node.neighbors, add the neighbor to open_list
    open_list.push_back(node);
// set the node's visited attribute to true.
    node->visited = true;
  }
}



// - Sort the open_list according to the sum of the h value and g value.

bool Compare(RouteModel::Node *a,RouteModel::Node *b){
  float f1 = a->g_value + a->h_value;  
  float f2 = b->g_value + b->h_value;  
  if(f1 > f2){
     return true;
  }
   else{
     return false;
   }
}

// The NextNode method sorts the open list and returns the next node.
 
  RouteModel::Node *RoutePlanner::NextNode() {
  sort(open_list.begin(), open_list.end(),Compare);
// - Create a pointer to the node in the list with the lowest sum.
  RouteModel::Node *lowest_sum = open_list.back();
// - Remove that node from the open_list.
  open_list.pop_back();
// - Return the pointer.
  return lowest_sum;

}


// The ConstructFinalPath method returns the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
// While loop iterates through the 
//   chain of parents of nodes until the starting node is found.
     while(current_node != start_node){
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
// reversing the vector such that the start node should be the first element
//   of the vector, the end node should be the last element.
    reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}