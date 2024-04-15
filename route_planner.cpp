#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
  	RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  	return node->distance((*RoutePlanner::end_node));
}


// Expand the current node by adding all unvisitedneighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  	
  	for(auto neighbor: current_node->neighbors){
      neighbor->parent = current_node;
      neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
      neighbor->g_value = neighbor->distance((*current_node)) + current_node->g_value;
      RoutePlanner::open_list.push_back(neighbor);
      neighbor->visited = true;
    }
}


// Sort the open list and return the next node.
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b){
	float f1 = a->h_value + a->g_value;
  	float f2 = b->h_value + b->g_value;
  	return f1 > f2;
}
RouteModel::Node *RoutePlanner::NextNode() {
  	sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(), Compare);
  	RouteModel::Node* lowest_cost_node = RoutePlanner::open_list.back();
  	RoutePlanner::open_list.pop_back();
  	return lowest_cost_node;
}


//  ConstructFinalPath method to return the final path found from the A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  	while((current_node->x != RoutePlanner::start_node->x) ||(current_node-> y != RoutePlanner::start_node->y)){
      path_found.push_back((*current_node));
      distance += current_node->distance((*current_node->parent));
      current_node = current_node->parent;
    }
	path_found.push_back((*RoutePlanner::start_node));
  
  	// Path order correction
  	int n = path_found.size();
  	for (int i = 0; i < (n/2); i++){
      RouteModel::Node temp = path_found[i];
      path_found[i] = path_found[n-i-1];
      path_found[n-i-1] = temp;
    }
  	
    distance *= m_Model.MetricScale();
    return path_found;

}


// Main A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  	RoutePlanner::open_list.push_back(RoutePlanner::start_node);   
  	while(RoutePlanner::open_list.size()!=0){
      current_node = RoutePlanner::NextNode();
      if((current_node->x == RoutePlanner::end_node->x)&&(current_node->y == RoutePlanner::end_node->y)){
        m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
        return;
      }else{
        RoutePlanner::AddNeighbors(current_node);
      }
    }
}
