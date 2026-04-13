/*#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
#include <iostream>

// Simple structs
struct Node {
  int id;
  double x, y;
};

struct Edge {
  int from, to;
  double cost;
};

static double dist2d(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return std::sqrt(dx*dx + dy*dy);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");  // private namespace

  int start_id = 0, goal_id = 4;
  pnh.param("start_id", start_id, 0);
  pnh.param("goal_id",  goal_id,  4);

  // ----- Load /graph/waypoints and /graph/edges from parameter server
  XmlRpc::XmlRpcValue wp_list, edge_list;
  if (!nh.getParam("/graph/waypoints", wp_list)) {
    ROS_ERROR("Missing param: /graph/waypoints. Did you run: rosparam load config/waypoints.yaml /graph ?");
    return 1;
  }
  if (!nh.getParam("/graph/edges", edge_list)) {
    ROS_ERROR("Missing param: /graph/edges. Did you run: rosparam load config/waypoints.yaml /graph ?");
    return 1;
  }

  if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("/graph/waypoints is not a list.");
    return 1;
  }
  if (edge_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("/graph/edges is not a list.");
    return 1;
  }

  // ----- Parse waypoints
  std::unordered_map<int, Node> nodes;
  nodes.reserve(wp_list.size());

  for (int i = 0; i < wp_list.size(); i++) {
    auto& w = wp_list[i];
    const int id = static_cast<int>(w["id"]);
    const double x = static_cast<double>(w["x"]);
    const double y = static_cast<double>(w["y"]);
    nodes[id] = Node{id, x, y};
  }

  if (!nodes.count(start_id) || !nodes.count(goal_id)) {
    ROS_ERROR("Start or goal id not found in waypoints. start=%d goal=%d", start_id, goal_id);
    return 1;
  }

  // ----- Build adjacency list
  // We'll compute cost if not present in YAML.
  std::unordered_map<int, std::vector<std::pair<int, double>>> adj;
  adj.reserve(nodes.size());

  for (int i = 0; i < edge_list.size(); i++) {
    auto& e = edge_list[i];
    const int from = static_cast<int>(e["from"]);
    const int to   = static_cast<int>(e["to"]);

    if (!nodes.count(from) || !nodes.count(to)) {
      ROS_WARN("Edge references unknown node: %d -> %d (skipping)", from, to);
      continue;
    }

    double cost = 0.0;
    // If YAML edge has "cost", use it; otherwise compute Euclidean distance.
    if (e.hasMember("cost")) {
      cost = static_cast<double>(e["cost"]);
    } else {
      const auto& a = nodes[from];
      const auto& b = nodes[to];
      cost = dist2d(a.x, a.y, b.x, b.y);
    }

    // Undirected edges (typical for waypoint graphs)
    adj[from].push_back({to, cost});
    adj[to].push_back({from, cost});
  }

  // ----- Dijkstra
  const double INF = std::numeric_limits<double>::infinity();
  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> parent;

  for (const auto& kv : nodes) dist[kv.first] = INF;
  dist[start_id] = 0.0;
  parent[start_id] = start_id;

  using QItem = std::pair<double, int>; // (dist, node_id)
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
  pq.push({0.0, start_id});

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (d > dist[u]) continue; // stale
    if (u == goal_id) break;

    for (const auto& [v, w] : adj[u]) {
      const double nd = d + w;
      if (nd < dist[v]) {
        dist[v] = nd;
        parent[v] = u;
        pq.push({nd, v});
      }
    }
  }

  if (!parent.count(goal_id)) {
    ROS_ERROR("No path found from %d to %d (graph disconnected?)", start_id, goal_id);
    return 1;
  }

  // ----- Reconstruct path
  std::vector<int> path;
  for (int cur = goal_id; ; cur = parent[cur]) {
    path.push_back(cur);
    if (cur == start_id) break;
  }
  std::reverse(path.begin(), path.end());

  // Print result
  std::cout << "Planned path " << start_id << " -> " << goal_id << ": ";
  for (size_t i = 0; i < path.size(); i++) {
    std::cout << path[i] << (i + 1 < path.size() ? " -> " : "");
  }
  std::cout << "\nTotal cost: " << dist[goal_id] << " meters\n";

  return 0;
}
*/


#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>

struct Waypoint {
  double x;
  double y;
};

double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph_planner");

  ros::NodeHandle nh;        // normal namespace
  ros::NodeHandle pnh("~");  // private namespace (IMPORTANT for _goal_id)

  // Read start/goal (these will work with _start_id:= and _goal_id:=)
  int start_id = 0;
  int goal_id = 4;
  pnh.param("start_id", start_id, 0);
  pnh.param("goal_id", goal_id, 4);

  // Load graph from ROS params (must be loaded by: rosparam load ... /graph)
  XmlRpc::XmlRpcValue wp_list, edge_list;
  if (!nh.getParam("/graph/waypoints", wp_list) || !nh.getParam("/graph/edges", edge_list)) {
    ROS_ERROR("Missing /graph/waypoints or /graph/edges. Run: rosparam load config/waypoints.yaml /graph");
    return 1;
  }

  // Parse waypoints
  std::unordered_map<int, Waypoint> wp;
  for (int i = 0; i < wp_list.size(); i++) {
    int id = (int)wp_list[i]["id"];
    double x = (double)wp_list[i]["x"];
    double y = (double)wp_list[i]["y"];
    wp[id] = {x, y};
  }

  // Check start/goal exist
  if (wp.find(start_id) == wp.end() || wp.find(goal_id) == wp.end()) {
    ROS_ERROR("Start or goal id not found in waypoints. start=%d goal=%d", start_id, goal_id);
    return 1;
  }

  // Build adjacency list (undirected)
  std::unordered_map<int, std::vector<std::pair<int, double>>> adj;

  for (int i = 0; i < edge_list.size(); i++) {
    int a = (int)edge_list[i]["from"];
    int b = (int)edge_list[i]["to"];

    if (wp.find(a) == wp.end() || wp.find(b) == wp.end()) {
      ROS_WARN("Edge %d-%d refers to unknown waypoint id, skipping.", a, b);
      continue;
    }

    double cost = distance2D(wp[a].x, wp[a].y, wp[b].x, wp[b].y);

    adj[a].push_back({b, cost});
    adj[b].push_back({a, cost});
  }

  // ---- Dijkstra ----
  const double INF = std::numeric_limits<double>::infinity();
  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> parent;

  for (auto& it : wp) dist[it.first] = INF;
  dist[start_id] = 0.0;
  parent[start_id] = start_id;

  using Item = std::pair<double, int>; // (distance, node_id)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  pq.push({0.0, start_id});

  while (!pq.empty()) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();

    if (d > dist[u]) continue;     // ignore old entries
    if (u == goal_id) break;       // reached goal

    for (auto& nb : adj[u]) {
      int v = nb.first;
      double w = nb.second;
      double nd = d + w;

      if (nd < dist[v]) {
        dist[v] = nd;
        parent[v] = u;
        pq.push({nd, v});
      }
    }
  }

  // If goal was never reached
  if (parent.find(goal_id) == parent.end()) {
    ROS_ERROR("No path found from %d to %d", start_id, goal_id);
    return 1;
  }

  // Reconstruct path
  std::vector<int> path;
  int cur = goal_id;
  while (cur != start_id) {
    path.push_back(cur);
    cur = parent[cur];
  }
  path.push_back(start_id);
  std::reverse(path.begin(), path.end());

  // Print
  std::cout << "Planned path " << start_id << " -> " << goal_id << ": ";
  for (size_t i = 0; i < path.size(); i++) {
    std::cout << path[i];
    if (i + 1 < path.size()) std::cout << " -> ";
  }
  std::cout << "\nTotal cost: " << dist[goal_id] << " meters\n";

  return 0;
}
