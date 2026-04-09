#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

// for A*
#include <vector>
#include <utility>   // std::pair
#include <algorithm> // std::reverse
#include <limits>    // std::numeric_limits

// for testing with Rviz 
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


// NEW for solving planner pb
#include <tf/tf.h>

//////////////////////////////////////////// Class GridPlannerNode START ////////////////////////////////////////////
class GridPlannerNode
{
public:
// ---------------------------- Constructeur ----------------------------
  GridPlannerNode()
  {
    map_sub = nh.subscribe("/map", 1, &GridPlannerNode::mapCallback, this);
    path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1, true); // for testing with Rviz

    got_map = false;

    // Initialization
    resolution = 0.0;
    origin_x = 0.0;
    origin_y = 0.0;
    width = 0;
    height = 0;
    planned_once = false; // flag to call algo once 

    ROS_INFO("GridPlannerNode started in testing file :) waiting for /map ...");
  }
// ---------------------------------------------------------------------
private:
  // ROS stuff
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Publisher path_pub; // for testing with Rviz

  // Map data
  nav_msgs::OccupancyGrid map;
  bool got_map;

  // Map info 
  double resolution;
  double origin_x;
  double origin_y;
  int width;
  int height;
  
  // Flag (to avoid planning everytime a map message arrives, idea given by ChatGPT)
  bool planned_once;

// ---------------------------- mapCallback ----------------------------
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    map = *msg;
    got_map = true;

    // store map info 
    resolution = map.info.resolution;
    origin_x = map.info.origin.position.x;
    origin_y = map.info.origin.position.y;
    width = (int)map.info.width;
    height = (int)map.info.height;

// NEw for planner pb 
    ROS_INFO("--> origin orientation q: [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
         map.info.origin.orientation.x,
         map.info.origin.orientation.y,
         map.info.origin.orientation.z,
         map.info.origin.orientation.w);
//



    ROS_INFO("Map received =) ");
    ROS_INFO("--> resolution: %.3f m/cell", resolution);
    ROS_INFO("--> origin:     (%.3f, %.3f) m", origin_x, origin_y);
    ROS_INFO("--> size:       %d x %d cells", width, height);


    // plath planning and printing if not already done 
    if (planned_once)
      return;

    // START and GOAL points 

/* {id: 0, x: -4.00995779037, y: -0.996777892113}
- {id: 1, x: 4.34944057465, y: -0.553416728973}
- {id: 2, x: 15.0812244415, y: 0.201877117157}
- {id: 3, x: 14.6693248749, y: 8.143907547}
- {id: 4, x: 17.9891986847, y: 0.419644832611}
*/ 
    double start_wx = -4.00995779037;
    double start_wy = -0.996777892113;
    double goal_wx  = 14.6693248749;
    double goal_wy  = 8.143907547;// 3



    planned_once = true; // set the flag 
    call_planner_and_print_waypoints_World(start_wx, start_wy, goal_wx, goal_wy);
  }

// ---------------------------- toIndex ----------------------------
  int toIndex(int gx, int gy)
  {
    return gy * width + gx;
  }

// ---------------------------- worldToGrid ----------------------------
  bool worldToGrid(double wx, double wy, int &gx, int &gy)
  {
    if (!got_map)
      return false;

    // convert to "map cell coordinates" (floating point)
    double gx_float = (wx - origin_x) / resolution;
    double gy_float = (wy - origin_y) / resolution;

    // convert to integer cell indices
    gx = (int)std::floor(gx_float);
    gy = (int)std::floor(gy_float);

    // bounds check, given by ChatGPT to prevent segmentation faults and planner failures
    if (gx < 0 || gy < 0 || gx >= width || gy >= height)
      return false;

    // no pb faced 
    return true;
  }

// ---------------------------- gridToWorld ----------------------------
  bool gridToWorld(int gx, int gy, double &wx, double &wy)
  {
    if (!got_map)
      return false;

    // bounds check, given by ChatGPT to prevent waypoints outside the map and make debugging easier
    if (gx < 0 || gy < 0 || gx >= width || gy >= height)
      return false;
    
    // calculate cell center
    wx = origin_x + (gx + 0.5) * resolution;
    wy = origin_y + (gy + 0.5) * resolution;

    // no pb faced
    return true;
  }
  

// ---------------------------- isCellFree ----------------------------
// if index value == -1 or >= 50 ------> cell occupied, 
// if index value < 50 ----------------> cell unoccupied

  bool isCellFree(int gx, int gy)
  {
    if (!got_map)
      return false;

    // bounds check, given by ChatGPT to avoid invalid access 
    if (gx < 0 || gy < 0 || gx >= width || gy >= height)
      return false;

    // convert cell into index for map.data
    int index = toIndex(gx,gy);

    // get occupancy value
    int8_t value = map.data[index];

    // value == -1 or >= 50 ------> cell occupied
    if (value == -1 || value >= 50 )
      return false;

    // ------------> cell unoccupied
    return true;
  }

// ---------------------------- heuristic ----------------------------
  // Manhattan heuristic
  int heuristic(int x1, int y1, int x2, int y2)
  {
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    return 10 * (dx + dy) + (14 - 20) * std::min(dx, dy);
  }

// ---------------------------- getBestOpenNode ----------------------------
// get the open node with the minimum f = g + h
  int getBestOpenNode(const std::vector<int> &open, const std::vector<int> &g_score, int goal_x, int goal_y)
  {
    // lets suppose index 0 in open has the lowest f 
    int best_index_in_open = 0;
    int best_x = open[0] % width;
    int best_y = open[0] / width;

    int best_f = g_score[open[0]] + heuristic(best_x, best_y, goal_x, goal_y);

    for (int i = 1; i < (int)open.size(); i++)
    {
      int current_cell_index = open[i];
      int current_x = current_cell_index % width;
      int current_y = current_cell_index / width;

      int current_f = g_score[current_cell_index] + heuristic(current_x, current_y, goal_x, goal_y);

      // check if a lower f has been calculated 
      if (current_f < best_f)
      {
        best_f = current_f;
        best_index_in_open = i;
      }
    }

    return best_index_in_open;
  }


// ----------------------------------- A_Star_Algo -----------------------------------

  std::vector<std::pair<int,int>> A_Star_Algo(int start_x, int start_y, int goal_x,  int goal_y)
  {
    std::vector<std::pair<int,int>> empty_path;

    // ----- checking if got map and correct points ----- 
    if (!got_map)
      return empty_path;

    // check initial and final cells
    if (!isCellFree(start_x, start_y) || !isCellFree(goal_x, goal_y) )
    {
      ROS_WARN("¡¡¡¡ you gave occupied initial or final cell !!!!");
      return empty_path;
    }
   

    // ----- preparation -----
    int total = width * height;
       
    std::vector<int> g_score(total, std::numeric_limits<int>::max()); // cost from start to each cell, initially max (infinity)
 
    std::vector<int> parent(total, -1); // parent index for each cell, initially no parent 
    
    std::vector<int> open; // cells discovered but not processed yet

    std::vector<bool> closed(total, false); // cells already processed

    // get index for start and goal
    int start_index = toIndex(start_x, start_y);
    int goal_index  = toIndex(goal_x, goal_y);

    g_score[start_index] = 0; // because start node 
    open.push_back(start_index); // add to open list 

    // 8-connected moves (4 cardinal + 4 diagonal)
    int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
    int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };


    // ----- algo loop  -----

    while (!open.empty())
    {
      // get best cell from open list
      int best_pos = getBestOpenNode(open, g_score, goal_x, goal_y);
      int current = open[best_pos];

      // remove it from open list
      open.erase(open.begin() + best_pos);

      // cell already seen or not ?   
      if (closed[current])
        continue;

      // add cell to close set 
      closed[current] = true;

      // if this is the goal --> reconstruct the path
      if (current == goal_index)
      {
        std::vector<std::pair<int,int>> path;
        int idx = current;

        while (idx != -1)
        {
          int x = idx % width;
          int y = idx / width;
          path.push_back({x, y});
          idx = parent[idx];
        }
	// put the path in the right order 
        std::reverse(path.begin(), path.end()); 
        return path;
      }

      // expand neighbors
      int current_x = current % width;
      int current_y = current / width;

      // for all (max) 8 possible moves 
      for (int i = 0; i < 8; i++)
      {
        int neighbor_x = current_x + dx[i];
        int neighbor_y = current_y + dy[i];

	// STEP 5: prevent corner cutting
  	if (dx[i] != 0 && dy[i] != 0) {
    	  if (!isCellFree(current_x + dx[i], current_y) || !isCellFree(current_x, current_y + dy[i]))
      	    continue;
  	}

        // check if neighbor cell is free 
        if (!isCellFree(neighbor_x, neighbor_y))
          continue;

        int neighbor = toIndex(neighbor_x, neighbor_y);
	
	// check if neighbor already seen 
        if (closed[neighbor])
          continue;

        // diagonal move-> cost √2 ≈ 1.414
	int move_cost;
	if (dx[i] != 0 && dy[i] != 0)
  	  move_cost = 14;   // diagonal move (≈ 1.4 * 10)
	else
  	  move_cost = 10;   // straight move 

	int tentative_g = g_score[current] + move_cost;
	
	// if moving to neighbor is cheaper than the current g_score
        if (tentative_g < g_score[neighbor])
        {
          g_score[neighbor] = tentative_g; // update neighbor g_score 
          parent[neighbor] = current; // set the parent of neighbor 
          open.push_back(neighbor); // add to open, maybe already in 
        }
      }
    }

    ROS_WARN("A*: No path found for these start and goal points  :((");
    return empty_path;
  }

// ------------------------------------------- simplifyPath ---------------NEW-------------------------

  std::vector<std::pair<int,int>> simplifyPath(const std::vector<std::pair<int,int>> &path)
  {
    if (path.size() < 3)
      return path;

    std::vector<std::pair<int,int>> simplified;
    simplified.push_back(path[0]);

    for (size_t i = 1; i < path.size() - 1; i++)
    {
      int dx1 = path[i].first  - path[i-1].first;
      int dy1 = path[i].second - path[i-1].second;

      int dx2 = path[i+1].first  - path[i].first;
      int dy2 = path[i+1].second - path[i].second;

      // keep point only if direction changes
      if (dx1 != dx2 || dy1 != dy2)
        simplified.push_back(path[i]);
    }

    simplified.push_back(path.back());
    return simplified;
  }


// ----------------------------------- call_planner_and_print_waypoints_World -----------------------------------
  void call_planner_and_print_waypoints_World(double start_wx, double start_wy, double goal_wx, double goal_wy)
  {
    int start_x, start_y, goal_x, goal_y;

    if (!worldToGrid(start_wx, start_wy, start_x, start_y))
    {
      ROS_WARN("¡¡¡¡ Start is outside map !!!!");
      return;
    }

    if (!worldToGrid(goal_wx, goal_wy, goal_x, goal_y))
    {
      ROS_WARN("¡¡¡¡ Goal is outside map !!!!");
      return;
    }

    ROS_INFO("----> You gave Start cell: (%d,%d) and Goal cell: (%d,%d)", start_x, start_y, goal_x, goal_y);

    std::vector<std::pair<int,int>> path_cells = A_Star_Algo(start_x, start_y, goal_x, goal_y);
    path_cells = simplifyPath(path_cells);

    if (path_cells.empty())
    {
      ROS_WARN("¡¡¡¡ No path returned by A_Star_Algo !!!!");
      return;
    }

    // publishing path for Rviz, help by ChatGPT 
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    for (size_t i = 0; i < path_cells.size(); i++)
    {
      geometry_msgs::PoseStamped pose;

      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();

      double wx, wy;
      gridToWorld(path_cells[i].first, path_cells[i].second, wx, wy);

      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
    ROS_INFO("Published path to /planned_path");

    ROS_INFO("A_Star_Algo returned a path of lenght (in cells): %zu", path_cells.size());

    // print the path given by A_Star_Algo in the World Frame 
    for (size_t i = 0; i < path_cells.size(); i++)
    {
      double wx, wy;
      gridToWorld(path_cells[i].first, path_cells[i].second, wx, wy);
      ROS_INFO("  %zu: (%.3f, %.3f)", i, wx, wy);
    }
  }

// ----------------------------------------------------------------------

};

//////////////////////////////////////////// Class GridPlannerNode END ////////////////////////////////////////////








//////////////////////////////////////////// Main function START ////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_planner");
  GridPlannerNode node;
  ros::spin();
  return 0;
}


//////////////////////////////////////////// Main function END ////////////////////////////////////////////





























