#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

// for A*
#include <vector>
#include <utility>   // std::pair
#include <algorithm> // std::reverse
#include <limits>    // std::numeric_limits



//////////////////////////////////////////// Class GridPlannerNode START ////////////////////////////////////////////
class GridPlannerNode
{
public:
// ---------------------------- Constructeur ----------------------------
  GridPlannerNode()
  {
    map_sub = nh.subscribe("/map", 1, &GridPlannerNode::mapCallback, this);

    got_map = false;

    // Initialization
    resolution = 0.0;
    origin_x = 0.0;
    origin_y = 0.0;
    width = 0;
    height = 0;
    planned_once = false; // flag to call algo once 

    ROS_INFO("GridPlannerNode started :) waiting for /map ...");
  }
// ---------------------------------------------------------------------
private:
  // ROS stuff
  ros::NodeHandle nh;
  ros::Subscriber map_sub;

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

    ROS_INFO("Map received =) ");
    ROS_INFO("--> resolution: %.3f m/cell", resolution);
    ROS_INFO("--> origin:     (%.3f, %.3f) m", origin_x, origin_y);
    ROS_INFO("--> size:       %d x %d cells", width, height);


    // plath planning and printing if not already done 
    if (planned_once)
      return;

    
    double start_wx = -4.00995779037;
    double start_wy = -0.996777892113;
    double goal_wx  = 4.34944057465;
    double goal_wy  = -0.553416728973;
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
    return std::abs(x1 - x2) + std::abs(y1 - y2);
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

    // 4-connected moves, in order Right, Left, Up, Down 
    int dx[4] = { 1, -1,  0,  0 };
    int dy[4] = { 0,  0,  1, -1 };


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

      // for all (max) 4 possible moves 
      for (int i = 0; i < 4; i++)
      {
        int neighbor_x = current_x + dx[i];
        int neighbor_y = current_y + dy[i];

        // check if neighbor cell is free 
        if (!isCellFree(neighbor_x, neighbor_y))
          continue;

        int neighbor = toIndex(neighbor_x, neighbor_y);
	
	// check if neighbor already seen 
        if (closed[neighbor])
          continue;

        // moving to a neighbor costs 1
        int tentative_g = g_score[current] + 1;
	
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


// ----------------------------------- call_planner_and_print -----------------------------------
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

    if (path_cells.empty())
    {
      ROS_WARN("¡¡¡¡ No path returned by A_Star_Algo !!!!");
      return;
    }

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





























