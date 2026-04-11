#include "ris/navigation/grid_utils.h"
#include <costmap_2d/cost_values.h>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace ris_navigation
{
bool worldToMap(const costmap_2d::Costmap2D& costmap, double wx, double wy, unsigned int& mx, unsigned int& my)
{
  return costmap.worldToMap(wx, wy, mx, my);
}

void mapToWorld(const costmap_2d::Costmap2D& costmap, unsigned int mx, unsigned int my, double& wx, double& wy)
{
  costmap.mapToWorld(mx, my, wx, wy);
}

bool isCellTraversable(
    const costmap_2d::Costmap2D& costmap,
    unsigned int mx,
    unsigned int my,
    unsigned char lethal_threshold,
    bool allow_unknown)
{
  if (mx >= costmap.getSizeInCellsX() || my >= costmap.getSizeInCellsY())
    return false;

  const unsigned char cost = costmap.getCost(mx, my);

  if (cost == costmap_2d::NO_INFORMATION)
    return allow_unknown;

  return cost < lethal_threshold;
}

bool findNearestValidCell(
    const costmap_2d::Costmap2D& costmap,
    unsigned int start_mx,
    unsigned int start_my,
    double max_radius_m,
    unsigned char lethal_threshold,
    bool allow_unknown,
    unsigned int& result_mx,
    unsigned int& result_my)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  if (start_mx >= size_x || start_my >= size_y)
    return false;

  const double resolution = costmap.getResolution();
  const int max_radius_cells = static_cast<int>(max_radius_m / resolution);

  std::queue<std::pair<unsigned int, unsigned int>> q;
  std::set<std::pair<unsigned int, unsigned int>> visited;

  q.push({start_mx, start_my});
  visited.insert({start_mx, start_my});

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!q.empty())
  {
    const auto current = q.front();
    q.pop();

    const unsigned int cx = current.first;
    const unsigned int cy = current.second;

    const int dist_x = static_cast<int>(cx) - static_cast<int>(start_mx);
    const int dist_y = static_cast<int>(cy) - static_cast<int>(start_my);
    if ((dist_x * dist_x + dist_y * dist_y) > (max_radius_cells * max_radius_cells))
      continue;

    if (isCellTraversable(costmap, cx, cy, lethal_threshold, allow_unknown))
    {
      result_mx = cx;
      result_my = cy;
      return true;
    }

    for (int i = 0; i < 8; ++i)
    {
      const int nx = static_cast<int>(cx) + dx[i];
      const int ny = static_cast<int>(cy) + dy[i];

      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y))
        continue;

      std::pair<unsigned int, unsigned int> next(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      if (visited.insert(next).second)
        q.push(next);
    }
  }

  return false;
}
}