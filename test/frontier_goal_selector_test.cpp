#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "g1_nav/frontier_goal_selector.hpp"

namespace
{

g1_nav::GridMapView make_grid(
  const std::vector<int8_t> & cells,
  int width,
  int height,
  double resolution = 0.5)
{
  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  return grid;
}

g1_nav::Frontier make_frontier(
  double x,
  double y,
  double cost,
  double heuristic_distance,
  int size = 10)
{
  g1_nav::Frontier frontier;
  frontier.centroid_x = x;
  frontier.centroid_y = y;
  frontier.cost = cost;
  frontier.heuristic_distance = heuristic_distance;
  frontier.size = size;
  return frontier;
}

g1_nav::FrontierGoalSelectorConfig make_config()
{
  g1_nav::FrontierGoalSelectorConfig config;
  config.snap_radius = 1.0;
  config.goal_clearance_radius = 0.35;
  return config;
}

}  // namespace

TEST(FrontierGoalSelector, UsesAnchorWhenAnchorIsAlreadyAdmissible)
{
  const std::vector<int8_t> grid_cells(25, 0);
  const std::vector<int8_t> global_costmap_cells(25, 0);
  const auto grid = make_grid(grid_cells, 5, 5);
  const auto global_costmap = make_grid(global_costmap_cells, 5, 5);
  const auto frontier = make_frontier(1.5, 1.5, 3.0, 2.0);

  const auto selected = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, {0.0, 0.0}, make_config());

  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected->anchor_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->anchor_xy.second, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 1.5);
}

TEST(FrontierGoalSelector, SnapsToRobotNearestAdmissibleRingCandidate)
{
  const std::vector<int8_t> grid_cells{
    100, 100,   0, 100, 100,
    100, 100, 100, 100, 100,
      0, 100,   0, 100,   0,
    100, 100, 100, 100, 100,
    100, 100,   0, 100, 100,
  };
  const std::vector<int8_t> global_costmap_cells{
    100, 100,   0, 100, 100,
    100, 100, 100, 100, 100,
      0, 100, 100, 100,   0,
    100, 100, 100, 100, 100,
    100, 100, 100, 100, 100,
  };
  const auto grid = make_grid(grid_cells, 5, 5);
  const auto global_costmap = make_grid(global_costmap_cells, 5, 5);
  const auto frontier = make_frontier(1.5, 1.5, 5.0, 4.0);

  auto config = make_config();
  config.goal_clearance_radius = 0.0;

  const auto selected = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, {0.0, 1.5}, config);

  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected->anchor_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->anchor_xy.second, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.first, 0.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 1.5);
}

TEST(FrontierGoalSelector, SuppressesLowerRankedFrontiersInsideSnapRadius)
{
  std::vector<g1_nav::Frontier> frontiers{
    make_frontier(1.0, 1.0, 9.0, 1.0),
    make_frontier(1.6, 1.0, 8.0, 1.1),
    make_frontier(3.5, 3.5, 7.0, 4.0),
  };

  const auto kept = g1_nav::suppress_frontiers_by_radius(frontiers, 1.0);

  ASSERT_EQ(kept.size(), 2u);
  EXPECT_DOUBLE_EQ(kept[0].centroid_x, 1.0);
  EXPECT_DOUBLE_EQ(kept[0].centroid_y, 1.0);
  EXPECT_DOUBLE_EQ(kept[1].centroid_x, 3.5);
  EXPECT_DOUBLE_EQ(kept[1].centroid_y, 3.5);
}
