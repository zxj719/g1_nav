#include "g1_nav/frontier_goal_selector.hpp"

namespace g1_nav
{

std::vector<Frontier> suppress_frontiers_by_radius(
  const std::vector<Frontier> & frontiers,
  double)
{
  return frontiers;
}

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView &,
  const GridMapView &,
  const Frontier &,
  const std::pair<double, double> &,
  const FrontierGoalSelectorConfig &)
{
  return std::nullopt;
}

}  // namespace g1_nav
