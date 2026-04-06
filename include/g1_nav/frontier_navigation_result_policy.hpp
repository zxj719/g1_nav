#pragma once

namespace g1_nav
{

inline bool should_blacklist_on_navigation_abort(
  bool preempt_requested,
  bool has_pending_goal)
{
  return !(preempt_requested && has_pending_goal);
}

}  // namespace g1_nav
