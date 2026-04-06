#include <gtest/gtest.h>

#include "g1_nav/frontier_navigation_result_policy.hpp"

TEST(FrontierNavigationResultPolicy, DoesNotBlacklistAbortDuringGoalHandoff)
{
  EXPECT_FALSE(g1_nav::should_blacklist_on_navigation_abort(true, true));
}

TEST(FrontierNavigationResultPolicy, BlacklistsAbortWhenNoHandoffPending)
{
  EXPECT_TRUE(g1_nav::should_blacklist_on_navigation_abort(false, false));
  EXPECT_TRUE(g1_nav::should_blacklist_on_navigation_abort(true, false));
}
