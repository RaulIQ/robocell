#include <gtest/gtest.h>

#include "view.hpp"

TEST(View, PrefersTurningTowardAnOffCenterMarker)
{
  const robocell_follow::View view{0.35f, 0.08f};
  EXPECT_LT(view.cost(1.2f, 0.48f, 0.32f, 0.13f), view.cost(1.2f, 0.48f, -0.06f, 0.13f))
    << "turning toward the marker did not lower the cost";
}

TEST(View, PrefersBrakingNearTheEdge)
{
  const robocell_follow::View view{0.33f, 0.09f};
  EXPECT_LT(view.cost(0.91f, -0.46f, -0.04f, 0.03f), view.cost(0.91f, -0.46f, -0.04f, 0.18f))
    << "slowing near the edge did not lower the cost";
}

TEST(View, DoesNotPenalizeDrivingWithACenteredMarker)
{
  const robocell_follow::View view{0.37f, 0.07f};
  EXPECT_FLOAT_EQ(view.cost(1.34f, 0.0f, 0.0f, 0.19f), 0.0f)
    << "moving forward with a centered marker did not leave the cost at zero";
}

TEST(View, PenalizesFacingAwayFromTheMarker)
{
  const robocell_follow::View view{0.34f, 0.08f};
  EXPECT_GT(view.cost(-1.25f, 0.0f, 0.0f, 0.12f), view.cost(1.25f, 0.61f, 0.0f, 0.12f))
    << "a marker behind the camera did not cost more than a visible marker";
}
