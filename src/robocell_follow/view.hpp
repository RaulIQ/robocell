#ifndef ROBOCELL_FOLLOW_VIEW_HPP_
#define ROBOCELL_FOLLOW_VIEW_HPP_

#include <algorithm>
#include <cmath>

namespace robocell_follow
{

/** A camera view whose edges penalize trajectories that keep moving forward. */
struct View final
{
  const float margin;
  const float offset;

  float cost(float x, float y, float yaw, float velocity) const
  {
    const float cosine = std::cos(yaw);
    const float sine = std::sin(yaw);
    const float forward = x * cosine + y * sine - offset;
    const float lateral = y * cosine - x * sine;
    const float angle = std::atan2(std::abs(lateral), forward);
    const float excess = std::max(0.0f, angle - margin);
    return 3.0f * angle * angle + 12.0f * excess * (excess + std::abs(velocity));
  }
};

}

#endif
