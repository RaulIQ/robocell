#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "view.hpp"

namespace mppi::critics
{

/** Scores the camera angle to the latest observed target along MPPI trajectories. */
class TargetCritic final : public CriticFunction
{
  using Pose = geometry_msgs::msg::PoseStamped;

  void initialize() override
  {
    subscription_ = parent_.lock()->create_subscription<Pose>(
      "/target/pose", rclcpp::QoS(1),
      [this](Pose::ConstSharedPtr pose) {std::atomic_store(&target_, std::move(pose));});
  }

  void score(CriticData & data) override
  {
    if (!enabled_) {
      return;
    }
    const auto target = std::atomic_load(&target_);
    if (!target || target->header.frame_id != data.state.pose.header.frame_id) {
      return;
    }
    const auto & position = target->pose.position;
    const auto now = parent_.lock()->now();
    const double age = (now - rclcpp::Time(target->header.stamp, now.get_clock_type())).seconds();
    if (age < 0.0 || age > 0.5 || !std::isfinite(position.x) || !std::isfinite(position.y)) {
      return;
    }
    const auto & trajectories = data.trajectories;
    const robocell_follow::View view{0.35f, 0.076f};
    const size_t stride = 4;
    const size_t steps = trajectories.x.shape(1);
    const size_t samples = (steps + stride - 1) / stride;
    for (size_t row = 0; row < trajectories.x.shape(0); ++row) {
      float cost = 0.0f;
      for (size_t step = 0; step < steps; step += stride) {
        cost += view.cost(
          static_cast<float>(position.x) - trajectories.x(row, step),
          static_cast<float>(position.y) - trajectories.y(row, step),
          trajectories.yaws(row, step), data.state.vx(row, step));
      }
      data.costs(row) += cost / static_cast<float>(samples);
    }
  }

  Pose::ConstSharedPtr target_;
  rclcpp::Subscription<Pose>::SharedPtr subscription_;
};

}

PLUGINLIB_EXPORT_CLASS(mppi::critics::TargetCritic, mppi::critics::CriticFunction)
