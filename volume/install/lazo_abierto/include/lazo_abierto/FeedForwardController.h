#include "TrajectoryFollower.h"

class FeedForwardController : public TrajectoryFollower
{
  public:

    FeedForwardController();

  protected:

    bool control(const rclcpp::Time &t, double &vx, double &vy, double &w) override;
};
