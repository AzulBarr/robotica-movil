#include "TrajectoryFollower.h"

class FeedForwardController : public TrajectoryFollower
{
  public:

    FeedForwardController();

  protected:

<<<<<<< HEAD
    bool control(const rclcpp::Time& t, double& v, double& w) override;
=======
    bool control(const rclcpp::Time &t, double &vx, double &vy, double &w) override;
>>>>>>> b63f5b9a05067ef0ad2a2c8a33cc374104bdce38
};
