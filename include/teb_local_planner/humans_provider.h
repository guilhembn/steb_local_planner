#ifndef TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
#define TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H

#include <ros/ros.h>
#include <teb_local_planner/humans.hpp>

#include <unordered_map>

#define UWDS_SERVICE_NAME "/uwds/get_scene"
#define HUMANS_WORLD "robot/env"

namespace teb_local_planner {

class HumansProvider{
public:
  HumansProvider() = default;
  HumansProvider(ros::NodeHandle& nh);

  bool getLastHumans(HumanContainer& humans);



  const float humanRadius = 0.3;

protected:
  ros::ServiceClient getuwdsSceneService_;
};

}  // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
