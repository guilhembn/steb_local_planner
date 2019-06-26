#ifndef TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
#define TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H

#include <ros/ros.h>
#include <teb_local_planner/humans.hpp>
#include <uwds/uwds.h>

#include <unordered_map>

#define UWDS_SERVICE_NAME "/uwds/get_scene"
#define HUMANS_WORLD "robot/merged"

namespace teb_local_planner {

class HumansProvider{
public:
  HumansProvider() = default;
  HumansProvider(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  bool getLastHumans(HumanContainer& humans);



  const float humanRadius = 0.3;

protected:
  void onChanges(string world_name, Header header, Invalidations invalidations);

  std::unordered_map<std::string, Human> humans_;
  boost::mutex humans_mutex_;
  uwds::UnderworldsProxyPtr ctx_;

};

}  // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
