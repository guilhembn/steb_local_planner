#ifndef TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
#define TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H

#include <ros/ros.h>
#include <uwds/reconfigurable_client.h>
#include <teb_local_planner/humans.hpp>

#include <unordered_map>

namespace teb_local_planner {

class HumansProvider : public uwds::ReconfigurableClient {
public:
  HumansProvider(ros::NodeHandlePtr nh, ros::NodeHandlePtr pnh);

  bool getLastHumans(HumanContainer& humans);
  void onChanges(const string &world_name, const Header &header,
                 const Invalidations &invalidations) override;
  void onReconfigure(const vector<string> &input_worlds) override;

protected:
  struct sHumansLastSeen{
    std_msgs::Time last_update;  // TODO: last_update or last_observation ? Are the ros::Time::now() and uwds given time synchronized ?
    Human human;
  };

  static const constexpr float humanRadius = 0.3;

  std::unordered_map<std::string, sHumansLastSeen> humans_;
};

}  // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
