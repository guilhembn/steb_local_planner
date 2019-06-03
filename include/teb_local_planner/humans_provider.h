#ifndef TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
#define TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H

#include <ros/ros.h>
#include <uwds/reconfigurable_client.h>
#include <teb_local_planner/humans.hpp>

#include <unordered_map>

namespace teb_local_planner {

class HumansProvider : public uwds::ReconfigurableClient {
public:
  HumansProvider();

  bool getLastHumans(HumanContainer& humans);
  void onChanges(const string &world_name, const Header &header,
                 const Invalidations &invalidations) override;
  void onReconfigure(const vector<string> &input_worlds) override;

protected:

  static const constexpr float humanRadius = 0.3;

  std::unordered_map<std::string, Human> humans_;

  boost::mutex humans_mutex_;
};

}  // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_HUMANS_PROVIDER_H
