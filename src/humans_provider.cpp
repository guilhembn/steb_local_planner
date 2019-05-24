//
// Created by gbuisan on 23/05/19.
//

#include "teb_local_planner/humans_provider.h"

teb_local_planner::HumansProvider::HumansProvider(ros::NodeHandlePtr nh,
                                                  ros::NodeHandlePtr pnh): uwds::ReconfigurableClient(uwds::READER) {

}

void teb_local_planner::HumansProvider::onChanges(
    const string &world_name, const Header &header,
    const Invalidations &invalidations){
  for (const auto& input_world: input_worlds_) {
    for (const auto id : invalidations.node_ids_updated) {
      std::size_t isHuman = ctx_->worlds()[input_world].scene().nodes()[id].name.find("Human");
      if (isHuman != std::string::npos){
        auto uwdsH = ctx_->worlds()[input_world].scene().nodes()[id];
        // TODO: Check frame... (everything should be in the map frame)
        // TODO: Add human orientation
        if (humans_.count(id) == 0){
          Human h(uwdsH.pose.position.x, uwdsH.pose.position.y, humanRadius);
          h.setCentroidVelocity(uwdsH.velocity, uwdsH.pose.orientation);
          humans_.emplace(id, {uwdsH.last_update, h})
        }else{
          humans_[id].last_update = uwdsH.last_update;
          humans_[id].human.x() = uwdsH.pose.position.x;
          humans_[id].human.y() = uwdsH.pose.position.y;
          humans_[id].human.setCentroidVelocity(uwdsH.velocity, uwdsH.pose.orientation);
        }
      }
    }
  }
}


void teb_local_planner::HumansProvider::onReconfigure(
    const vector<string> &input_worlds) {}
