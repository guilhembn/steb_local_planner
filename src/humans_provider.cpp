//
// Created by gbuisan on 23/05/19.
//

#include "teb_local_planner/humans_provider.h"

teb_local_planner::HumansProvider::HumansProvider(): uwds::ReconfigurableClient(uwds::READER) {
}

void teb_local_planner::HumansProvider::onChanges(
    const string &world_name, const Header &header,
    const Invalidations &invalidations){
  humans_mutex_.lock();
  for (const auto& input_world: input_worlds_) {
    for (const auto& id : invalidations.node_ids_updated) {
      std::size_t isHuman = ctx_->worlds()[input_world].scene().nodes()[id].name.find("Human");
      if (isHuman != std::string::npos){
        auto uwdsH = ctx_->worlds()[input_world].scene().nodes()[id];
        // TODO: Check frame... (everything should be in the map frame)
        // TODO: Add human orientation
        if (humans_.count(id) == 0){
          teb_local_planner::Human h(uwdsH.position.pose.position.x, uwdsH.position.pose.position.y, humanRadius);
          h.setCentroidVelocity(uwdsH.velocity, uwdsH.position.pose.orientation);
          humans_[id] = h;
        }else{
          humans_[id].x() = uwdsH.position.pose.position.x;
          humans_[id].y() = uwdsH.position.pose.position.y;
          humans_[id].setCentroidVelocity(uwdsH.velocity, uwdsH.position.pose.orientation);
        }
      }
    }

    for (const auto& id : invalidations.node_ids_deleted){
      std::size_t isHuman = ctx_->worlds()[input_world].scene().nodes()[id].name.find("Human");
      if (isHuman != std::string::npos){
        if (humans_.count(id) != 0){
          humans_.erase(id);
        }
      }
    }
  }
  humans_mutex_.unlock();
}


void teb_local_planner::HumansProvider::onReconfigure(
    const vector<string> &input_worlds) {}


bool teb_local_planner::HumansProvider::getLastHumans(
    teb_local_planner::HumanContainer &humans) {
  humans_mutex_.lock();
  humans.reserve(humans_.size());
  for (auto const& h: humans_){
    humans.push_back(HumanPtr(new Human(h.second)));
  }
  humans_mutex_.unlock();
}
