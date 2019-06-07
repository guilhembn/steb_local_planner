//
// Created by gbuisan on 23/05/19.
//

#include "teb_local_planner/humans_provider.h"
#include <uwds_msgs/GetScene.h>

teb_local_planner::HumansProvider::HumansProvider(ros::NodeHandle& nh, ros::NodeHandle& pnh){
  ctx_ = boost::make_shared<uwds::UnderworldsProxy>(boost::make_shared<ros::NodeHandle>(nh),
                                                   boost::make_shared<ros::NodeHandle>(pnh), "steb_local_planner", uwds::READER);
  if (ctx_->worlds()[HUMANS_WORLD].connect(bind(&HumansProvider::onChanges, this, _1, _2, _3))){
    ROS_INFO("Connected to uwds");
  }
}

void teb_local_planner::HumansProvider::onChanges(string world_name, Header header, Invalidations invalidations){
  humans_mutex_.lock();
  for (const auto& id: invalidations.node_ids_updated){
    if (ctx_->worlds()[HUMANS_WORLD].scene().nodes()[id].name.find("Human") != std::string::npos){//TODO: Check class instead
      auto hPose = ctx_->worlds()[HUMANS_WORLD].scene().getWorldPose(id);
      if (humans_.count(id) == 0){
        teb_local_planner::Human h(hPose.position.x, hPose.position.y, humanRadius);
        h.setCentroidVelocity(ctx_->worlds()[HUMANS_WORLD].scene().nodes()[id].velocity,
                              hPose.orientation);
        humans_[id] = h;
      }else{
        humans_[id].x() = hPose.position.x;
        humans_[id].y() = hPose.position.y;
        humans_[id].setCentroidVelocity(ctx_->worlds()[HUMANS_WORLD].scene().nodes()[id].velocity,
                              hPose.orientation);
      }
    }
  }

  for (const auto& id: invalidations.node_ids_deleted){
    if (ctx_->worlds()[HUMANS_WORLD].scene().nodes()[id].name.find("Human") != std::string::npos){ // TODO: Check class instead
      if (humans_.count(id) != 0){
        humans_.erase(id);
      }
    }
  }
  humans_mutex_.unlock();
}


bool teb_local_planner::HumansProvider::getLastHumans(
    teb_local_planner::HumanContainer &humans) {
    humans_mutex_.lock();
    humans.reserve(humans_.size());
    for (auto const& h: humans_){
      ROS_INFO("Human added !");
      humans.push_back(HumanPtr(new Human(h.second)));
    }
    humans_mutex_.unlock();
}
