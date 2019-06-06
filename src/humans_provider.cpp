//
// Created by gbuisan on 23/05/19.
//

#include "teb_local_planner/humans_provider.h"
#include <uwds_msgs/GetScene.h>

teb_local_planner::HumansProvider::HumansProvider(ros::NodeHandle& nh){
  getuwdsSceneService_ = nh.serviceClient<uwds_msgs::GetScene>(UWDS_SERVICE_NAME);
}


bool teb_local_planner::HumansProvider::getLastHumans(
    teb_local_planner::HumanContainer &humans) {
  uwds_msgs::GetScene scene;
  scene.request.ctxt.client.type = scene.request.ctxt.client.READER;
  scene.request.ctxt.client.name = "steb_local_planner";
  scene.request.ctxt.world = HUMANS_WORLD;
  if (!getuwdsSceneService_.call(scene)){
    return false;
  }
  humans.clear();
  for (auto &node: scene.response.nodes){
    if (node.name.find("Human") != std::string::npos){
      HumanPtr h = boost::make_shared<Human>(node.position.pose.position.x, node.position.pose.position.y,
                                  humanRadius);
      h->setCentroidVelocity(node.velocity, node.position.pose.orientation);
      humans.push_back(h);
    }
  }
  return true;

}
