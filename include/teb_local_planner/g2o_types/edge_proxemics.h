//
// Created by gbuisan on 04/06/19.
//

#ifndef TEB_LOCAL_PLANNER_EDGE_PROXEMICS_H
#define TEB_LOCAL_PLANNER_EDGE_PROXEMICS_H

#include <teb_local_planner/g2o_types/base_teb_edges.h>

namespace teb_local_planner{

class EdgeProxemics : public BaseTebUnaryEdge<1, const Human*, VertexPose>
{
public:
  EdgeProxemics(){
    _measurement = NULL;
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setHuman() and setRobotModel() on EdgeProxemics()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = robot_model_->calculateDistance(bandpt->pose(), _measurement);

    _error[0] = penaltyBoundFromBelow(dist, cfg_->socialTeb.min_robot_human_distance, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeObstacle::computeError() _error[0]=%f\n",_error[0]);
  }

  void setHuman(const Human* human)
  {
    _measurement = human;
  }

  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Human* human)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = human;
  }

protected:
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model


};



} // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_EDGE_PROXEMICS_H
