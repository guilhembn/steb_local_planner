/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/social_optimal_planner.h>
#include <map>
#include <memory>
#include <limits>


namespace teb_local_planner
{

// ============== Implementation ===================

SocialTebOptimalPlanner::SocialTebOptimalPlanner() : TebOptimalPlanner(), humans_(NULL)
{    
}
SocialTebOptimalPlanner::SocialTebOptimalPlanner(
    const TebConfig &cfg, ObstContainer *obstacles,
    RobotFootprintModelPtr robot_model, TebVisualizationPtr visual,
    const ViaPointContainer *via_points, HumanContainer *humans): TebOptimalPlanner(cfg, obstacles, robot_model, visual, via_points), humans_(humans) {
    initialize(cfg, obstacles, robot_model, visual, via_points, humans);
}

void SocialTebOptimalPlanner::initialize(
    const teb_local_planner::TebConfig &cfg,
    teb_local_planner::ObstContainer *obstacles,
    teb_local_planner::RobotFootprintModelPtr robot_model,
    teb_local_planner::TebVisualizationPtr visual,
    const teb_local_planner::ViaPointContainer *via_points,
    teb_local_planner::HumanContainer *humans) {

    // init optimizer (set solver and block ordering settings)
    optimizer_ = initOptimizerWithHumans();

    cfg_ = &cfg;
    obstacles_ = obstacles;
    humans_ = humans;
    robot_model_ = robot_model;
    via_points_ = via_points;
    cost_ = HUGE_VAL;
    prefer_rotdir_ = RotType::none;
    setVisualization(visual);

    vel_start_.first = true;
    vel_start_.second.linear.x = 0;
    vel_start_.second.linear.y = 0;
    vel_start_.second.angular.z = 0;

    vel_goal_.first = true;
    vel_goal_.second.linear.x = 0;
    vel_goal_.second.linear.y = 0;
    vel_goal_.second.angular.z = 0;
    initialized_ = true;
  }

void SocialTebOptimalPlanner::registerG2OTypesWithHumans()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
}

boost::shared_ptr<g2o::SparseOptimizer> SocialTebOptimalPlanner::initOptimizerWithHumans(){
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypesWithHumans, flag);

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading(); // required for >Eigen 3.1

  return optimizer;
}

bool SocialTebOptimalPlanner::buildGraph(double weight_multiplier){
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();

  AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.


  AddEdgesPreferRotDir();

  AddEdgesProxemics();

  return true;
}

void SocialTebOptimalPlanner::AddEdgesProxemics(){
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_social_proxemics);

  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
    Human* human = nullptr;

    std::vector<Human*> relevant_humans;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const HumanPtr& human : *humans_)
    {

      // calculate distance to current pose
      double dist = human->getMinimumDistance(teb_.Pose(i).position());

      // force considering obstacle if really close to the current pose
      if (dist < cfg_->socialTeb.min_robot_human_distance * cfg_->socialTeb.robot_human_distance_cutoff_factor)
      {
        relevant_humans.push_back(human.get());
        continue;
      }
    }

    for (const Human* human : relevant_humans)
    {
      EdgeProxemics* dist_bandpt_prox = new EdgeProxemics;
      dist_bandpt_prox->setVertex(0,teb_.PoseVertex(i));
      dist_bandpt_prox->setInformation(information);
      dist_bandpt_prox->setParameters(*cfg_, robot_model_.get(), human);
      optimizer_->addEdge(dist_bandpt_prox);
    }
  }
}

} // namespace teb_local_planner
