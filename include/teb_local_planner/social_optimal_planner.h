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

#ifndef SOCIAL_OPTIMAL_PLANNER_H_
#define SOCIAL_OPTIMAL_PLANNER_H_

#include <math.h>


// teb stuff

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/humans.hpp>

// Custom g2o types
#include <teb_local_planner/g2o_types/edge_proxemics.h>

namespace teb_local_planner
{


/**
 * @class SocialTebOptimalPlanner
 * @brief This class optimizes an internal Timed Elastic Band trajectory using the g2o-framework.
 * 
 * For an introduction and further details about the TEB optimization problem refer to:
 * 	- C. Rösmann et al.: Trajectory modification considering dynamic constraints of autonomous robots, ROBOTIK, 2012.
 * 	- C. Rösmann et al.: Efficient trajectory optimization using a sparse model, ECMR, 2013.
 * 	- R. Kümmerle et al.: G2o: A general framework for graph optimization, ICRA, 2011. 
 * 
 * @todo: Call buildGraph() only if the teb structure has been modified to speed up hot-starting from previous solutions.
 * @todo: We introduced the non-fast mode with the support of dynamic obstacles
 *        (which leads to better results in terms of x-y-t homotopy planning).
 *        However, we have not tested this mode intensively yet, so we keep
 *        the legacy fast mode as default until we finish our tests.
 */
class SocialTebOptimalPlanner : public TebOptimalPlanner
{
public:
    
  /**
   * @brief Default constructor
   */
  SocialTebOptimalPlanner();
  
  /**
   * @brief Construct and initialize the TEB optimal planner.
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visual Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  SocialTebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                    TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL, HumanContainer* humans = NULL);
  
  /**
   * @brief Destruct the optimal planner.
   */
  virtual ~SocialTebOptimalPlanner() {};

  /**
    * @brief Initializes the optimal planner
    * @param cfg Const reference to the TebConfig class for internal parameters
    * @param obstacles Container storing all relevant obstacles (see Obstacle)
    * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
    * @param visual Shared pointer to the TebVisualization class (optional)
    * @param via_points Container storing via-points (optional)
    */
  void initialize(const TebConfig& cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                  TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL) override;

  boost::shared_ptr<g2o::SparseOptimizer> initOptimizerWithHumans();

  void visualize() override;

protected:

  static void registerG2OTypesWithHumans();
  bool buildGraph(double weight_multiplier=1.0) override;
  void AddEdgesProxemics();

  // external objects (store weak pointers)
  HumanContainer* humans_; //!< Store humans that are relevant for planning

  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the TebOptimalPlanner
typedef boost::shared_ptr<TebOptimalPlanner> TebOptimalPlannerPtr;
//! Abbrev. for shared const TebOptimalPlanner pointers
typedef boost::shared_ptr<const TebOptimalPlanner> TebOptimalPlannerConstPtr;
//! Abbrev. for containers storing multiple teb optimal planners
typedef std::vector< TebOptimalPlannerPtr > TebOptPlannerContainer;

} // namespace teb_local_planner

#endif /* OPTIMAL_PLANNER_H_ */
