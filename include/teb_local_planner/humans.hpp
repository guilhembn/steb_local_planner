//
// Created by gbuisan on 17/05/19.
//

#ifndef TEB_LOCAL_PLANNER_HUMANS_HPP
#define TEB_LOCAL_PLANNER_HUMANS_HPP

#include <teb_local_planner/obstacles.h>

namespace teb_local_planner {

//TODO add human orientation

class Human : public CircularObstacle {
public:
  /**
  * @brief Default constructor of the circular obstacle class
   */
  Human() : CircularObstacle(){}

  /**
    * @brief Construct CircularObstacle using a 2d center position vector and radius
    * @param position 2d position that defines the current obstacle position
    * @param radius radius of the obstacle
   */
  Human(const Eigen::Ref<const Eigen::Vector2d> &position, double orientation, double radius)
      : CircularObstacle(position, radius), orientation_(orientation){}

  /**
    * @brief Construct CircularObstacle using x- and y-center-coordinates and radius
    * @param x x-coordinate
    * @param y y-coordinate
    * @param radius radius of the obstacle
   */
  Human(double x, double y, double theta, double radius)
      : CircularObstacle(x, y, radius), orientation_(theta) {}

  Human(const Human& human): CircularObstacle(human.x(), human.y(), human.radius()), orientation_(human.orientation()){}

  double& orientation() {return orientation_;} //!< Return the current orientation of the human
  const double& orientation() const {return orientation_;} //!< Return the current orientation of the human

protected:
  double orientation_;




};

//! Abbrev. for shared human pointers
typedef boost::shared_ptr<Human> HumanPtr;
//! Abbrev. for shared human const pointers
typedef boost::shared_ptr<const Human> HumanConstPtr;
//! Abbrev. for containers storing multiple humans
typedef std::vector<HumanPtr> HumanContainer;

} // namespace teb_local_planner
#endif // TEB_LOCAL_PLANNER_HUMANS_HPP
