#ifndef TESSERACT_COLLISION_BENCHMARK_UTILS_H
#define TESSERACT_COLLISION_BENCHMARK_UTILS_H

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometric_shapes/shape_operations.h>
#include <console_bridge/console.h>
#include <random_numbers/random_numbers.h>
#include <tesseract_common/types.h>
#include <tesseract_geometry/geometry.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_environment/core/state_solver.h>

namespace tesseract_collision
{

/** \brief Factor to compute the maximum number of trials random clutter generation. */
static const int MAX_SEARCH_FACTOR_CLUTTER = 3;

/** \brief Factor to compute the maximum number of trials for random state generation. */
static const int MAX_SEARCH_FACTOR_STATES = 30;

/** \brief Enumerates the different types of collision objects. */
enum class CollisionObjectType
{
  MESH,
  CONVEX_MESH,
  BOX,
};

/** \brief Defines a random robot state. */
enum class RobotStateSelector
{
  IN_COLLISION,
  NOT_IN_COLLISION,
  RANDOM,
};

/** \brief Enumerates the available collision detectors. */
enum class CollisionDetector
{
  FCL,
  BULLET,
};

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *  \param planning_scene The planning scene
 *  \param num_objects The number of objects to be cluttered
 *  \param CollisionObjectType Type of object to clutter (mesh or box)
 */
void clutterWorld(std::vector<tesseract_geometry::Geometry::ConstPtr>& shapes,
                  tesseract_common::VectorIsometry3d& shape_poses,
                  const planning_scene::PlanningScenePtr& planning_scene,
                  const tesseract_collision::DiscreteContactManager::Ptr& contact_checker,
                  const tesseract_environment::StateSolver::Ptr& state_solver,
                  const std::size_t num_objects,
                  CollisionObjectType type);

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param scene The planning scene
 *  \param robot_states Result vector
 *  \return number of state in collision
 */
int findStates(std::vector<moveit::core::RobotState>& robot_states,
               RobotStateSelector desired_states,
               unsigned int num_states,
               const planning_scene::PlanningScenePtr& scene,
               const DiscreteContactManager::Ptr &contact_checker,
               const tesseract_environment::StateSolver::Ptr &state_solver);

}

#endif // TESSERACT_COLLISION_BENCHMARK_UTILS_H
