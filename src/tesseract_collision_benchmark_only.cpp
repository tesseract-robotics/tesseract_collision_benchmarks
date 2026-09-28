/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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
 *********************************************************************/

/* Author: Jens Petit */

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/stopwatch.h>
#include <tesseract/common/types.h>
#include <tesseract/common/logging.h>
#include <tesseract/common/utils.h>
#include <tesseract_collision_benchmark/types.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/coal/coal_discrete_managers.h>
#include <tesseract/collision/coal/coal_cast_managers.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
// #include <tesseract_collision_physx/physx_discrete_manager.h>

#include <tesseract/geometry/geometry.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/impl/convex_mesh.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/commands/modify_allowed_collisions_command.h>

#include <random_numbers/random_numbers.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace tesseract::collision
{

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *  \param num_objects The number of objects to be cluttered
 *  \param CollisionObjectType Type of object to clutter (mesh or box)
 */
void clutterWorld(std::vector<tesseract::geometry::Geometry::ConstPtr>& shapes,
                  tesseract::common::VectorIsometry3d& shape_poses,
                  const DiscreteContactManager::Ptr& contact_checker,
                  const tesseract::scene_graph::StateSolver::Ptr& state_solver,
                  const std::size_t num_objects,
                  CollisionObjectType type)
{
  TESSERACT_LOG_INFO("Cluttering scene...");

  auto num_generator = random_numbers::RandomNumberGenerator(123);

  auto t_env_state = state_solver->getState();
  contact_checker->setCollisionObjectsTransform(t_env_state.link_transforms);

  // load panda link5 as world collision object
  std::string name;
  tesseract::geometry::Geometry::Ptr t_shape;
  std::string kinect = "package://moveit_resources_panda_description/meshes/collision/link5.stl";

  Eigen::Quaterniond quat;
  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };

  size_t added_objects{ 0 };
  size_t i{ 0 };
  // create random objects until as many added as desired or quit if too many attempts
  while (added_objects < num_objects && i < num_objects * MAX_SEARCH_FACTOR_CLUTTER)
  {
    // add with random size and random position
    pos.translation().x() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().y() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().z() = num_generator.uniformReal(0.0, 1.0);

    quat.x() = num_generator.uniformReal(-1.0, 1.0);
    quat.y() = num_generator.uniformReal(-1.0, 1.0);
    quat.z() = num_generator.uniformReal(-1.0, 1.0);
    quat.w() = num_generator.uniformReal(-1.0, 1.0);
    quat.normalize();
    pos.rotate(quat);

    switch (type)
    {
      case CollisionObjectType::MESH:
      case CollisionObjectType::CONVEX_MESH:
      {
        name = "mesh";
        tesseract::common::GeneralResourceLocator locator;
        auto resource = locator.locateResource(kinect);
        Eigen::Vector3d scale = Eigen::Vector3d::Constant(num_generator.uniformReal(0.3, 1.0));
        auto t_mesh =
            tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(resource, scale, true, false);
        assert(t_mesh.size() == 1);
        if (type == CollisionObjectType::MESH)
          t_shape = t_mesh[0];
        else
          t_shape = tesseract::collision::makeConvexMesh(*t_mesh[0]);

        break;
      }
      case CollisionObjectType::BOX:
      {
        name = "box";
        const double x = num_generator.uniformReal(0.05, 0.2);
        const double y = num_generator.uniformReal(0.05, 0.2);
        const double z = num_generator.uniformReal(0.05, 0.2);
        t_shape = std::make_shared<tesseract::geometry::Box>(x, y, z);
        break;
      }
    }

    name.append(std::to_string(i));
    contact_checker->addCollisionObject(name, 0, { t_shape }, { pos });

    tesseract::collision::ContactRequest t_req(tesseract::collision::ContactTestType::FIRST);
    tesseract::collision::ContactResultMap t_res;
    contact_checker->contactTest(t_res, t_req);

    if (t_res.empty())
    {
      added_objects++;
      shapes.push_back(t_shape);
      shape_poses.push_back(pos);
    }
    else
    {
      TESSERACT_LOG_INFO("Object was in collision, remove");
    }
    contact_checker->removeCollisionObject(name);
    i++;
  }
  TESSERACT_LOG_INFO("Cluttered the planning scene with {} objects", added_objects);
}

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param robot_states Result vector
 *  \param robot_joint_values Joint values behind each entry of robot_states, same order and size.
 *         Interpolating a state needs the joint values; the link transforms alone cannot be
 *         interpolated into a reachable configuration.
 *  \return number of state in collision
 */
int findStates(std::vector<tesseract::common::LinkIdTransformMap>& robot_states,
               std::vector<tesseract::scene_graph::SceneState::JointValues>& robot_joint_values,
               RobotStateSelector desired_states,
               unsigned int num_states,
               const DiscreteContactManager::Ptr& contact_checker,
               const tesseract::scene_graph::StateSolver::Ptr& state_solver)
{
  std::size_t i{ 0 };
  int states_in_collision{ 0 };
  while (robot_states.size() < num_states)
  {
    auto t_env_state = state_solver->getRandomState();
    auto t_transforms = t_env_state.link_transforms;
    contact_checker->setCollisionObjectsTransform(t_transforms);
    tesseract::collision::ContactRequest t_req(tesseract::collision::ContactTestType::FIRST);
    tesseract::collision::ContactResultMap t_res;
    contact_checker->contactTest(t_res, t_req);

    switch (desired_states)
    {
      case RobotStateSelector::IN_COLLISION:
        if (!t_res.empty())
        {
          robot_states.push_back(t_transforms);
          robot_joint_values.push_back(t_env_state.joints);
          ++states_in_collision;
        }
        break;
      case RobotStateSelector::NOT_IN_COLLISION:
        if (t_res.empty())
        {
          robot_states.push_back(t_transforms);
          robot_joint_values.push_back(t_env_state.joints);
        }
        break;
      case RobotStateSelector::RANDOM:
        robot_states.push_back(t_transforms);
        robot_joint_values.push_back(t_env_state.joints);
        if (!t_res.empty())
          ++states_in_collision;
        break;
    }
    i++;
  }

  return states_in_collision;
}

/** \brief Expands sampled states into a cyclic joint-space path visiting each of them in turn.
 *
 *  Segment i runs from sampled state i to state i+1, the last wrapping back to the first, and
 *  contributes @p waypoints evenly spaced configurations starting at state i. The result therefore
 *  contains every sampled state and has size states * waypoints, and walking it cyclically returns
 *  to its start. Interpolation is in joint space, so every waypoint is a configuration the robot
 *  can actually hold.
 *
 *  \param sampled_joint_values Joint values of the sampled states, in order
 *  \param waypoints Configurations per segment; 1 reproduces the sampled states themselves
 *  \param state_solver Solver used to run forward kinematics on each interpolated configuration */
std::vector<tesseract::common::LinkIdTransformMap>
buildTrajectoryStates(const std::vector<tesseract::scene_graph::SceneState::JointValues>& sampled_joint_values,
                      unsigned int waypoints,
                      const tesseract::scene_graph::StateSolver& state_solver)
{
  std::vector<tesseract::common::LinkIdTransformMap> trajectory;
  const std::size_t num_sampled = sampled_joint_values.size();
  trajectory.reserve(num_sampled * waypoints);

  tesseract::scene_graph::SceneState::JointValues interpolated;
  for (std::size_t i = 0; i < num_sampled; ++i)
  {
    const auto& from = sampled_joint_values[i];
    const auto& to = sampled_joint_values[(i + 1) % num_sampled];
    for (unsigned int k = 0; k < waypoints; ++k)
    {
      const double t = static_cast<double>(k) / static_cast<double>(waypoints);
      interpolated.clear();
      for (const auto& [joint_id, value] : from)
        interpolated[joint_id] = value + (t * (to.at(joint_id) - value));

      trajectory.push_back(state_solver.getState(interpolated).link_transforms);
    }
  }

  return trajectory;
}
}  // namespace tesseract::collision

/** \brief Whether a contact manager name matches a filter list.
 *
 *   Matching is a case-insensitive substring test, so "coal" selects every Coal manager. Filters are
 *   expected to be lowercase already. An empty filter list selects every manager.
 *
 *   \param filters Lowercase name substrings to match against
 *   \param name The manager name as reported by getName() */
bool managerSelected(const std::vector<std::string>& filters, const std::string& name)
{
  if (filters.empty())
    return true;

  std::string lowered = name;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  return std::any_of(filters.begin(), filters.end(), [&lowered](const std::string& filter) {
    return lowered.find(filter) != std::string::npos;
  });
}

/**
 * @brief How often collision object transforms are updated relative to contact checks.
 *
 * REPEAT sets a sampled state once and re-tests it for every trial, so all but the first
 * check of each state skips the broadphase update. SWEEP advances to the next sampled state
 * before every check. The sampled states are independent configurations rather than
 * successive trajectory waypoints, so SWEEP is an upper bound on update cost, not a
 * simulation of one.
 *
 * TRAJECTORY also advances before every check, but along joint-space interpolations between
 * consecutive sampled states, so the motion between two checks is a controlled fraction of the
 * SWEEP jump. It exists because a backend that warm-starts its narrowphase from the previous
 * check reads as fastest under REPEAT and slowest under SWEEP, and neither bound is what a
 * planner sees; the interpolation count selects a point between them. TRAJECTORY with one
 * waypoint per segment is SWEEP.
 */
enum class DutyCycle : std::uint8_t
{
  REPEAT,
  SWEEP,
  TRAJECTORY
};

/** @brief A duty cycle together with the interpolation count TRAJECTORY needs. */
struct DutyCycleSpec
{
  DutyCycle cycle{ DutyCycle::SWEEP };
  /** @brief Interpolation steps between consecutive sampled states. Ignored unless cycle is TRAJECTORY. */
  unsigned int waypoints{ 1 };
};

/** \brief Lowercase name for a duty cycle, as written to the CSV and shown in --help/log text.
 *
 *  TRAJECTORY carries its waypoint count, because two trajectory runs at different step sizes are
 *  not comparable and the CSV records one duty cycle per file. */
std::string dutyCycleName(const DutyCycleSpec& spec)
{
  switch (spec.cycle)
  {
    case DutyCycle::SWEEP:
      return "sweep";
    case DutyCycle::TRAJECTORY:
      return "trajectory-" + std::to_string(spec.waypoints);
    case DutyCycle::REPEAT:
    default:
      return "repeat";
  }
}

/** \brief Runs a collision detection benchmark and measures the time.
 *
 *   \param name Name to give the benchmark
 *   \param trials The number of repeated collision checks for each state
 *   \param checker Tesseract contact checker
 *   \param state A vector of collision object transforms
 *   \param test_type The tesseract contact test type (FIRST, ALL, CLOSEST)
 *   \param distance Turn on distance
 *   \param penetration Turn on penetration */
void runTesseractCollisionDetection(std::ostream& csv_stream,
                                    const std::string& name,
                                    const std::string& scenario,
                                    unsigned int trials,
                                    tesseract::collision::DiscreteContactManager& checker,
                                    const std::vector<tesseract::common::LinkIdTransformMap>& states,
                                    tesseract::collision::ContactTestType test_type,
                                    bool distance,
                                    bool penetration,
                                    bool is_physx,
                                    const DutyCycleSpec& duty_cycle)
{
  //  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
  //      scene->getRobotModel()->getLinkModelNames(), true) };

  std::string ct = tesseract::collision::ContactTestTypeStrings.at(static_cast<std::size_t>(test_type));
  std::string desc = name + "(" + ct + ")";

  tesseract::collision::ContactResultMap res;
  tesseract::collision::ContactRequest req(test_type);
  req.calculate_distance = distance;
  req.calculate_penetration = penetration;

  tesseract::common::Stopwatch stopwatch;
  stopwatch.start();

  // Physx links go to sleep if they have not moved in 3-4 contact test requests, so physx
  // must see a transform update before every check.
  const DutyCycleSpec effective_duty_cycle =
      (is_physx && duty_cycle.cycle == DutyCycle::REPEAT) ? DutyCycleSpec{ DutyCycle::SWEEP, 1 } : duty_cycle;

  if (effective_duty_cycle.cycle != DutyCycle::REPEAT)
  {
    for (unsigned int i = 0; i < trials; ++i)
    {
      for (const auto& state : states)
      {
        res.clear();
        checker.setCollisionObjectsTransform(state);
        checker.contactTest(res, req);
      }
    }
  }
  else
  {
    for (const auto& state : states)
    {
      checker.setCollisionObjectsTransform(state);
      for (unsigned int i = 0; i < trials; ++i)
      {
        res.clear();
        checker.contactTest(res, req);
      }
    }
  }

  stopwatch.stop();
  const double duration = stopwatch.elapsedSeconds();

  const double checks_per_second = static_cast<double>(trials * states.size()) / duration;
  const std::size_t total_num_checks = trials * states.size();

  std::size_t contact_count = 0;
  for (const auto& c : res)
    contact_count += c.second.size();

  csv_stream << std::quoted(scenario) << "," << std::quoted(name) << "," << std::quoted(ct) << "," << checks_per_second
             << "," << total_num_checks << "," << contact_count << ","
             << std::quoted(dutyCycleName(effective_duty_cycle)) << "\n";

  TESSERACT_LOG_INFO("{:<40} | {:>17.0f} | {:>16} | {:>12}", desc, checks_per_second, total_num_checks, contact_count);
}

/** \brief Runs a continuous collision detection benchmark and measures the time.
 *
 *   \param name Name to give the benchmark
 *   \param trials The number of repeated collision checks for each state pair
 *   \param checker Tesseract continuous contact checker
 *   \param state_pairs A vector of start/end collision object transform pairs
 *   \param test_type The tesseract contact test type (FIRST, ALL, CLOSEST)
 *   \param distance Turn on distance
 *   \param penetration Turn on penetration */
void runTesseractContinuousCollisionDetection(
    std::ostream& csv_stream,
    const std::string& name,
    const std::string& scenario,
    unsigned int trials,
    tesseract::collision::ContinuousContactManager& checker,
    const std::vector<std::pair<tesseract::common::LinkIdTransformMap, tesseract::common::LinkIdTransformMap>>&
        state_pairs,
    const std::vector<tesseract::common::LinkId>& active_links,
    tesseract::collision::ContactTestType test_type,
    bool distance,
    bool penetration,
    bool clone_per_state,
    const DutyCycleSpec& duty_cycle)
{
  std::string ct = tesseract::collision::ContactTestTypeStrings.at(static_cast<std::size_t>(test_type));
  std::string desc = name + "(" + ct + ")";

  tesseract::collision::ContactResultMap res;
  tesseract::collision::ContactRequest req(test_type);
  req.calculate_distance = distance;
  req.calculate_penetration = penetration;

  // Pre-compute active link ID set for fast lookup
  std::unordered_set<tesseract::common::LinkId> active_link_set(active_links.begin(), active_links.end());

  // Active links carry a cast (moving) transform, everything else a static one. The split is done
  // once here rather than per check: a caller such as TrajOpt already holds its transforms in this
  // shape, so splitting inside the timed loop would charge every backend for work it does not do.
  // Parallel arrays rather than maps: the manager looks each id up regardless, so the map forms only
  // add a hash lookup per link to find the second pose.
  struct SplitState
  {
    std::vector<tesseract::common::LinkId> cast_ids;
    tesseract::common::VectorIsometry3d cast_pose1;
    tesseract::common::VectorIsometry3d cast_pose2;
    std::vector<tesseract::common::LinkId> static_ids;
    tesseract::common::VectorIsometry3d static_poses;
  };
  std::vector<SplitState> split_states;
  split_states.reserve(state_pairs.size());
  for (const auto& [pose1, pose2] : state_pairs)
  {
    SplitState split;
    for (const auto& tf : pose1)
    {
      if (active_link_set.count(tf.first) != 0)
      {
        split.cast_ids.push_back(tf.first);
        split.cast_pose1.push_back(tf.second);
        split.cast_pose2.push_back(pose2.at(tf.first));
      }
      else
      {
        split.static_ids.push_back(tf.first);
        split.static_poses.push_back(tf.second);
      }
    }
    split_states.push_back(std::move(split));
  }

  auto apply_state = [](tesseract::collision::ContinuousContactManager& mgr, const SplitState& split) {
    // Two bulk calls rather than one per link: a manager that defers its broadphase update to the end
    // of a call then refits once per call, not once per link.
    if (!split.static_ids.empty())
      mgr.setCollisionObjectsTransform(split.static_ids, split.static_poses);
    if (!split.cast_ids.empty())
      mgr.setCollisionObjectsTransform(split.cast_ids, split.cast_pose1, split.cast_pose2);
  };

  tesseract::common::Stopwatch stopwatch;
  stopwatch.start();

  if (duty_cycle.cycle != DutyCycle::REPEAT)
  {
    // This branch never honours clone_per_state; it always reuses the single incoming checker.
    // That is safe only because main() rejects --clone together with an advancing duty cycle, so
    // clone_per_state is never true here.
    for (unsigned int i = 0; i < trials; ++i)
    {
      for (const auto& split : split_states)
      {
        res.clear();
        apply_state(checker, split);
        checker.contactTest(res, req);
      }
    }
  }
  else
  {
    for (const auto& split : split_states)
    {
      auto cloned = clone_per_state ? checker.clone() : nullptr;
      auto& active_checker = clone_per_state ? *cloned : checker;

      apply_state(active_checker, split);
      for (unsigned int i = 0; i < trials; ++i)
      {
        res.clear();
        active_checker.contactTest(res, req);
      }
    }
  }
  stopwatch.stop();
  const double duration = stopwatch.elapsedSeconds();

  const double checks_per_second = static_cast<double>(trials * state_pairs.size()) / duration;
  const std::size_t total_num_checks = trials * state_pairs.size();

  std::size_t contact_count = 0;
  for (const auto& c : res)
    contact_count += c.second.size();

  csv_stream << std::quoted(scenario) << "," << std::quoted(name) << "," << std::quoted(ct) << "," << checks_per_second
             << "," << total_num_checks << "," << contact_count << "," << std::quoted(dutyCycleName(duty_cycle))
             << "\n";

  TESSERACT_LOG_INFO("{:<40} | {:>17.0f} | {:>16} | {:>12}", desc, checks_per_second, total_num_checks, contact_count);
}

int main(int argc, char** argv)
{
  tesseract::common::getLogger()->set_level(spdlog::level::info);
  const unsigned int trials = 1000;
  const unsigned int num_states = 50;

  std::string csv_path = "tesseract_collision_benchmark.csv";
  std::string mode = "both";
  std::string test_type = "all-types";
  int seed = -1;
  bool clone_per_state = false;
  DutyCycleSpec duty_cycle;
  bool duty_cycle_explicit = false;
  std::vector<std::string> manager_filters;
  // Applied to the two distance scenarios only; the contact-only and penetration scenarios are
  // defined by a zero margin and do not move with this.
  double distance_margin = 0.2;

  const std::string mode_values = "discrete, continuous, or both";
  const std::string test_type_values = "first, closest, all, or all-types";
  const std::string duty_cycle_values = "repeat, sweep, or trajectory";

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];

    // Consume and return the value following a flag, or exit if it is missing.
    auto require_value = [&](const std::string& hint) -> std::string {
      if (i + 1 >= argc)
      {
        TESSERACT_LOG_ERROR("Missing value for {}. {}", arg, hint);
        std::exit(1);
      }
      return argv[++i];
    };

    if (arg == "--mode" || arg == "-m")
    {
      mode = require_value("Use: " + mode_values);
      if (mode != "discrete" && mode != "continuous" && mode != "both")
      {
        TESSERACT_LOG_ERROR("Invalid mode '{}'. Use: {}", mode, mode_values);
        return 1;
      }
    }
    else if (arg == "--test-type" || arg == "-t")
    {
      test_type = require_value("Use: " + test_type_values);
      if (test_type != "first" && test_type != "closest" && test_type != "all" && test_type != "all-types")
      {
        TESSERACT_LOG_ERROR("Invalid test-type '{}'. Use: {}", test_type, test_type_values);
        return 1;
      }
    }
    else if (arg == "--seed" || arg == "-s")
    {
      seed = std::stoi(require_value("Provide an integer seed."));
    }
    else if (arg == "--clone")
    {
      clone_per_state = true;
    }
    else if (arg == "--duty-cycle" || arg == "-d")
    {
      const std::string value = require_value("Use: " + duty_cycle_values);
      duty_cycle_explicit = true;
      if (value == "repeat")
        duty_cycle.cycle = DutyCycle::REPEAT;
      else if (value == "sweep")
        duty_cycle.cycle = DutyCycle::SWEEP;
      else if (value == "trajectory")
        duty_cycle.cycle = DutyCycle::TRAJECTORY;
      else
      {
        TESSERACT_LOG_ERROR("Unknown duty cycle '{}'. Expected {}.", value, duty_cycle_values);
        return 1;
      }
    }
    else if (arg == "--waypoints" || arg == "-w")
    {
      const int value = std::stoi(require_value("Provide a positive integer."));
      if (value < 1)
      {
        TESSERACT_LOG_ERROR("--waypoints must be at least 1, got {}.", value);
        return 1;
      }
      duty_cycle.waypoints = static_cast<unsigned int>(value);
    }
    else if (arg == "--margin")
    {
      const std::string value = require_value("Provide a non-negative distance in metres.");
      const double parsed = std::stod(value);
      // Rejects NaN and infinity as well as negatives: a margin that is not a real length would be
      // written into the scenario label and mislabel every row the run produces.
      if (!(parsed >= 0.0 && parsed < 1e6))
      {
        TESSERACT_LOG_ERROR("--margin must be a finite, non-negative distance in metres, got '{}'.", value);
        return 1;
      }
      distance_margin = parsed;
    }
    else if (arg == "--manager" || arg == "-M")
    {
      const std::string value = require_value("Provide a comma separated list of manager name substrings.");
      std::stringstream token_stream(value);
      std::string token;
      while (std::getline(token_stream, token, ','))
      {
        std::transform(token.begin(), token.end(), token.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
        if (!token.empty())
          manager_filters.push_back(token);
      }

      if (manager_filters.empty())
      {
        TESSERACT_LOG_ERROR("No manager names found in '{}'.", value);
        return 1;
      }
    }
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: " << argv[0]
                << " [CSV_PATH] [--mode discrete|continuous|both] [--test-type first|closest|all|all-types] [--seed N] "
                   "[--manager NAMES] [--clone] [--duty-cycle repeat|sweep|trajectory] [--waypoints N] "
                   "[--margin M]\n"
                << "  CSV_PATH        Output CSV file (default: tesseract_collision_benchmark.csv)\n"
                << "  --mode/-m       Benchmark mode: " << mode_values << " (default: both)\n"
                << "  --test-type/-t  Contact test type filter: " << test_type_values << " (default: all-types)\n"
                << "  --seed/-s       Fixed RNG seed for reproducible robot states (default: time-based)\n"
                << "  --manager/-M    Only benchmark managers whose name contains one of these comma separated,\n"
                << "                  case insensitive substrings (default: all managers)\n"
                << "  --clone         Clone manager per state pair in continuous mode (simulates TrajOpt).\n"
                << "                  Implies --duty-cycle repeat for the whole run (discrete rows included,\n"
                << "                  even though cloning itself only happens in continuous mode) unless\n"
                << "                  --duty-cycle is given explicitly, since sweep would clone once per trial\n"
                << "                  instead of once per state pair.\n"
                << "  --duty-cycle/-d Transform update frequency: repeat (set once per sampled state),\n"
                << "                  sweep (advance to the next sampled state before every check), or\n"
                << "                  trajectory (advance along a joint-space interpolation between\n"
                << "                  consecutive sampled states before every check) (default: sweep)\n"
                << "  --waypoints/-w  Interpolated configurations per segment for --duty-cycle trajectory.\n"
                << "                  Must divide " << trials << " so the total check count matches the other\n"
                << "                  duty cycles. 1 reproduces sweep. (default: 1)\n"
                << "  --margin        Collision margin for the two distance scenarios, in metres (default: 0.2).\n"
                << "                  The contact-only and penetration scenarios always run at zero margin.\n"
                << "                  The scenario label and the CSV carry the value actually used.\n"
                << "  --help/-h       Show this help message and exit\n";
      return 0;
    }
    else if (!arg.empty() && arg[0] == '-')
    {
      TESSERACT_LOG_ERROR("Unknown option '{}'. Use --help for usage.", arg);
      return 1;
    }
    else
    {
      csv_path = arg;
    }
  }

  const std::vector<std::string> all_manager_names{ "BulletDiscreteBVHManager", "BulletDiscreteSimpleManager",
                                                    "FCLDiscreteBVHManager",    "CoalDiscreteBVHManager",
                                                    "BulletCastBVHManager",     "CoalCastBVHManager" };
  for (const std::string& filter : manager_filters)
  {
    const bool matches_any =
        std::any_of(all_manager_names.begin(), all_manager_names.end(), [&filter](const std::string& manager_name) {
          return managerSelected({ filter }, manager_name);
        });

    if (!matches_any)
    {
      TESSERACT_LOG_ERROR("No manager matches '{}'. Available managers: BulletDiscreteBVHManager, "
                          "BulletDiscreteSimpleManager, FCLDiscreteBVHManager, CoalDiscreteBVHManager, "
                          "BulletCastBVHManager, CoalCastBVHManager",
                          filter);
      return 1;
    }
  }

  if (seed >= 0)
  {
    tesseract::common::mersenne.seed(static_cast<std::mt19937::result_type>(seed));
    TESSERACT_LOG_INFO("Using fixed RNG seed: {}", seed);
  }

  // --clone models TrajOpt cloning the manager once per state pair. Under sweep the loop order
  // inverts and every trial would re-clone, so an unqualified --clone means repeat instead.
  //
  // This implication applies to the whole run, not just continuous mode, even though cloning
  // itself only happens there: under --mode both it also switches the discrete rows to repeat.
  // Scoping it to continuous mode would let a single run emit discrete rows under sweep and
  // continuous rows under repeat in the same CSV, breaking the one-run-one-duty-cycle invariant
  // the duty_cycle column and the plot script's mixed-duty-cycle refusal both depend on. A whole
  // run under one labelled duty cycle is worth more than avoiding this surprise.
  const bool duty_cycle_implied_by_clone = clone_per_state && !duty_cycle_explicit;
  if (duty_cycle_implied_by_clone)
    duty_cycle.cycle = DutyCycle::REPEAT;

  TESSERACT_LOG_INFO(
      "Duty cycle: {}{}", dutyCycleName(duty_cycle), duty_cycle_implied_by_clone ? " (implied by --clone)" : "");

  const bool run_discrete = (mode == "discrete" || mode == "both");
  const bool run_continuous = (mode == "continuous" || mode == "both");

  // Only continuous mode visits state pairs, so --clone only conflicts with an advancing duty
  // cycle when continuous mode will actually run.
  if (run_continuous && duty_cycle.cycle != DutyCycle::REPEAT && clone_per_state)
  {
    TESSERACT_LOG_ERROR("--clone and --duty-cycle {} are mutually exclusive: an advancing duty cycle visits "
                        "every state pair once per trial, so cloning per visit would dominate the measurement.",
                        dutyCycleName(duty_cycle));
    return 1;
  }

  // Built once and interpolated into all four distance scenario labels, so a CSV can never report a
  // margin the run did not use.
  std::ostringstream margin_stream;
  margin_stream << distance_margin;
  const std::string margin_label = margin_stream.str() + " m";

  // Holding the total check count equal across duty cycles is what makes their checks_per_second
  // comparable: a trajectory run walks states * waypoints configurations, so it gets
  // proportionally fewer passes over them.
  if (duty_cycle.cycle == DutyCycle::TRAJECTORY && (trials % duty_cycle.waypoints) != 0)
  {
    TESSERACT_LOG_ERROR("--waypoints {} does not divide the {} trials per state, so the run would not perform "
                        "the same number of checks as the other duty cycles.",
                        duty_cycle.waypoints,
                        trials);
    return 1;
  }

  const bool run_first = (test_type == "first" || test_type == "all-types");
  const bool run_closest = (test_type == "closest" || test_type == "all-types");
  const bool run_all = (test_type == "all" || test_type == "all-types");

  std::ofstream csv_file(csv_path);
  if (!csv_file.is_open())
  {
    TESSERACT_LOG_ERROR("Failed to open CSV file: {}", csv_path);
    return 1;
  }

  // ************************************************
  // SETUP TESSERACT ENVIRONMENT
  // ************************************************
  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  auto urdf = locator->locateResource("package://tesseract_collision_benchmarks/data/panda.urdf");
  auto srdf = locator->locateResource("package://tesseract_collision_benchmarks/data/panda.srdf");

  tesseract::environment::Environment tesseract_env;
  tesseract_env.init(std::filesystem::path(urdf->getFilePath()), std::filesystem::path(srdf->getFilePath()), locator);
  std::vector<tesseract::common::LinkId> link_ids = tesseract_env.getActiveLinkIds();

  // Exclude robot collisions
  tesseract::common::AllowedCollisionMatrix modify_ac;
  for (std::size_t i = 0; i < link_ids.size() - 1; ++i)
    for (std::size_t j = i + 1; j < link_ids.size(); ++j)
      modify_ac.addAllowedCollision(link_ids[i], link_ids[j], "exclude robot links");

  auto cmd = std::make_shared<tesseract::environment::ModifyAllowedCollisionsCommand>(
      modify_ac, tesseract::environment::ModifyAllowedCollisionsType::ADD);
  tesseract_env.applyCommand(cmd);

  tesseract::scene_graph::StateSolver::Ptr tesseract_state_solver = tesseract_env.getStateSolver();

  std::vector<tesseract::collision::DiscreteContactManager::UPtr> contact_checkers;
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("BulletDiscreteBVHManager"));
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("BulletDiscreteSimpleManager"));
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("FCLDiscreteBVHManager"));
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("CoalDiscreteBVHManager"));
  // contact_checkers.push_back(tesseract_env.getDiscreteContactManager("PhysxDiscreteManager"));

  std::vector<tesseract::geometry::Geometry::ConstPtr> shapes;
  tesseract::common::VectorIsometry3d shape_poses;
  clutterWorld(shapes,
               shape_poses,
               contact_checkers.front()->clone(),
               tesseract_state_solver->clone(),
               num_states,
               tesseract::collision::CollisionObjectType::CONVEX_MESH);

  for (auto& contact_checker : contact_checkers)
  {
    contact_checker->addCollisionObject("world", 0, shapes, shape_poses);
    contact_checker->setDefaultCollisionMargin(0);
    contact_checker->setActiveCollisionObjects(link_ids);
  }

  TESSERACT_LOG_INFO("Starting benchmark: Robot in cluttered world, in collision with world");

  sleep(1);

  std::vector<tesseract::common::LinkIdTransformMap> t_sampled_states;
  std::vector<tesseract::scene_graph::SceneState::JointValues> t_sampled_joint_values;
  int states_in_collision = findStates(t_sampled_states,
                                       t_sampled_joint_values,
                                       tesseract::collision::RobotStateSelector::IN_COLLISION,
                                       num_states,
                                       contact_checkers.front()->clone(),
                                       tesseract_state_solver->clone());

  for (auto& s : t_sampled_states)
    s.erase("world");

  // Every duty cycle walks bench_states cyclically for bench_trials passes. Only the contents of
  // that vector differ: the sampled states themselves, or a joint-space interpolation through
  // them. The scenario strings keep quoting the sampled-state count, so a trajectory run stays
  // joinable with the runs it is being compared against.
  std::vector<tesseract::common::LinkIdTransformMap> t_trajectory_states;
  if (duty_cycle.cycle == DutyCycle::TRAJECTORY)
  {
    t_trajectory_states = tesseract::collision::buildTrajectoryStates(
        t_sampled_joint_values, duty_cycle.waypoints, *tesseract_state_solver);
    for (auto& s : t_trajectory_states)
      s.erase("world");

    TESSERACT_LOG_INFO(
        "Expanded {} sampled states into {} trajectory waypoints", t_sampled_states.size(), t_trajectory_states.size());
  }

  const std::vector<tesseract::common::LinkIdTransformMap>& bench_states =
      (duty_cycle.cycle == DutyCycle::TRAJECTORY) ? t_trajectory_states : t_sampled_states;
  const unsigned int bench_trials =
      (duty_cycle.cycle == DutyCycle::TRAJECTORY) ? (trials / duty_cycle.waypoints) : trials;

  for (auto& contact_checker : contact_checkers)
    contact_checker->setDefaultCollisionMargin(0);

  // The manager list must stay complete until this point: clutterWorld() and findStates() clone
  // contact_checkers.front(), so removing entries earlier would change which backend decides the
  // generated world and the sampled robot states for a given seed.
  contact_checkers.erase(std::remove_if(contact_checkers.begin(),
                                        contact_checkers.end(),
                                        [&manager_filters](const auto& contact_checker) {
                                          return !managerSelected(manager_filters, contact_checker->getName());
                                        }),
                         contact_checkers.end());

  csv_file << "scenario,manager,mode,checks_per_second,total_num_checks,num_contacts,duty_cycle\n";

  std::ostringstream scenario;

  // ************************************************
  // DISCRETE COLLISION BENCHMARKS
  // ************************************************
  if (run_discrete)
  {
    scenario << "Discrete: Contact Only, " << states_in_collision << " out of " << t_sampled_states.size()
             << " states in collision";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       false,
                                       false,
                                       is_physx,
                                       duty_cycle);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       false,
                                       false,
                                       is_physx,
                                       duty_cycle);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       false,
                                       false,
                                       is_physx,
                                       duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    scenario.str("");
    scenario << "Discrete: Penetration Enabled, " << states_in_collision << " out of " << t_sampled_states.size()
             << " states in collision";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       false,
                                       true,
                                       is_physx,
                                       duty_cycle);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       false,
                                       true,
                                       is_physx,
                                       duty_cycle);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       false,
                                       true,
                                       is_physx,
                                       duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    scenario.str("");
    scenario << "Discrete: Distance (" << margin_label << ") Enabled, " << states_in_collision << " out of "
             << t_sampled_states.size() << " states in collision";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& contact_checker : contact_checkers)
      contact_checker->setDefaultCollisionMargin(distance_margin);

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       true,
                                       false,
                                       is_physx,
                                       duty_cycle);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       true,
                                       false,
                                       is_physx,
                                       duty_cycle);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       true,
                                       false,
                                       is_physx,
                                       duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    scenario.str("");
    scenario << "Discrete: Distance (" << margin_label << ") and Penetration Enabled, " << states_in_collision
             << " out of " << t_sampled_states.size() << " states in collision";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       true,
                                       true,
                                       is_physx,
                                       duty_cycle);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       true,
                                       true,
                                       is_physx,
                                       duty_cycle);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       bench_trials,
                                       *contact_checker,
                                       bench_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       true,
                                       true,
                                       is_physx,
                                       duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");
  }  // run_discrete

  // ************************************************
  // CONTINUOUS COLLISION BENCHMARKS
  // ************************************************
  if (run_continuous)
  {
    // Filtering at construction is safe here: nothing clones cast_checkers.front(), and the state pairs
    // come from states that were already sampled.
    std::vector<tesseract::collision::ContinuousContactManager::UPtr> cast_checkers;
    if (managerSelected(manager_filters, "BulletCastBVHManager"))
      cast_checkers.push_back(tesseract_env.getContinuousContactManager("BulletCastBVHManager"));
    if (managerSelected(manager_filters, "CoalCastBVHManager"))
      cast_checkers.push_back(tesseract_env.getContinuousContactManager("CoalCastBVHManager"));

    for (auto& checker : cast_checkers)
    {
      checker->addCollisionObject("world", 0, shapes, shape_poses);
      checker->setDefaultCollisionMargin(0);
      checker->setActiveCollisionObjects(link_ids);
    }

    // Build state pairs from consecutive states of whichever vector this duty cycle walks. Under
    // trajectory that shortens the swept volume as well as the jump between checks, which is what a
    // cast check against consecutive trajectory waypoints actually looks like.
    std::vector<std::pair<tesseract::common::LinkIdTransformMap, tesseract::common::LinkIdTransformMap>> state_pairs;
    for (std::size_t i = 0; i + 1 < bench_states.size(); ++i)
      state_pairs.emplace_back(bench_states[i], bench_states[i + 1]);

    TESSERACT_LOG_INFO("Starting continuous collision benchmarks with {} state pairs", state_pairs.size());

    sleep(1);

    // Scenario 1: Continuous Contact Only
    for (auto& checker : cast_checkers)
      checker->setDefaultCollisionMargin(0);

    scenario.str("");
    scenario << "Continuous: Contact Only, " << state_pairs.size() << " state pairs";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 false,
                                                 false,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 false,
                                                 false,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 false,
                                                 false,
                                                 clone_per_state,
                                                 duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    // Scenario 2: Continuous Penetration Enabled
    scenario.str("");
    scenario << "Continuous: Penetration Enabled, " << state_pairs.size() << " state pairs";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 false,
                                                 true,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 false,
                                                 true,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 false,
                                                 true,
                                                 clone_per_state,
                                                 duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    // Scenario 3: Continuous Distance Enabled
    for (auto& checker : cast_checkers)
      checker->setDefaultCollisionMargin(distance_margin);

    scenario.str("");
    scenario << "Continuous: Distance (" << margin_label << ") Enabled, " << state_pairs.size() << " state pairs";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 true,
                                                 false,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 true,
                                                 false,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 true,
                                                 false,
                                                 clone_per_state,
                                                 duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    // Scenario 4: Continuous Distance and Penetration Enabled
    scenario.str("");
    scenario << "Continuous: Distance (" << margin_label << ") and Penetration Enabled, " << state_pairs.size()
             << " state pairs";

    TESSERACT_LOG_INFO("Starting scenario: {}", scenario.str());
    TESSERACT_LOG_INFO(
        "{:<40} | {:>17} | {:>16} | {:>12}", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 true,
                                                 true,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 true,
                                                 true,
                                                 clone_per_state,
                                                 duty_cycle);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 bench_trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 true,
                                                 true,
                                                 clone_per_state,
                                                 duty_cycle);
    }
    TESSERACT_LOG_INFO("-----------------------------------------+-------------------+------------------+--------"
                       "-----");
  }  // run_continuous

  TESSERACT_LOG_INFO("CSV results written to {}", csv_path);

  return 0;
}
