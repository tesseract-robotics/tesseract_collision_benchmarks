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
#include <console_bridge/console.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
#include <unordered_set>

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
  CONSOLE_BRIDGE_logInform("Cluttering scene...");

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
      CONSOLE_BRIDGE_logInform("Object was in collision, remove");
    }
    contact_checker->removeCollisionObject(name);
    i++;
  }
  CONSOLE_BRIDGE_logInform("Cluttered the planning scene with %d objects", added_objects);
}

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param robot_states Result vector
 *  \return number of state in collision
 */
int findStates(std::vector<tesseract::common::LinkIdTransformMap>& robot_states,
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
          ++states_in_collision;
        }
        break;
      case RobotStateSelector::NOT_IN_COLLISION:
        if (t_res.empty())
          robot_states.push_back(t_transforms);
        break;
      case RobotStateSelector::RANDOM:
        robot_states.push_back(t_transforms);
        if (!t_res.empty())
          ++states_in_collision;
        break;
    }
    i++;
  }

  return states_in_collision;
}
}  // namespace tesseract::collision

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
                                    bool distance = false,
                                    bool penetration = false,
                                    bool is_physx = false)
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
  if (is_physx)
  {
    // Physx requires that all active links be set prior to contact test, because in physx links can go to
    // sleep if they have not moved in 3-4 contact test requests.
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
    // This is more representative of moveit because the state transforms only get updated the first time it is
    // called and does not update them for subsequent request becasuse the joint values have not change
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
             << "," << total_num_checks << "," << contact_count << "\n";

  CONSOLE_BRIDGE_logInform(
      "%-40s | %17.0f | %16zu | %12zu", desc.c_str(), checks_per_second, total_num_checks, contact_count);
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
    bool distance = false,
    bool penetration = false,
    bool clone_per_state = false)
{
  std::string ct = tesseract::collision::ContactTestTypeStrings.at(static_cast<std::size_t>(test_type));
  std::string desc = name + "(" + ct + ")";

  tesseract::collision::ContactResultMap res;
  tesseract::collision::ContactRequest req(test_type);
  req.calculate_distance = distance;
  req.calculate_penetration = penetration;

  // Pre-compute active link ID set for fast lookup
  std::unordered_set<tesseract::common::LinkId> active_link_set(active_links.begin(), active_links.end());

  tesseract::common::Stopwatch stopwatch;
  stopwatch.start();
  for (const auto& [pose1, pose2] : state_pairs)
  {
    auto cloned = clone_per_state ? checker.clone() : nullptr;
    auto& active_checker = clone_per_state ? *cloned : checker;

    // Active links get cast (moving) transforms; others get static transforms
    for (const auto& tf : pose1)
    {
      if (active_link_set.count(tf.first) != 0)
        active_checker.setCollisionObjectsTransform(tf.first, tf.second, pose2.at(tf.first));
      else
        active_checker.setCollisionObjectsTransform(tf.first, tf.second);
    }
    for (unsigned int i = 0; i < trials; ++i)
    {
      res.clear();
      active_checker.contactTest(res, req);
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
             << "," << total_num_checks << "," << contact_count << "\n";

  CONSOLE_BRIDGE_logInform(
      "%-40s | %17.0f | %16zu | %12zu", desc.c_str(), checks_per_second, total_num_checks, contact_count);
}

int main(int argc, char** argv)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  const unsigned int trials = 1000;
  const unsigned int num_states = 50;

  std::string csv_path = "tesseract_collision_benchmark.csv";
  std::string mode = "both";
  std::string test_type = "all-types";
  int seed = -1;
  bool clone_per_state = false;

  for (int i = 1; i < argc; ++i)
  {
    if ((std::string(argv[i]) == "--mode" || std::string(argv[i]) == "-m") && i + 1 < argc)
    {
      mode = argv[++i];
      if (mode != "discrete" && mode != "continuous" && mode != "both")
      {
        CONSOLE_BRIDGE_logError("Invalid mode '%s'. Use: discrete, continuous, or both", mode.c_str());
        return 1;
      }
    }
    else if ((std::string(argv[i]) == "--test-type" || std::string(argv[i]) == "-t") && i + 1 < argc)
    {
      test_type = argv[++i];
      if (test_type != "first" && test_type != "closest" && test_type != "all" && test_type != "all-types")
      {
        CONSOLE_BRIDGE_logError("Invalid test-type '%s'. Use: first, closest, all, or all-types", test_type.c_str());
        return 1;
      }
    }
    else if ((std::string(argv[i]) == "--seed" || std::string(argv[i]) == "-s") && i + 1 < argc)
    {
      seed = std::stoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--clone")
    {
      clone_per_state = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
    {
      std::cout << "Usage: " << argv[0]
                << " [CSV_PATH] [--mode discrete|continuous|both] [--test-type first|closest|all|all-types] [--seed N] "
                   "[--clone]\n"
                << "  CSV_PATH        Output CSV file (default: tesseract_collision_benchmark.csv)\n"
                << "  --mode/-m       Benchmark mode: discrete, continuous, or both (default: both)\n"
                << "  --test-type/-t  Contact test type filter: first, closest, all, or all-types (default: "
                   "all-types)\n"
                << "  --seed/-s       Fixed RNG seed for reproducible robot states (default: time-based)\n"
                << "  --clone         Clone manager per state pair in continuous mode (simulates TrajOpt)\n";
      return 0;
    }
    else if (argv[i][0] != '-')
    {
      csv_path = argv[i];
    }
  }

  if (seed >= 0)
  {
    tesseract::common::mersenne.seed(static_cast<std::mt19937::result_type>(seed));
    CONSOLE_BRIDGE_logInform("Using fixed RNG seed: %d", seed);
  }

  const bool run_discrete = (mode == "discrete" || mode == "both");
  const bool run_continuous = (mode == "continuous" || mode == "both");
  const bool run_first = (test_type == "first" || test_type == "all-types");
  const bool run_closest = (test_type == "closest" || test_type == "all-types");
  const bool run_all = (test_type == "all" || test_type == "all-types");

  std::ofstream csv_file(csv_path);
  if (!csv_file.is_open())
  {
    CONSOLE_BRIDGE_logError("Failed to open CSV file: %s", csv_path.c_str());
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

  CONSOLE_BRIDGE_logInform("Starting benchmark: Robot in cluttered world, in collision with world");

  sleep(1);

  std::vector<tesseract::common::LinkIdTransformMap> t_sampled_states;
  int states_in_collision = findStates(t_sampled_states,
                                       tesseract::collision::RobotStateSelector::IN_COLLISION,
                                       num_states,
                                       contact_checkers.front()->clone(),
                                       tesseract_state_solver->clone());

  for (auto& s : t_sampled_states)
    s.erase("world");

  for (auto& contact_checker : contact_checkers)
    contact_checker->setDefaultCollisionMargin(0);

  csv_file << "scenario,manager,mode,checks_per_second,total_num_checks,num_contacts\n";

  std::ostringstream scenario;

  // ************************************************
  // DISCRETE COLLISION BENCHMARKS
  // ************************************************
  if (run_discrete)
  {
    scenario << "Contact Only, " << states_in_collision << " out of " << t_sampled_states.size()
             << " states in collision";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       false,
                                       false,
                                       is_physx);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       false,
                                       false,
                                       is_physx);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       false,
                                       false,
                                       is_physx);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    scenario.str("");
    scenario << "Penetration Enabled, " << states_in_collision << " out of " << t_sampled_states.size()
             << " states in collision";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       false,
                                       true,
                                       is_physx);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       false,
                                       true,
                                       is_physx);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       false,
                                       true,
                                       is_physx);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    scenario.str("");
    scenario << "Distance (0.2 m) Enabled, " << states_in_collision << " out of " << t_sampled_states.size()
             << " states in collision";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& contact_checker : contact_checkers)
      contact_checker->setDefaultCollisionMargin(0.2);

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       true,
                                       false,
                                       is_physx);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       true,
                                       false,
                                       is_physx);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       true,
                                       false,
                                       is_physx);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    scenario.str("");
    scenario << "Distance (0.2 m) and Penetration Enabled, " << states_in_collision << " out of "
             << t_sampled_states.size() << " states in collision";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& contact_checker : contact_checkers)
    {
      const bool is_physx{ contact_checker->getName() == "PhysxDiscreteManager" };
      if (run_first)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::FIRST,
                                       true,
                                       true,
                                       is_physx);
      if (run_closest)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::CLOSEST,
                                       true,
                                       true,
                                       is_physx);
      if (run_all)
        runTesseractCollisionDetection(csv_file,
                                       contact_checker->getName(),
                                       scenario.str(),
                                       trials,
                                       *contact_checker,
                                       t_sampled_states,
                                       tesseract::collision::ContactTestType::ALL,
                                       true,
                                       true,
                                       is_physx);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");
  }  // run_discrete

  // ************************************************
  // CONTINUOUS COLLISION BENCHMARKS
  // ************************************************
  if (run_continuous)
  {
    std::vector<tesseract::collision::ContinuousContactManager::UPtr> cast_checkers;
    cast_checkers.push_back(tesseract_env.getContinuousContactManager("BulletCastBVHManager"));
    cast_checkers.push_back(tesseract_env.getContinuousContactManager("CoalCastBVHManager"));

    for (auto& checker : cast_checkers)
    {
      checker->addCollisionObject("world", 0, shapes, shape_poses);
      checker->setDefaultCollisionMargin(0);
      checker->setActiveCollisionObjects(link_ids);
    }

    // Build state pairs from consecutive sampled states
    std::vector<std::pair<tesseract::common::LinkIdTransformMap, tesseract::common::LinkIdTransformMap>> state_pairs;
    for (std::size_t i = 0; i + 1 < t_sampled_states.size(); ++i)
      state_pairs.emplace_back(t_sampled_states[i], t_sampled_states[i + 1]);

    CONSOLE_BRIDGE_logInform("Starting continuous collision benchmarks with %zu state pairs", state_pairs.size());

    sleep(1);

    // Scenario 1: Continuous Contact Only
    for (auto& checker : cast_checkers)
      checker->setDefaultCollisionMargin(0);

    scenario.str("");
    scenario << "Continuous: Contact Only, " << state_pairs.size() << " state pairs";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 false,
                                                 false,
                                                 clone_per_state);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 false,
                                                 false,
                                                 clone_per_state);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 false,
                                                 false,
                                                 clone_per_state);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    // Scenario 2: Continuous Penetration Enabled
    scenario.str("");
    scenario << "Continuous: Penetration Enabled, " << state_pairs.size() << " state pairs";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 false,
                                                 true,
                                                 clone_per_state);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 false,
                                                 true,
                                                 clone_per_state);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 false,
                                                 true,
                                                 clone_per_state);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    // Scenario 3: Continuous Distance (0.2 m) Enabled
    for (auto& checker : cast_checkers)
      checker->setDefaultCollisionMargin(0.2);

    scenario.str("");
    scenario << "Continuous: Distance (0.2 m) Enabled, " << state_pairs.size() << " state pairs";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 true,
                                                 false,
                                                 clone_per_state);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 true,
                                                 false,
                                                 clone_per_state);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 true,
                                                 false,
                                                 clone_per_state);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    // Scenario 4: Continuous Distance (0.2 m) and Penetration Enabled
    scenario.str("");
    scenario << "Continuous: Distance (0.2 m) and Penetration Enabled, " << state_pairs.size() << " state pairs";

    CONSOLE_BRIDGE_logInform("Starting scenario: %s", scenario.str().c_str());
    CONSOLE_BRIDGE_logInform(
        "%-40s | %17s | %16s | %12s", "Description", "Checks Per Second", "Total Num Checks", "Num Contacts");
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");

    for (auto& checker : cast_checkers)
    {
      if (run_first)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::FIRST,
                                                 true,
                                                 true,
                                                 clone_per_state);
      if (run_closest)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::CLOSEST,
                                                 true,
                                                 true,
                                                 clone_per_state);
      if (run_all)
        runTesseractContinuousCollisionDetection(csv_file,
                                                 checker->getName(),
                                                 scenario.str(),
                                                 trials,
                                                 *checker,
                                                 state_pairs,
                                                 link_ids,
                                                 tesseract::collision::ContactTestType::ALL,
                                                 true,
                                                 true,
                                                 clone_per_state);
    }
    CONSOLE_BRIDGE_logInform("-----------------------------------------+-------------------+------------------+--------"
                             "-----");
  }  // run_continuous

  CONSOLE_BRIDGE_logInform("CSV results written to %s", csv_path.c_str());

  return 0;
}
