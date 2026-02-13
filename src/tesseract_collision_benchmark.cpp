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

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/stopwatch.h>
#include <tesseract_collision_benchmark/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_collision/coal/coal_discrete_managers.h>
// #include <tesseract_collision_physx/physx_discrete_manager.h>

#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands/modify_allowed_collisions_command.h>

#include <tesseract_collision_benchmark/types.h>

namespace tesseract_collision
{

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
                  const DiscreteContactManager::Ptr &contact_checker,
                  const tesseract_scene_graph::StateSolver::Ptr &state_solver,
                  const std::size_t num_objects,
                  CollisionObjectType type)
{
    CONSOLE_BRIDGE_logInform("Cluttering scene...");

    random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(123);

    // allow all robot links to be in collision for world check
    collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
        planning_scene->getRobotModel()->getLinkModelNames(), true) };

    // set the robot state to home position
    moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
    collision_detection::CollisionRequest req;
    current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
    current_state.update();

    auto t_env_state = state_solver->getState(current_state.getVariableNames(), Eigen::Map<Eigen::VectorXd>(current_state.getVariablePositions(), static_cast<long>(current_state.getVariableNames().size())));
    contact_checker->setCollisionObjectsTransform(t_env_state.link_transforms);

    // load panda link5 as world collision object
    std::string name;
    shapes::ShapeConstPtr shape;
    tesseract_geometry::Geometry::Ptr t_shape;
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
            shapes::Mesh* mesh = shapes::createMeshFromResource(kinect);
            mesh->scale(num_generator.uniformReal(0.3, 1.0));
            shape.reset(mesh);
            name = "mesh";

            // Do not need to scale since we are using data from mesh above
            auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
            auto triangles = std::make_shared<Eigen::VectorXi>();

            vertices->reserve(mesh->vertex_count);
            triangles->resize(mesh->triangle_count * 4);
            for (std::size_t i = 0; i < mesh->vertex_count; ++i)
                vertices->emplace_back(mesh->vertices[3*i], mesh->vertices[(3*i) + 1], mesh->vertices[(3*i) + 2]);

            for (std::size_t i = 0; i < mesh->triangle_count; ++i)
            {
                (*triangles)[static_cast<int>((i*4))] = 3;
                (*triangles)[static_cast<int>((i*4) + 1)] = static_cast<int>(mesh->triangles[(i*3)]);
                (*triangles)[static_cast<int>((i*4) + 2)] = static_cast<int>(mesh->triangles[(i*3) + 1]);
                (*triangles)[static_cast<int>((i*4) + 3)] = static_cast<int>(mesh->triangles[(i*3) + 2]);
            }

            if (type == CollisionObjectType::MESH)
            {
                auto t_mesh = std::make_shared<tesseract_geometry::Mesh>(vertices, triangles);
                t_shape = t_mesh;
            }
            else
            {
                // This is required because convex hull cannot have multiple faces on the same plane.
                auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
                auto ch_faces = std::make_shared<Eigen::VectorXi>();
                int ch_num_faces = tesseract_collision::createConvexHull(*ch_verticies, *ch_faces, *vertices);
                auto t_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
                t_shape = t_mesh;
            }
            break;
        }
        case CollisionObjectType::BOX:
        {
            double x = num_generator.uniformReal(0.05, 0.2);
            double y = num_generator.uniformReal(0.05, 0.2);
            double z = num_generator.uniformReal(0.05, 0.2);
            shape.reset(new shapes::Box(x, y, z));
            name = "box";

            t_shape = std::make_shared<tesseract_geometry::Box>(x, y ,z);
            break;
        }
        }

        name.append(std::to_string(i));
        planning_scene->getWorldNonConst()->addToObject(name, shape, pos);
        contact_checker->addCollisionObject(name, 0, {t_shape}, {pos});

        // try if it isn't in collision if yes, ok, if no remove.
        collision_detection::CollisionResult res;
        planning_scene->checkCollision(req, res, current_state, acm);

        tesseract_collision::ContactRequest t_req(tesseract_collision::ContactTestType::FIRST);
        tesseract_collision::ContactResultMap t_res;
        contact_checker->contactTest(t_res, t_req);

        if (!res.collision && t_res.empty())
        {
            added_objects++;
            shapes.push_back(t_shape);
            shape_poses.push_back(pos);
        }
        else
        {
            CONSOLE_BRIDGE_logInform("Object was in collision, remove");
            planning_scene->getWorldNonConst()->removeObject(name);
        }
        contact_checker->removeCollisionObject(name);
        i++;
    }
    CONSOLE_BRIDGE_logInform("Cluttered the planning scene with %d objects", added_objects);
}

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
               const tesseract_scene_graph::StateSolver::Ptr &state_solver)
{
    moveit::core::RobotState& current_state{ scene->getCurrentStateNonConst() };
    collision_detection::CollisionRequest req;

    size_t i{ 0 };
    int states_in_collision {0};
    while (robot_states.size() < num_states)
    {
        current_state.setToRandomPositions();
        current_state.update();
        collision_detection::CollisionResult res;
        scene->checkSelfCollision(req, res);

        auto t_env_state = state_solver->getState(current_state.getVariableNames(), Eigen::Map<Eigen::VectorXd>(current_state.getVariablePositions(), static_cast<long>(current_state.getVariableNames().size())));
        contact_checker->setCollisionObjectsTransform(t_env_state.link_transforms);
        tesseract_collision::ContactRequest t_req(tesseract_collision::ContactTestType::FIRST);
        tesseract_collision::ContactResultMap t_res;
        contact_checker->contactTest(t_res, t_req);

        switch (desired_states)
        {
        case RobotStateSelector::IN_COLLISION:
            if (res.collision && !t_res.empty())
            {
                robot_states.push_back(current_state);
                ++states_in_collision;
            }
            break;
        case RobotStateSelector::NOT_IN_COLLISION:
            if (!res.collision && t_res.empty())
                robot_states.push_back(current_state);
            break;
        case RobotStateSelector::RANDOM:
            robot_states.push_back(current_state);
            if (res.collision && !t_res.empty())
                ++states_in_collision;
            break;
        }
        i++;
    }

    return states_in_collision;
}
}

/** \brief Runs a collision detection benchmark and measures the time.
*
*   \param trials The number of repeated collision checks for each state
*   \param scene The planning scene
*   \param CollisionDetector The type of collision detector
*   \param distance Turn on distance */
void runCollisionDetection(unsigned int trials,
                           const planning_scene::PlanningScenePtr& scene,
                           const std::vector<moveit::core::RobotState>& states,
                           const tesseract_collision::CollisionDetector col_detector,
                           tesseract_collision::ContactTestType test_type,
                           bool distance = false,
                           bool contacts = false)
{
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      scene->getRobotModel()->getLinkModelNames(), true) };

  std::string ct = tesseract_collision::ContactTestTypeStrings.at(static_cast<std::size_t>(test_type));
  std::string desc = (col_detector == tesseract_collision::CollisionDetector::FCL ? "MoveIt FCL (" + ct + ")" : "MoveIt Bullet (" + ct + ")");

  if (col_detector == tesseract_collision::CollisionDetector::FCL)
  {
    scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
  }
  else
  {
    scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
  }

  collision_detection::CollisionResult res;
  collision_detection::CollisionRequest req;

  req.distance = distance;
  req.contacts = contacts;
  if (test_type == tesseract_collision::ContactTestType::FIRST)
  {
    req.max_contacts = 1;
    req.max_contacts_per_pair = 1;
  }
  else
  {
    req.contacts = true; // If this is not true it will not generate more than one contact in MoveIt
    req.max_contacts = 300;
    req.max_contacts_per_pair = 300;
  }

  tesseract_common::Stopwatch stopwatch;
  stopwatch.start();
  for (unsigned int i = 0; i < trials; ++i)
  {
    for (auto& state : states)
    {
      res.clear();
      scene->checkCollision(req, res, state);
      if (res.collision && test_type == tesseract_collision::ContactTestType::FIRST)
        continue;

      scene->checkSelfCollision(req, res, state);
    }
  }
  stopwatch.stop();
  const double duration = stopwatch.elapsedSeconds();

  const double checks_per_second = static_cast<double>(trials * states.size()) / duration;
  const std::size_t total_num_checks = trials * states.size();
  const std::size_t contact_count = (res.collision == true && res.contact_count == 0) ? 1 : res.contact_count;
  CONSOLE_BRIDGE_logInform("%s, %lf, %ld, %ld", desc.c_str(), checks_per_second, total_num_checks, contact_count);

  // color collided objects red
//  for (auto& contact : res.contacts)
//  {
//    ROS_INFO_STREAM("Between: " << contact.first.first << " and " << contact.first.second);
//    std_msgs::ColorRGBA red;
//    red.a = 0.8f;
//    red.r = 1;
//    red.g = 0;
//    red.b = 0;
//    scene->setObjectColor(contact.first.first, red);
//    scene->setObjectColor(contact.first.second, red);
//  }

  scene->setCurrentState(states.back());
}

/** \brief Runs a collision detection benchmark and measures the time.
*
*   \param name Name to give the benchmark
*   \param trials The number of repeated collision checks for each state
*   \param checker Tesseract contact checker
*   \param state A vector of collision object transforms
*   \param test_type The tesseract contact test type (FIRST, ALL, CLOSEST)
*   \param distance Turn on distance */
void runTesseractCollisionDetection(const std::string& name,
                                    unsigned int trials,
                                    tesseract_collision::DiscreteContactManager& checker,
                                    const std::vector<tesseract_common::TransformMap>& states,
                                    tesseract_collision::ContactTestType test_type,
                                    bool distance = false,
                                    bool contacts = false,
                                    bool is_physx = false)
{
//  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
//      scene->getRobotModel()->getLinkModelNames(), true) };


  std::string ct = tesseract_collision::ContactTestTypeStrings.at(static_cast<std::size_t>(test_type));
  std::string desc = name + "(" + ct + ")";

  tesseract_collision::ContactResultMap res;
  tesseract_collision::ContactRequest req(test_type);
  req.calculate_distance = distance;
  req.calculate_penetration = contacts;

  tesseract_common::Stopwatch stopwatch;
  stopwatch.start();
  if (is_physx)
  {
    // Physx requires that all active links be set prior to contact test, because in physx links can go to
    // sleep if they have not moved in 3-4 contact test requests.
    for (unsigned int i = 0; i < trials; ++i)
    {
      for (auto& state : states)
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
    for (auto& state : states)
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

  CONSOLE_BRIDGE_logInform("%s, %lf, %ld, %ld", desc.c_str(), checks_per_second, total_num_checks, contact_count);
}

int main(int argc, char** argv)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  moveit::core::RobotModelPtr robot_model;
  unsigned int trials = 1000;

  // ************************************************
  // SETUP MOVEIT ENVIRONMENT
  // ************************************************
  robot_model = moveit::core::loadTestingRobotModel("panda");

  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  std::vector<std::string> link_names = robot_model->getLinkModelNames();
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(link_names, true) };
  planning_scene->checkCollision(req, res, planning_scene->getCurrentState(), acm);

  // ************************************************
  // SETUP TESSERACT ENVIRONMENT
  // ************************************************
  auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto urdf = locator->locateResource("package://tesseract_collision_benchmarks/data/panda.urdf");
  auto srdf = locator->locateResource("package://tesseract_collision_benchmarks/data/panda.srdf");

  tesseract_environment::Environment tesseract_env;
  tesseract_env.init(std::filesystem::path(urdf->getFilePath()), std::filesystem::path(srdf->getFilePath()), locator);

  // Exclude robot collisions
  tesseract_common::AllowedCollisionMatrix modify_ac;
  for (std::size_t i = 0; i < link_names.size() - 1; ++i)
    for (std::size_t j = i + 1; j < link_names.size(); ++j)
      modify_ac.addAllowedCollision(link_names[i], link_names[j], "exclude robot links");

  auto cmd = std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(modify_ac, tesseract_environment::ModifyAllowedCollisionsType::ADD);
  tesseract_env.applyCommand(cmd);

  tesseract_scene_graph::StateSolver::Ptr tesseract_state_solver = tesseract_env.getStateSolver();

  std::vector<tesseract_collision::DiscreteContactManager::UPtr> contact_checkers;
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("BulletDiscreteBVHManager"));
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("BulletDiscreteSimpleManager"));
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("FCLDiscreteBVHManager"));
  contact_checkers.push_back(tesseract_env.getDiscreteContactManager("CoalDiscreteBVHManager"));
  // contact_checkers.push_back(tesseract_env.getDiscreteContactManager("PhysxDiscreteManager"));

  std::vector<tesseract_geometry::Geometry::ConstPtr> shapes;
  tesseract_common::VectorIsometry3d shape_poses;
  clutterWorld(shapes, shape_poses, planning_scene, contact_checkers.front()->clone(), tesseract_state_solver->clone(), 50, tesseract_collision::CollisionObjectType::CONVEX_MESH);

  for (auto& contact_checker : contact_checkers)
  {
    contact_checker->addCollisionObject("world", 0, shapes, shape_poses);
    contact_checker->setDefaultCollisionMargin(0);
    contact_checker->setActiveCollisionObjects(link_names);
  }

  CONSOLE_BRIDGE_logInform("Starting...");

  sleep(1);

  moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  current_state.update();

  std::vector<moveit::core::RobotState> sampled_states;
  std::vector<tesseract_common::TransformMap> t_sampled_states;
  int states_in_collision = findStates(sampled_states, tesseract_collision::RobotStateSelector::IN_COLLISION , 50, planning_scene, contact_checkers.front()->clone(), tesseract_state_solver->clone());

  t_sampled_states.clear();
  for (auto& s : sampled_states)
  {
    auto t_env_state = tesseract_state_solver->getState(current_state.getVariableNames(), Eigen::Map<Eigen::VectorXd>(s.getVariablePositions(), static_cast<long>(s.getVariableNames().size())));
    t_env_state.link_transforms.erase("world");
    t_sampled_states.push_back(t_env_state.link_transforms);
  }

  for (auto& contact_checker : contact_checkers)
    contact_checker->setDefaultCollisionMargin(0);


  CONSOLE_BRIDGE_logInform("Starting benchmark: Robot in cluttered world, in collision with world (Contact Only), %u out of %u states in collision", states_in_collision, 50);
  CONSOLE_BRIDGE_logInform("Description, Checks Per Second, Total Num Checks, Num Contacts");
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::FIRST, false, false);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::ALL, false, false);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::FIRST, false, false);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::ALL, false, false);

  for (auto& contact_checker : contact_checkers)
  {
    const bool is_physx { contact_checker->getName() == "PhysxDiscreteManager"};
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, false, is_physx);
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, false, is_physx);
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, false, is_physx);
  }

  CONSOLE_BRIDGE_logInform("Starting benchmark: Robot in cluttered world, in collision with world, %u out of %u states in collision", states_in_collision, 50);
  CONSOLE_BRIDGE_logInform("Description, Checks Per Second, Total Num Checks, Num Contacts");
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::FIRST, false, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::ALL, false, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::FIRST, false, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::ALL, false, true);

  for (auto& contact_checker : contact_checkers)
  {
    const bool is_physx { contact_checker->getName() == "PhysxDiscreteManager"};
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, true, is_physx);
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, true, is_physx);
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, true, is_physx);
  }

  CONSOLE_BRIDGE_logInform("Starting benchmark: Robot in cluttered world, in collision with world (Distance Enabled, 0.2m), %u out of %u states in collision", states_in_collision, t_sampled_states.size());
  CONSOLE_BRIDGE_logInform("Description, Checks Per Second, Total Num Checks, Num Contacts");

  for (auto& contact_checker : contact_checkers)
    contact_checker->setDefaultCollisionMargin(0.2);

  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::FIRST, true, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::ALL, true, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::FIRST, true, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::ALL, true, true);
  for (auto& contact_checker : contact_checkers)
  {
    const bool is_physx { contact_checker->getName() == "PhysxDiscreteManager"};
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, true, true, is_physx);
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, true, true, is_physx);
    runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, true, true, is_physx);
  }

  return 0;
}
