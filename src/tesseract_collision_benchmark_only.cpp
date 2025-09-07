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
// #include <tesseract_collision_physx/physx_discrete_manager.h>

#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands/modify_allowed_collisions_command.h>

#include <tesseract_collision_benchmark/types.h>

#include <random_numbers/random_numbers.h>
#include <console_bridge/console.h>

namespace tesseract_collision
{

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *  \param num_objects The number of objects to be cluttered
 *  \param CollisionObjectType Type of object to clutter (mesh or box)
 */
void clutterWorld(std::vector<tesseract_geometry::Geometry::ConstPtr>& shapes,
                  tesseract_common::VectorIsometry3d& shape_poses,
                  const DiscreteContactManager::Ptr &contact_checker,
                  const tesseract_scene_graph::StateSolver::Ptr &state_solver,
                  const std::size_t num_objects,
                  CollisionObjectType type)
{
    CONSOLE_BRIDGE_logInform("Cluttering scene...");

    random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(123);

    auto t_env_state = state_solver->getState();
    contact_checker->setCollisionObjectsTransform(t_env_state.link_transforms);

    // load panda link5 as world collision object
    std::string name;
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
            name = "mesh";
            tesseract_common::GeneralResourceLocator locator;
            auto resource = locator.locateResource(kinect);
            Eigen::Vector3d scale = Eigen::Vector3d::Constant(num_generator.uniformReal(0.3, 1.0));
            auto t_mesh = tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(resource, scale, true, false);
            assert(t_mesh.size() == 1);
            if (type == CollisionObjectType::MESH)
                t_shape = t_mesh[0];
            else
                t_shape = tesseract_collision::makeConvexMesh(*t_mesh[0]);

            break;
        }
        case CollisionObjectType::BOX:
        {
            name = "box";
            const double x = num_generator.uniformReal(0.05, 0.2);
            const double y = num_generator.uniformReal(0.05, 0.2);
            const double z = num_generator.uniformReal(0.05, 0.2);
            t_shape = std::make_shared<tesseract_geometry::Box>(x, y ,z);
            break;
        }
        }

        name.append(std::to_string(i));
        contact_checker->addCollisionObject(name, 0, {t_shape}, {pos});

        tesseract_collision::ContactRequest t_req(tesseract_collision::ContactTestType::FIRST);
        tesseract_collision::ContactResultMap t_res;
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
int findStates(std::vector<tesseract_common::TransformMap>& robot_states,
               RobotStateSelector desired_states,
               unsigned int num_states,
               const DiscreteContactManager::Ptr &contact_checker,
               const tesseract_scene_graph::StateSolver::Ptr &state_solver)
{
    std::size_t i{ 0 };
    int states_in_collision {0};
    while (robot_states.size() < num_states)
    {
        auto t_env_state = state_solver->getRandomState();
        contact_checker->setCollisionObjectsTransform(t_env_state.link_transforms);
        tesseract_collision::ContactRequest t_req(tesseract_collision::ContactTestType::FIRST);
        tesseract_collision::ContactResultMap t_res;
        contact_checker->contactTest(t_res, t_req);

        switch (desired_states)
        {
        case RobotStateSelector::IN_COLLISION:
            if (!t_res.empty())
            {
                robot_states.push_back(t_env_state.link_transforms);
                ++states_in_collision;
            }
            break;
        case RobotStateSelector::NOT_IN_COLLISION:
            if (t_res.empty())
                robot_states.push_back(t_env_state.link_transforms);
            break;
        case RobotStateSelector::RANDOM:
            robot_states.push_back(t_env_state.link_transforms);
            if (!t_res.empty())
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
    unsigned int trials = 1000;

    // ************************************************
    // SETUP TESSERACT ENVIRONMENT
    // ************************************************
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto urdf = locator->locateResource("package://tesseract_collision_benchmarks/data/panda.urdf");
    auto srdf = locator->locateResource("package://tesseract_collision_benchmarks/data/panda.srdf");

    tesseract_environment::Environment tesseract_env;
    tesseract_env.init(std::filesystem::path(urdf->getFilePath()), std::filesystem::path(srdf->getFilePath()), locator);
    std::vector<std::string> link_names = tesseract_env.getActiveLinkNames();

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
    // contact_checkers.push_back(tesseract_env.getDiscreteContactManager("PhysxDiscreteManager"));

    std::vector<tesseract_geometry::Geometry::ConstPtr> shapes;
    tesseract_common::VectorIsometry3d shape_poses;
    clutterWorld(shapes, shape_poses, contact_checkers.front()->clone(), tesseract_state_solver->clone(), 50, tesseract_collision::CollisionObjectType::CONVEX_MESH);

    for (auto& contact_checker : contact_checkers)
    {
        contact_checker->addCollisionObject("world", 0, shapes, shape_poses);
        contact_checker->setDefaultCollisionMargin(0);
        contact_checker->setActiveCollisionObjects(link_names);
    }

    CONSOLE_BRIDGE_logInform("Starting...");

    sleep(1);

    std::vector<tesseract_common::TransformMap> t_sampled_states;
    int states_in_collision = findStates(t_sampled_states, tesseract_collision::RobotStateSelector::IN_COLLISION , 50, contact_checkers.front()->clone(), tesseract_state_solver->clone());

    for (auto& s : t_sampled_states)
      s.erase("world");

    for (auto& contact_checker : contact_checkers)
        contact_checker->setDefaultCollisionMargin(0);

    CONSOLE_BRIDGE_logInform("Starting benchmark: Robot in cluttered world, in collision with world (Contact Only), %u out of %u states in collision", states_in_collision, 50);
    CONSOLE_BRIDGE_logInform("Description, Checks Per Second, Total Num Checks, Num Contacts");

    for (auto& contact_checker : contact_checkers)
    {
        const bool is_physx { contact_checker->getName() == "PhysxDiscreteManager"};
        runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, false, is_physx);
        runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, false, is_physx);
        runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, false, is_physx);
    }

    CONSOLE_BRIDGE_logInform("Starting benchmark: Robot in cluttered world, in collision with world, %u out of %u states in collision", states_in_collision, 50);
    CONSOLE_BRIDGE_logInform("Description, Checks Per Second, Total Num Checks, Num Contacts");

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

    for (auto& contact_checker : contact_checkers)
    {
        const bool is_physx { contact_checker->getName() == "PhysxDiscreteManager"};
        runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, true, true, is_physx);
        runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, true, true, is_physx);
        runTesseractCollisionDetection(contact_checker->getName(), trials, *contact_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, true, true, is_physx);
    }

    return 0;
}
