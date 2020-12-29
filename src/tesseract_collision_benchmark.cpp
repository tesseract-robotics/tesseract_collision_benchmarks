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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <tesseract_collision_benchmark/utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_collision_physx/physx_discrete_manager.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

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
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
  }
  else
  {
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
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

  ros::WallTime start = ros::WallTime::now();
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
  double duration = (ros::WallTime::now() - start).toSec();

  double checks_per_second = static_cast<double>(trials * states.size()) / duration;
  std::size_t total_num_checks = trials * states.size();
  std::size_t contact_count = (res.collision == true && res.contact_count == 0) ? 1 : res.contact_count;
  ROS_INFO("%s, %lf, %ld, %ld", desc.c_str(), checks_per_second, total_num_checks, contact_count);

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

  ros::WallTime start = ros::WallTime::now();
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

  double duration = (ros::WallTime::now() - start).toSec();

  double checks_per_second = static_cast<double>(trials * states.size()) / duration;
  std::size_t total_num_checks = trials * states.size();

  std::size_t contact_count = 0;
  for (const auto& c : res)
    contact_count += c.second.size();

  ROS_INFO("%s, %lf, %ld, %ld", desc.c_str(), checks_per_second, total_num_checks, contact_count);
}

int main(int argc, char** argv)
{
  moveit::core::RobotModelPtr robot_model;
  ros::init(argc, argv, "tesseract_collision_benchmark");
  ros::NodeHandle node_handle;

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  unsigned int trials = 1000;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration sleep_t(2.5);

  // ************************************************
  // SETUP MOVEIT ENVIRONMENT
  // ************************************************
  robot_model = moveit::core::loadTestingRobotModel("panda");

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  std::vector<std::string> link_names = robot_model->getLinkModelNames();
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(link_names, true) };
  planning_scene->checkCollision(req, res, planning_scene->getCurrentState(), acm);

  // ************************************************
  // SETUP TESSERACT ENVIRONMENT
  // ************************************************
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  auto urdf = locator->locateResource("package://moveit_resources_panda_description/urdf/panda.urdf");
//  auto scene_graph = std::make_shared<tesseract_scene_graph::SceneGraph>();
//  tesseract_urdf::parseURDFFile(scene_graph, urdf->getFilePath(), locator);
  tesseract_environment::Environment tesseract_env;
  tesseract_env.init<tesseract_environment::OFKTStateSolver>(boost::filesystem::path(urdf->getFilePath()), locator);
  tesseract_env.registerDiscreteContactManager("BulletDiscreteBVHManager", &tesseract_collision::tesseract_collision_bullet::BulletDiscreteBVHManager::create);
  tesseract_env.registerDiscreteContactManager("BulletDiscreteSimpleManager", &tesseract_collision::tesseract_collision_bullet::BulletDiscreteSimpleManager::create);
  tesseract_env.registerDiscreteContactManager("FCLDiscreteBVHManager", &tesseract_collision::tesseract_collision_fcl::FCLDiscreteBVHManager::create);
  tesseract_env.registerDiscreteContactManager("PhysxDiscreteManager", &tesseract_collision::PhysxDiscreteManager::create);

  // Exclude robot collisions
  for (std::size_t i = 0; i < link_names.size() - 1; ++i)
    for (std::size_t j = i + 1; j < link_names.size(); ++j)
      tesseract_env.addAllowedCollision(link_names[i], link_names[j], "exclude robot links");

  tesseract_environment::StateSolver::Ptr tesseract_state_solver = tesseract_env.getStateSolver();

  auto bullet_bvh_checker = tesseract_env.getDiscreteContactManager("BulletDiscreteBVHManager");
  auto bullet_simple_checker = tesseract_env.getDiscreteContactManager("BulletDiscreteSimpleManager");
  auto fcl_bvh_checker = tesseract_env.getDiscreteContactManager("FCLDiscreteBVHManager");
  auto physx_bvh_checker = tesseract_env.getDiscreteContactManager("PhysxDiscreteManager");

  std::vector<tesseract_geometry::Geometry::ConstPtr> shapes;
  tesseract_common::VectorIsometry3d shape_poses;
  clutterWorld(shapes, shape_poses, planning_scene, bullet_bvh_checker->clone(), tesseract_state_solver->clone(), 50, tesseract_collision::CollisionObjectType::CONVEX_MESH);

//  for (std::size_t i = 0; i < shapes.size(); ++i)
//  {
//    bullet_bvh_checker->addCollisionObject("world" + std::to_string(i), 0, {shapes[i]}, {shape_poses[i]});
//    bullet_simple_checker->addCollisionObject("world" + std::to_string(i), 0, {shapes[i]}, {shape_poses[i]});
//    fcl_bvh_checker->addCollisionObject("world" + std::to_string(i), 0, {shapes[i]}, {shape_poses[i]});
//  }

  bullet_bvh_checker->addCollisionObject("world", 0, shapes, shape_poses);
  bullet_simple_checker->addCollisionObject("world", 0, shapes, shape_poses);
  fcl_bvh_checker->addCollisionObject("world", 0, shapes, shape_poses);
  physx_bvh_checker->addCollisionObject("world", 0, shapes, shape_poses);

  bullet_bvh_checker->setDefaultCollisionMarginData(0);
  bullet_simple_checker->setDefaultCollisionMarginData(0);
  fcl_bvh_checker->setDefaultCollisionMarginData(0);
  physx_bvh_checker->setDefaultCollisionMarginData(0);

  bullet_bvh_checker->setActiveCollisionObjects(link_names);
  bullet_simple_checker->setActiveCollisionObjects(link_names);
  fcl_bvh_checker->setActiveCollisionObjects(link_names);
  physx_bvh_checker->setActiveCollisionObjects(link_names);

  ROS_INFO("Starting...");

  ros::Duration(0.5).sleep();

  moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  current_state.update();

  std::vector<moveit::core::RobotState> sampled_states;
  std::vector<tesseract_common::TransformMap> t_sampled_states;
  int states_in_collision = findStates(sampled_states, tesseract_collision::RobotStateSelector::IN_COLLISION , 50, planning_scene,bullet_bvh_checker->clone(), tesseract_state_solver->clone());

  t_sampled_states.clear();
  for (auto& s : sampled_states)
  {
    auto t_env_state = tesseract_state_solver->getState(current_state.getVariableNames(), Eigen::Map<Eigen::VectorXd>(s.getVariablePositions(), static_cast<long>(s.getVariableNames().size())));
    t_env_state->link_transforms.erase("world");
    t_sampled_states.push_back(t_env_state->link_transforms);
  }

  bullet_bvh_checker->setDefaultCollisionMarginData(0);
  bullet_simple_checker->setDefaultCollisionMarginData(0);
  fcl_bvh_checker->setDefaultCollisionMarginData(0);
  physx_bvh_checker->setDefaultCollisionMarginData(0);
  ROS_INFO("Starting benchmark: Robot in cluttered world, in collision with world (Contact Only), %u out of %u states in collision", states_in_collision, 50);
  ROS_INFO("Description, Checks Per Second, Total Num Checks, Num Contacts");
//  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::FIRST, false, false);
//  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::BULLET, tesseract_collision::ContactTestType::ALL, false, false);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::FIRST, false, false);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::ALL, false, false);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, false);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, false);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, false);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, false);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, false);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, false);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, false);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, false);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, false);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, false, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, false, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, false, true);

  ROS_INFO("Starting benchmark: Robot in cluttered world, in collision with world, %u out of %u states in collision", states_in_collision, 50);
  ROS_INFO("Description, Checks Per Second, Total Num Checks, Num Contacts");
//    runCollisionDetection(trials, planning_scene, sampled_states_2, CollisionDetector::BULLET, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::FIRST, false, true);
  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::ALL, false, true);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, true);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, true);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, true);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, true);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, true);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, true);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, true);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, true);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, false, true, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, false, true, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, false, true, true);

  ROS_INFO("Starting benchmark: Robot in cluttered world, in collision with world (Distance Enabled, 0.2m), %u out of %u states in collision", states_in_collision, t_sampled_states.size());
  ROS_INFO("Description, Checks Per Second, Total Num Checks, Num Contacts");
  bullet_bvh_checker->setDefaultCollisionMarginData(0.2);
  bullet_simple_checker->setDefaultCollisionMarginData(0.2);
  fcl_bvh_checker->setDefaultCollisionMarginData(0.2);
  physx_bvh_checker->setDefaultCollisionMarginData(0.2);
//  runCollisionDetection(trials, planning_scene, sampled_states, CollisionDetector::BULLET, true, true);
//  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::FIRST, true, true);
//  runCollisionDetection(trials, planning_scene, sampled_states, tesseract_collision::CollisionDetector::FCL, tesseract_collision::ContactTestType::ALL, true, true);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, true, true);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, true, true);
  runTesseractCollisionDetection("BulletDiscreteBVHManager", trials, *bullet_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, true, true);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, true, true);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, true, true);
  runTesseractCollisionDetection("BulletDiscreteSimpleManager", trials, *bullet_simple_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, true, true);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, true, true);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, true, true);
  runTesseractCollisionDetection("FCLDiscreteBVHManager", trials, *fcl_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, true, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::FIRST, true, true, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::CLOSEST, true, true, true);
  runTesseractCollisionDetection("PhysxDiscreteManager", trials, *physx_bvh_checker, t_sampled_states, tesseract_collision::ContactTestType::ALL, true, true, true);

  bool visualize;
  node_handle.getParam("/compare_collision_checking_speed/visualization", visualize);
  if (visualize)
  {
    // publishes the planning scene to visualize in rviz if possible
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_diff_publisher.publish(planning_scene_msg);
  }

  return 0;
}
