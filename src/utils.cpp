#include <tesseract_collision_benchmark/utils.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_collision
{
void clutterWorld(std::vector<tesseract_geometry::Geometry::ConstPtr>& shapes,
                  tesseract_common::VectorIsometry3d& shape_poses,
                  const planning_scene::PlanningScenePtr& planning_scene,
                  const DiscreteContactManager::Ptr &contact_checker,
                  const tesseract_environment::StateSolver::Ptr& state_solver,
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
  contact_checker->setCollisionObjectsTransform(t_env_state->link_transforms);

  // load panda link5 as world collision object
  std::string name;
  shapes::ShapeConstPtr shape;
  tesseract_geometry::Geometry::Ptr t_shape;
  std::string kinect = "package://moveit_resources/panda_description/meshes/collision/link5.stl";

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
      ROS_DEBUG_STREAM("Object was in collision, remove");
      planning_scene->getWorldNonConst()->removeObject(name);
    }
    contact_checker->removeCollisionObject(name);
    i++;
  }
  ROS_INFO_STREAM("Cluttered the planning scene with " << added_objects << " objects");
}

int findStates(std::vector<moveit::core::RobotState>& robot_states,
               RobotStateSelector desired_states,
               unsigned int num_states,
               const planning_scene::PlanningScenePtr& scene,
               const DiscreteContactManager::Ptr &contact_checker,
               const tesseract_environment::StateSolver::Ptr& state_solver)
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
    contact_checker->setCollisionObjectsTransform(t_env_state->link_transforms);
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
