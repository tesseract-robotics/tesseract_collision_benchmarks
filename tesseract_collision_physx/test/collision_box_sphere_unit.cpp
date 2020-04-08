#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

//#include <tesseract_collision/test_suite/collision_box_sphere_unit.hpp>
#include <tesseract_collision_physx/physx_discrete_manager.h>

namespace tesseract_collision
{
namespace test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  CollisionShapePtr box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  CollisionShapePtr thin_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces), 0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);
  }

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj3_shapes, obj3_poses);
}

inline void runTestPrimitive(DiscreteContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "sphere_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.55, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.251);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.25, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

inline void runTestConvex(DiscreteContactManager& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "sphere_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.53776, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.03776, 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.27);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.26224, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.76224, 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  if (use_convex_mesh)
    detail::runTestConvex(checker);
  else
    detail::runTestPrimitive(checker);
}

}  // namespace test_suite
}  // namespace tesseract_collision

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionBoxSphereUnit)  // NOLINT
{
  tesseract_collision::PhysxDiscreteManager checker;
  tesseract_collision::test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionBoxSphereConvexHullUnit)  // NOLINT
{
  tesseract_collision::PhysxDiscreteManager checker;
  tesseract_collision::test_suite::runTest(checker, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
