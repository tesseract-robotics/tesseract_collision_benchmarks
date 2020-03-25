#include <tesseract_collision_physx/tesseract_physx.h>
#include <tesseract_collision_physx/types.h>

using namespace tesseract_collision;

int main(int, const char*const*)
{
  TesseractPhysx phy;

  std::string link_name = "static_link";
  physx::PxTransform global_tf;
  global_tf.p = physx::PxVec3(0, 0, 0);
  global_tf.q = physx::PxQuat(0, 0, 1, 0);

  physx::PxRigidStatic* sphere = physx::PxCreateStatic(*phy.getPhysics(), global_tf, physx::PxSphereGeometry(physx::PxReal(0.5)), *phy.getMaterial());
  sphere->setName(link_name.c_str());
  std::printf("%s\n", sphere->getName());
  physx::PxFilterData filter_data;
  filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::STATIC);
  filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
  phy.setupFiltering(sphere, filter_data);
  phy.getScene()->addActor(*sphere);

  physx::PxTransform kin_global_tf, kin_shape_tf;
  kin_global_tf.p = physx::PxVec3(0, 0, 0.7f);
  kin_global_tf.q = physx::PxQuat(0, 0, 1, 0);
  kin_shape_tf.p = physx::PxVec3(0, 0, 0);
  kin_shape_tf.q = physx::PxQuat(0, 0, 1, 0);
  physx::PxRigidDynamic* kinematic_sphere = physx::PxCreateKinematic(*phy.getPhysics(),
                                                                     kin_global_tf,
                                                                     physx::PxSphereGeometry(physx::PxReal(0.25)),
                                                                     *phy.getMaterial(),
                                                                     physx::PxReal(0.1),
                                                                     kin_shape_tf);
  kinematic_sphere->setName("kinematic_sphere");

  filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
  filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::STATIC);
  phy.setupFiltering(kinematic_sphere, filter_data);
  phy.getScene()->addActor(*kinematic_sphere);

  for (int i = 0; i < 5; ++i)
  {
    std::printf("%d!\n", i);
    phy.getScene()->simulate(0.0001f);
    phy.getScene()->fetchResults(true);

    physx::PxTransform update_tf;
    update_tf.p = physx::PxVec3(0, 0, 0.7f - ((physx::PxReal(i) + 1.f) * 0.1f));
    update_tf.q = physx::PxQuat(0, 0, 1, 0);
    kinematic_sphere->setKinematicTarget(update_tf);
  }
  std::printf("Done!\n");
  return 0;
}
