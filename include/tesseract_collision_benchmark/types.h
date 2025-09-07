#ifndef TESSERACT_COLLISION_BENCHMARK_TYPES_H
#define TESSERACT_COLLISION_BENCHMARK_TYPES_H

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

}

#endif // TESSERACT_COLLISION_BENCHMARK_TYPES_H
