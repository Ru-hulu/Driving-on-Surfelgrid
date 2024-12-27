#ifndef HELPER
#define HELPER

#include <cmath>
#include <algorithm>
#include "constants.h"
namespace HybridAStar {

namespace Helper {


/**
 * 将朝向t的值归一化到[0, 360)，Degree
 */
static inline float normalizeHeading(float t) 
{
  if ((int)t <= 0 || (int)t >= 360) {
    if (t < -0.1) {
      t += 360.f;
    } else if ((int)t >= 360) {
      t -= 360.f;
    } else {
      t =  0;
    }
  }

  return t;
}


static inline float normalizeHeadingRad(float t) 
{
  t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
  return t;
}


static inline float toDeg(float t) 
{
  return normalizeHeadingRad(t) * 180.f / M_PI ;
}


static inline float toRad(float t) {
  return normalizeHeadingRad(t / 180.f * M_PI);
}

/*!
   \fn float clamp(float n, float lower, float upper)
   \brief Clamps a number between a lower and an upper bound
   \param t heading in rad
*/
static inline float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

}
}

#endif // HELPER

