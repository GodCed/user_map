#ifndef ZONE_HPP
#define ZONE_HPP

#include <QRect>

namespace user_map {

enum OrientationMode
{
  none,
  tangent,
  parallel,
  fixed
};

typedef struct
{
  QRect rect;
  OrientationMode mode;
  int angle;
} Zone;

}

#endif // ZONE_HPP
