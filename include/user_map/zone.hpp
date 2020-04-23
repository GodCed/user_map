#ifndef ZONE_HPP
#define ZONE_HPP

#include <QRect>
#include "orientation_mode.hpp"

namespace user_map {

  typedef struct
  {
    QRect rect;
    OrientationMode mode;
    int angle;
  } Zone;

}

#endif // ZONE_HPP
