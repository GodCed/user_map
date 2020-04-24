#ifndef ZONE_HPP
#define ZONE_HPP

#include <QRect>
#include <QDataStream>
#include "orientation_mode.hpp"

namespace user_map {

  typedef struct
  {
    QRect rect;
    OrientationMode mode;
    int angle;
  } Zone;

  inline QDataStream& operator << (QDataStream& stream, const Zone& zone)
  {
    stream << zone.rect << zone.mode << zone.angle;
    return stream;
  }

  inline QDataStream& operator >> (QDataStream& stream, Zone& zone)
  {
    stream >> zone.rect;

    int mode_int;
    stream >> mode_int;
    zone.mode = static_cast<OrientationMode>(mode_int);

    stream >> zone.angle;
    return stream;
  }
}

#endif // ZONE_HPP
