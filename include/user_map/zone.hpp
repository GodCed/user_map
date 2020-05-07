#ifndef ZONE_HPP
#define ZONE_HPP

#include <QRect>
#include <QDataStream>
#include <user_map/orientation_mode.hpp>
#include <user_map/Zone.h>

namespace user_map {

  typedef struct UserZone
  {
    QRect rect;
    OrientationMode mode;
    int angle;

    UserZone() { }

    UserZone(const Zone& zone)
    {
      rect = QRect(zone.left, zone.top, zone.width, zone.height);
      mode = OrientationMode_from_value(zone.value);
      angle = zone.value % 1000;
    }
  } UserZone;

  inline QDataStream& operator << (QDataStream& stream, const UserZone& zone)
  {
    stream << zone.rect << zone.mode << zone.angle;
    return stream;
  }

  inline QDataStream& operator >> (QDataStream& stream, UserZone& zone)
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
