#include <QGraphicsSimpleTextItem>
#include <user_map/qzone.hpp>
#include <user_map/qorientation_zone.hpp>
#include <user_map/qcirculation_zone.hpp>


namespace user_map {


  QDataStream& operator >> (QDataStream& stream, std::shared_ptr<QZone>& zone)
  {
    zone = QZone::fromStream(stream);
    return stream;
  }


  QDataStream& operator << (QDataStream& stream, const std::shared_ptr<QZone>& zone)
  {
    zone->toStream(stream);
    return stream;
  }


  QZone::~QZone() { }


  std::unique_ptr<QZone> QZone::fromStream(QDataStream &stream)
  {
    int type_int;
    stream >> type_int;
    QZoneType type = static_cast<QZoneType>(type_int);

    switch (type) {
      case orientation: return QOrientationZone::fromStream(stream);
      case circulation: return QCirculationZone::fromStream(stream);
      }
  }


  std::unique_ptr<QZone> QZone::fromZoneMsg(Zone zone)
  {
    switch (zone.type) {
      case orientation: return QOrientationZone::fromZoneMsg(zone);
      case circulation: return QCirculationZone::fromZoneMsg(zone);
    }
  }


}
