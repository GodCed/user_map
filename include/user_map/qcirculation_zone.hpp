#ifndef QCIRCULATION_ZONE_HPP
#define QCIRCULATION_ZONE_HPP


#include <QDataStream>
#include <QGraphicsScene>
#include <user_map/circulation_mode.hpp>
#include <user_map/Zone.h>
#include <user_map/qzone.hpp>


namespace user_map {


  class QCirculationZone: public QZone
  {
    public:

      QCirculationZone(Circulation::Mode mode): QZone(circulation), mode_{mode} {}

      void draw(QGraphicsScene &scene);
      void erase(QGraphicsScene &scene);
      void setSelected(bool selected);

      void toStream(QDataStream &stream) const;
      static std::unique_ptr<QCirculationZone> fromStream(QDataStream &stream);

      user_map::Zone toZoneMsg();
      static std::unique_ptr<QCirculationZone> fromZoneMsg(Zone zone_msg);

    private:

      Circulation::Mode mode_;
  };


}


#endif // QCIRCULATION_ZONE_HPP
