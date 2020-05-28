#ifndef QORIENTATION_ZONE_HPP
#define QORIENTATION_ZONE_HPP


#include <QDataStream>
#include <QGraphicsScene>
#include <user_map/orientation_mode.hpp>
#include <user_map/Zone.h>
#include <user_map/qzone.hpp>


namespace user_map {


  class QOrientationZone: public QZone
  {
    public:

      QOrientationZone(): QZone(orientation) {}

      void draw(QGraphicsScene &scene);
      void erase(QGraphicsScene &scene);
      void setSelected(bool selected);

      void toStream(QDataStream &stream) const;
      static std::unique_ptr<QOrientationZone> fromStream(QDataStream &stream);

      user_map::Zone toZoneMsg();
      static std::unique_ptr<QOrientationZone> fromZoneMsg(Zone zone);

      inline void setAngle(int angle) { angle_ = angle; }
      inline void setMode(OrientationMode mode) { mode_ = mode; }

    private:

      QGraphicsSimpleTextItem* label_ptr_ = nullptr;

      OrientationMode mode_;
      int angle_;
  };


}


#endif // QORIENTATION_ZONE_HPP
