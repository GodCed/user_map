#ifndef QZONE_HPP
#define QZONE_HPP


#include <QDataStream>
#include <QGraphicsScene>
#include <user_map/orientation_mode.hpp>
#include <user_map/Zone.h>
#include <user_map/zone_type.hpp>


namespace user_map {


  class QZone
  {
    public:

      QZone(QZoneType type): type_{type} {}
      virtual ~QZone();

      virtual void draw(QGraphicsScene &scene) = 0;
      virtual void erase(QGraphicsScene &scene) = 0;
      virtual void setSelected(bool selected) = 0;

      inline QZoneType getType() { return type_; }
      inline QGraphicsRectItem* getQGraphicsRectItem() const { return rect_ptr_; }

      inline void setQRect(QRect rect) { rect_ = rect; }

      virtual void toStream(QDataStream &stream) const = 0;
      static std::unique_ptr<QZone> fromStream(QDataStream &stream);

      virtual user_map::Zone toZoneMsg() = 0;
      static std::unique_ptr<QZone> fromZoneMsg(Zone zone);

    private:
      long index_;
      QZoneType type_;

    protected:
      QRect rect_;
      QGraphicsRectItem* rect_ptr_ = nullptr;
  };


  QDataStream& operator << (QDataStream& stream, const std::shared_ptr<QZone>& zone);
  QDataStream& operator >> (QDataStream& stream, std::shared_ptr<QZone>& zone);


}


#endif // QZONE_HPP
