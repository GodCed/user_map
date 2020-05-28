#include <QGraphicsSimpleTextItem>
#include <user_map/qcirculation_zone.hpp>


namespace user_map {


  Zone QCirculationZone::toZoneMsg()
  {
    Zone msg;

    msg.type = circulation;
    msg.top = rect_.top();
    msg.left = rect_.left();
    msg.width = rect_.width();
    msg.height = rect_.height();
    msg.value = Circulation::valueFromMode(mode_);

    return msg;
  }


  std::unique_ptr<QCirculationZone> QCirculationZone::fromZoneMsg(Zone zone_msg)
  {
    Circulation::Mode mode = Circulation::modeFromValue(zone_msg.value);
    std::unique_ptr<QCirculationZone> zone_ptr = std::make_unique<QCirculationZone>(mode);

    zone_ptr->rect_ = QRect(
          zone_msg.left,
          zone_msg.top,
          zone_msg.width,
          zone_msg.height);

    return zone_ptr;
  }


  void QCirculationZone::draw(QGraphicsScene &scene)
  {
    if (rect_ptr_ != nullptr)
    {
      erase(scene);
    }

    switch (mode_) {
      case Circulation::Mode::preferred:
        rect_ptr_ = scene.addRect(
            rect_,
            QPen(QColor(0, 255, 0, 100), 5, Qt::SolidLine),
            QBrush(QColor(0, 255, 0, 50))
        );
        break;
      case Circulation::Mode::keep_out:
        rect_ptr_ = scene.addRect(
            rect_,
            QPen(QColor(255, 0, 0, 100), 5, Qt::SolidLine),
            QBrush(QColor(255, 0, 0, 50))
        );
        break;
    }
  }


  void QCirculationZone::erase(QGraphicsScene &scene)
  {
    if (rect_ptr_ == nullptr)
    {
      return;
    }

    scene.removeItem(rect_ptr_);
    delete rect_ptr_;
    rect_ptr_ = nullptr;
  }


  void QCirculationZone::setSelected(bool selected)
  {
    if (selected)
    {
      rect_ptr_->setPen(QPen(QColor(0, 128, 128, 100), 5, Qt::DashLine));
      rect_ptr_->setBrush(QBrush(QColor(0, 128, 128, 50)));
    }
    else
    {
      switch (mode_) {
        case Circulation::Mode::preferred:
          rect_ptr_->setPen(QPen(QColor(0, 255, 0, 100), 5, Qt::SolidLine));
          rect_ptr_->setBrush(QBrush(QColor(0, 255, 0, 50)));
          break;
        case Circulation::Mode::keep_out:
        rect_ptr_->setPen(QPen(QColor(255, 0, 0, 100), 5, Qt::SolidLine));
        rect_ptr_->setBrush(QBrush(QColor(255, 0, 0, 50)));
          break;
      }
    }
  }


  void QCirculationZone::toStream(QDataStream &stream) const
  {
    stream << QZoneType::orientation;
    stream  << Circulation::valueFromMode(mode_) << rect_;
  }


  std::unique_ptr<QCirculationZone> QCirculationZone::fromStream(QDataStream &stream)
  {
    int mode_int;
    stream >> mode_int;
    Circulation::Mode mode = Circulation::modeFromValue(mode_int);

    std::unique_ptr<QCirculationZone> zone_ptr = std::make_unique<QCirculationZone>(mode);
    stream >> zone_ptr->rect_;

    return zone_ptr;
  }


}
