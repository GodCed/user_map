#include <QGraphicsSimpleTextItem>
#include <user_map/qorientation_zone.hpp>


namespace user_map {


  Zone QOrientationZone::toZoneMsg()
  {
    Zone msg;

    msg.type = orientation;
    msg.top = rect_.top();
    msg.left = rect_.left();
    msg.width = rect_.width();
    msg.height = rect_.height();
    msg.value = value_from_OrientationMode(mode_, angle_);

    return msg;
  }


  std::unique_ptr<QOrientationZone> QOrientationZone::fromZoneMsg(Zone zone)
  {
    std::unique_ptr<QOrientationZone> zone_ptr = std::make_unique<QOrientationZone>();

    zone_ptr->rect_ = QRect(zone.left, zone.top, zone.width, zone.height);
    zone_ptr->mode_ = OrientationMode_from_value(zone.value);
    zone_ptr->angle_ = zone.value % 1000;

    return zone_ptr;
  }


  void QOrientationZone::draw(QGraphicsScene &scene)
  {
    if (rect_ptr_ != nullptr)
    {
      erase(scene);
    }

    QString label_string_prefix;
    switch (mode_) {
      case OrientationMode::fixed: label_string_prefix = "F "; break;
      case OrientationMode::tangent: label_string_prefix = "T "; break;
      case OrientationMode::parallel: label_string_prefix = "P "; break;
      default: break;
    }

    QString raw_label_string("%2 deg");
    QString label_string = raw_label_string.arg(angle_);

    label_ptr_ = scene.addSimpleText(label_string_prefix + label_string);
    label_ptr_->setPos(rect_.topLeft() + QPoint(5,5));
    label_ptr_->setBrush(QBrush(QColor(0,0,255,100)));

    QFont label_font = label_ptr_->font();
    label_font.setBold(true);
    label_ptr_->setFont(label_font);


    rect_ptr_ = scene.addRect(
          rect_,
          QPen(QColor(0, 0, 255, 100), 5, Qt::SolidLine),
          QBrush(QColor(0, 0, 255, 50)));
  }


  void QOrientationZone::erase(QGraphicsScene &scene)
  {
    if (rect_ptr_ == nullptr)
    {
      return;
    }

    scene.removeItem(rect_ptr_);
    delete rect_ptr_;
    rect_ptr_ = nullptr;

    scene.removeItem(label_ptr_);
    delete label_ptr_;
    label_ptr_ = nullptr;
  }


  void QOrientationZone::setSelected(bool selected)
  {
    if (selected)
    {
      rect_ptr_->setPen(QPen(QColor(0, 128, 128, 100), 5, Qt::DashLine));
      rect_ptr_->setBrush(QBrush(QColor(0, 128, 128, 50)));
    }
    else
    {
      rect_ptr_->setPen(QPen(QColor(0, 0, 255, 100), 5, Qt::SolidLine));
      rect_ptr_->setBrush(QBrush(QColor(0, 0, 255, 50)));
    }
  }


  void QOrientationZone::toStream(QDataStream &stream) const
  {
    stream << QZoneType::orientation;
    stream << rect_ << mode_ << angle_;
  }


  std::unique_ptr<QOrientationZone> QOrientationZone::fromStream(QDataStream &stream)
  {
    std::unique_ptr<QOrientationZone> zone_ptr = std::make_unique<QOrientationZone>();

    stream >> zone_ptr->rect_;

    int mode_int;
    stream >> mode_int;
    zone_ptr->mode_ = static_cast<OrientationMode>(mode_int);

    stream >> zone_ptr->angle_;

    return zone_ptr;
  }


}
