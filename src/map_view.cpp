#include <iostream>
#include <QMouseEvent>
#include <QGraphicsPixmapItem>
#include <user_map/map_view.hpp>

namespace user_map
{

  MapView::MapView(QWidget *parent): QGraphicsView(parent)
  {
    selection_rect_.setPen(QPen(QColor(255, 0, 128, 200), 5, Qt::DashLine));
    setScene(&scene_);
  }

  void MapView::mousePressEvent(QMouseEvent* event)
  {
    if(event->button() == Qt::LeftButton && occupancy_grid_ptr_ != nullptr) {
      if(is_adding_) {
        is_dragging_ = true;
        is_adding_ = false;
        setCursor(Qt::CursorShape::DragMoveCursor);

        QPointF pointer_pos_in_scene = mapToScene(event->pos());
        drag_start_ = occupancy_grid_ptr_->mapFromScene(pointer_pos_in_scene);

        selection_rect_.setRect(QRect(drag_start_.toPoint(), drag_start_.toPoint()));
        scene_.addItem(&selection_rect_);
      }
      else {
        QGraphicsRectItem *clicked_item = reinterpret_cast<QGraphicsRectItem*>(itemAt(event->pos()));
        if(std::find(zone_rect_ptrs_.begin(), zone_rect_ptrs_.end(), clicked_item) != zone_rect_ptrs_.end()) {
          selectZone(clicked_item);
        }
        else {
          clearSelection();
        }
      }
    }
  }

  void MapView::mouseMoveEvent(QMouseEvent *event)
  {
    if(!is_dragging_) {
      return;
    }

    QPointF pointer_pos_in_scene = mapToScene(event->pos());
    QPointF drag_now = occupancy_grid_ptr_->mapFromScene(pointer_pos_in_scene);

    QRect drag_rect = rectFromTwoPoints(drag_start_.toPoint(), drag_now.toPoint());
    selection_rect_.setRect(drag_rect);
  }

  void MapView::mouseReleaseEvent(QMouseEvent *event)
  {
    if(event->button() == Qt::LeftButton && is_dragging_) {
      is_dragging_ = false;
      setCursor(Qt::CursorShape::ArrowCursor);
      scene_.removeItem(&selection_rect_);

      QPointF pointer_pos_in_scene = mapToScene(event->pos());
      QPointF drag_end = occupancy_grid_ptr_->mapFromScene(pointer_pos_in_scene);

      QRect zone_rect = rectFromTwoPoints(drag_start_.toPoint(), drag_end.toPoint());
      new_zone_.rect = zone_rect;

      zones_.push_back(new_zone_);
      drawZone(new_zone_);
      Q_EMIT newZone(new_zone_);
    }
  }

  void MapView::drawZone(UserZone zone)
  {
    QString label_string_prefix;
    switch (zone.mode) {
      case OrientationMode::fixed: label_string_prefix = "F "; break;
      case OrientationMode::tangent: label_string_prefix = "T "; break;
      case OrientationMode::parallel: label_string_prefix = "P "; break;
      default: break;
    }

    QString raw_label_string("%2 deg");
    QString label_string = raw_label_string.arg(zone.angle);

    QGraphicsSimpleTextItem* label_ptr = scene_.addSimpleText(label_string_prefix + label_string);
    label_ptr->setPos(zone.rect.topLeft() + QPoint(5,5));
    label_ptr->setBrush(QBrush(QColor(0,0,255,100)));

    QFont label_font = label_ptr->font();
    label_font.setBold(true);
    label_ptr->setFont(label_font);

    zone_label_ptrs_.push_back(label_ptr);

    zone_rect_ptrs_.push_back(
          scene_.addRect(
            zone.rect,
            QPen(QColor(0, 0, 255, 100), 5, Qt::SolidLine),
            QBrush(QColor(0, 0, 255, 50))));
  }

  void MapView::selectZone(QGraphicsRectItem *zone_rect_ptr)
  {
    clearSelection();

    zone_rect_ptr->setPen(QPen(QColor(0, 128, 128, 100), 5, Qt::DashLine));
    zone_rect_ptr->setBrush(QBrush(QColor(0, 128, 128, 50)));

    selected_rect_ptr_ = zone_rect_ptr;
  }

  void MapView::clearSelection()
  {
    if(selected_rect_ptr_ == nullptr) {
      return;
    }

    selected_rect_ptr_->setPen(QPen(QColor(0, 0, 255, 100), 5, Qt::SolidLine));
    selected_rect_ptr_->setBrush(QBrush(QColor(0, 0, 255, 50)));

    selected_rect_ptr_ = nullptr;
  }

  void MapView::fitScene()
  {
    fitInView(scene_.itemsBoundingRect(), Qt::AspectRatioMode::KeepAspectRatio);
  }

  void MapView::updateOccupancyGrid(QImage grid)
  {
    if(occupancy_grid_ptr_ != nullptr) {
      scene_.removeItem(static_cast<QGraphicsItem*>(occupancy_grid_ptr_));
      delete occupancy_grid_ptr_;
      occupancy_grid_ptr_ = nullptr;
    }

    occupancy_grid_ptr_ = scene_.addPixmap(QPixmap::fromImage(grid));
    occupancy_grid_ptr_->setZValue(-1);
    fitScene();
  }

  QRect MapView::rectFromTwoPoints(QPoint a, QPoint b)
  {
    int left = static_cast<int>(a.x() < b.x() ? a.x() : b.x());
    int top = static_cast<int>(a.y() < b.y() ? a.y() : b.y());

    int right = static_cast<int>(a.x() > b.x() ? a.x() : b.x());
    int bottom = static_cast<int>(a.y() > b.y() ? a.y() : b.y());

    return QRect(QPoint(left, top), QPoint(right, bottom));
  }

  void MapView::addZone(UserZone zone)
  {
    new_zone_ = zone;
    is_adding_ = true;
    setCursor(Qt::CursorShape::CrossCursor);
  }

  void MapView::cancelZone()
  {
    is_adding_ = false;
    setCursor(Qt::CursorShape::ArrowCursor);
  }

  void MapView::deleteZone()
  {
    if(selected_rect_ptr_ == nullptr) {
      return;
    }

    auto rect_it = std::find(zone_rect_ptrs_.begin(), zone_rect_ptrs_.end(), selected_rect_ptr_);
    long index = rect_it - zone_rect_ptrs_.begin();
    Q_EMIT deletedZone(index);

    zone_rect_ptrs_.erase(rect_it);
    scene_.removeItem(selected_rect_ptr_);
    delete selected_rect_ptr_;
    selected_rect_ptr_ = nullptr;

    auto label_it = zone_label_ptrs_.begin() + index;
    scene_.removeItem(*label_it);
    delete *label_it;
    zone_label_ptrs_.erase(label_it);

    auto zone_it = zones_.begin() + index;
    zones_.erase(zone_it);
  }

  void MapView::addZones(QVector<UserZone> zones)
  {
    clearZones();
    for(UserZone &zone: zones)
    {
      drawZone(zone);
      zones_.push_back(zone);
    }
  }

  void MapView::clearZones()
  {
    for(auto &zone: zone_rect_ptrs_) {
      scene_.removeItem(zone);
      delete zone;
    }
    zone_rect_ptrs_.clear();

    for(auto &label: zone_label_ptrs_) {
      scene_.removeItem(label);
      delete label;
    }
    zone_label_ptrs_.clear();

    zones_.clear();
  }

  void MapView::loadZonesFromFile(QDataStream& filestream)
  {
    clearZones();
    Q_EMIT clearedZones();

    filestream >> zones_;
    for(UserZone zone: zones_) {
      drawZone(zone);
      Q_EMIT newZone(zone);
    }
  }

  void MapView::saveZonesToFile(QDataStream& filestream)
  {
    filestream << zones_;
  }

}
