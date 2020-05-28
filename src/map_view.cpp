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
        auto clicked_zone = std::find_if(
              zones_.begin(),
              zones_.end(),
              [clicked_item](const std::shared_ptr<QZone>& zone) -> bool {
                return clicked_item == zone->getQGraphicsRectItem();
              }
        );

        if (selected_zone_ptr_) {
          if (*clicked_zone == selected_zone_ptr_) {
            clicked_zone = zones_.end();
          }
          selected_zone_ptr_->setSelected(false);
          selected_zone_ptr_ = nullptr;
        }

        if(clicked_zone != zones_.end()) {
          (*clicked_zone)->setSelected(true);
          selected_zone_ptr_ = *clicked_zone;
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
      new_zone_->setQRect(zone_rect);

      zones_.append(new_zone_);
      new_zone_->draw(scene_);
      Q_EMIT newZone(new_zone_);
    }
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

  void MapView::addZone(std::shared_ptr<QZone> zone)
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
    if(!selected_zone_ptr_) {
      return;
    }

    auto selected_it = std::find_if(
          zones_.begin(),
          zones_.end(),
          [this](std::shared_ptr<QZone> zone_ptr) -> bool {
            return zone_ptr == this->selected_zone_ptr_;
          }
    );

    if (selected_it != zones_.end()) {
      long index = selected_it - zones_.begin();

      Q_EMIT deletedZone(index);
      selected_zone_ptr_->erase(scene_);
      zones_.erase(selected_it);
      selected_zone_ptr_ = nullptr;
    }
  }

  void MapView::addZones(QVector<std::shared_ptr<QZone>> zones)
  {
    clearZones();
    for(auto &zone: zones)
    {
      zone->draw(scene_);
      zones_.append(zone);
    }
  }

  void MapView::clearZones()
  {
    for (auto &zone: zones_) {
      zone->erase(scene_);
    }
    zones_.clear();
  }

  void MapView::loadZonesFromFile(QDataStream& filestream)
  {
    clearZones();
    Q_EMIT clearedZones();

    filestream >> zones_;
    for(int index=0; index < zones_.size(); index++) {
      zones_[index]->draw(scene_);
      Q_EMIT newZone(zones_[index]);
    }
  }

  void MapView::saveZonesToFile(QDataStream& filestream)
  {
    filestream << zones_;
  }

}
