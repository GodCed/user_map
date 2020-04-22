#include "../include/user_map/map_view.hpp"

#include <iostream>
#include <QMouseEvent>
#include <QGraphicsPixmapItem>

namespace user_map
{

MapView::MapView(QWidget *parent): QGraphicsView(parent)
{
  selection_rect.setPen(QPen(Qt::red, 5, Qt::DashLine));

  setScene(&scene);
}

void MapView::mousePressEvent(QMouseEvent* event)
{
  if(event->button() == Qt::LeftButton && occupancy_grid_ptr != nullptr) {
    is_dragging = true;

    QPointF pointer_pos_in_scene = mapToScene(event->pos());
    drag_start = occupancy_grid_ptr->mapFromScene(pointer_pos_in_scene);

    selection_rect.setRect(QRect(drag_start.toPoint(), drag_start.toPoint()));
    scene.addItem(&selection_rect);
  }
}

void MapView::mouseMoveEvent(QMouseEvent *event)
{
  if(!is_dragging) {
    return;
  }

  QPointF pointer_pos_in_scene = mapToScene(event->pos());
  QPointF drag_now = occupancy_grid_ptr->mapFromScene(pointer_pos_in_scene);

  QRect drag_rect = rectFromTwoPoints(drag_start.toPoint(), drag_now.toPoint());
  selection_rect.setRect(drag_rect);
}

void MapView::mouseReleaseEvent(QMouseEvent *event)
{
  if(event->button() == Qt::LeftButton && is_dragging) {
    is_dragging = false;
    scene.removeItem(&selection_rect);

    QPointF pointer_pos_in_scene = mapToScene(event->pos());
    QPointF drag_end = occupancy_grid_ptr->mapFromScene(pointer_pos_in_scene);

    QRect zone_rect = rectFromTwoPoints(drag_start.toPoint(), drag_end.toPoint());
    newZoneFromRect(zone_rect);
  }
}

void MapView::fitPixmap()
{
  fitInView(scene.itemsBoundingRect(), Qt::AspectRatioMode::KeepAspectRatio);
}

void MapView::updateOccupancyGrid(QImage grid)
{
  if(occupancy_grid_ptr != nullptr) {
    scene.removeItem(static_cast<QGraphicsItem*>(occupancy_grid_ptr));
    delete occupancy_grid_ptr;
    occupancy_grid_ptr = nullptr;
  }

  occupancy_grid_ptr = scene.addPixmap(QPixmap::fromImage(grid));
  occupancy_grid_ptr->setZValue(-1);
  fitPixmap();
}

void MapView::newZoneFromRect(QRect rect)
{
  zone_rect_ptrs.push_back(scene.addRect(rect));
}

QRect MapView::rectFromTwoPoints(QPoint a, QPoint b)
{
  int left = static_cast<int>(a.x() < b.x() ? a.x() : b.x());
  int top = static_cast<int>(a.y() < b.y() ? a.y() : b.y());

  int right = static_cast<int>(a.x() > b.x() ? a.x() : b.x());
  int bottom = static_cast<int>(a.y() > b.y() ? a.y() : b.y());

  return QRect(QPoint(left, top), QPoint(right, bottom));
}

}
