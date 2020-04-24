#include "../include/user_map/map_view.hpp"

#include <iostream>
#include <QMouseEvent>
#include <QGraphicsPixmapItem>

namespace user_map
{

MapView::MapView(QWidget *parent): QGraphicsView(parent)
{
  selection_rect.setPen(QPen(QColor(255, 0, 128, 200), 5, Qt::DashLine));
  setScene(&scene);
}

void MapView::mousePressEvent(QMouseEvent* event)
{
  if(event->button() == Qt::LeftButton && occupancy_grid_ptr != nullptr && is_adding) {
    is_dragging = true;
    is_adding = false;
    setCursor(Qt::CursorShape::DragMoveCursor);

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
    setCursor(Qt::CursorShape::ArrowCursor);
    scene.removeItem(&selection_rect);

    QPointF pointer_pos_in_scene = mapToScene(event->pos());
    QPointF drag_end = occupancy_grid_ptr->mapFromScene(pointer_pos_in_scene);

    QRect zone_rect = rectFromTwoPoints(drag_start.toPoint(), drag_end.toPoint());
    new_zone.rect = zone_rect;

    zones.push_back(new_zone);
    drawZone(new_zone);
    Q_EMIT newZone(new_zone);
  }
}

void MapView::drawZone(Zone zone)
{
  zone_rect_ptrs.push_back(
        scene.addRect(
          zone.rect,
          QPen(QColor(0, 0, 255, 100), 5, Qt::SolidLine),
          QBrush(QColor(0, 0, 255, 50))));

  QString label_string_prefix;
  switch (zone.mode) {
    case OrientationMode::fixed: label_string_prefix = "F "; break;
    case OrientationMode::tangent: label_string_prefix = "T "; break;
    case OrientationMode::parallel: label_string_prefix = "P "; break;
    default: break;
  }

  QString raw_label_string("%2 deg");
  QString label_string = raw_label_string.arg(zone.angle);

  QGraphicsSimpleTextItem* label_ptr = scene.addSimpleText(label_string_prefix + label_string);
  label_ptr->setPos(zone.rect.topLeft() + QPoint(5,5));
  label_ptr->setBrush(QBrush(QColor(0,0,255,100)));

  QFont label_font = label_ptr->font();
  label_font.setBold(true);
  label_ptr->setFont(label_font);

  zone_label_ptrs.push_back(label_ptr);
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

QRect MapView::rectFromTwoPoints(QPoint a, QPoint b)
{
  int left = static_cast<int>(a.x() < b.x() ? a.x() : b.x());
  int top = static_cast<int>(a.y() < b.y() ? a.y() : b.y());

  int right = static_cast<int>(a.x() > b.x() ? a.x() : b.x());
  int bottom = static_cast<int>(a.y() > b.y() ? a.y() : b.y());

  return QRect(QPoint(left, top), QPoint(right, bottom));
}

void MapView::addZone(Zone zone)
{
  new_zone = zone;
  is_adding = true;
  setCursor(Qt::CursorShape::CrossCursor);
}

void MapView::clearZones()
{
  for(auto &zone: zone_rect_ptrs) {
    scene.removeItem(zone);
    delete zone;
  }
  zone_rect_ptrs.clear();

  for(auto &label: zone_label_ptrs) {
    scene.removeItem(label);
    delete label;
  }
  zone_label_ptrs.clear();

  zones.clear();
}

void MapView::loadZonesFromFile(QDataStream& filestream)
{
  clearZones();
  Q_EMIT clearedZones();

  filestream >> zones;
  for(Zone zone: zones) {
    drawZone(zone);
    Q_EMIT newZone(zone);
  }
}

void MapView::saveZonesToFile(QDataStream& filestream)
{
  filestream << zones;
}

}
