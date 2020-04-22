#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QGraphicsView>
#include <QGraphicsRectItem>

namespace user_map {

class MapView : public QGraphicsView
{
Q_OBJECT

public:
  MapView(QWidget* parent=nullptr);
  void fitPixmap();

protected:

  QGraphicsScene scene;

  QGraphicsPixmapItem* occupancy_grid_ptr = nullptr;

  QGraphicsRectItem selection_rect;
  std::vector<QGraphicsRectItem*> zone_rect_ptrs;

  bool is_dragging = false;
  QPointF drag_start;

  QRect rectFromTwoPoints(QPoint a, QPoint b);
  void newZoneFromRect(QRect rect);

  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);

public Q_SLOTS:
  void updateOccupancyGrid(QImage grid);
};

}

#endif // MAP_VIEW_H
