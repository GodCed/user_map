#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QGraphicsView>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsRectItem>

#include "zone.hpp"

namespace user_map {

class MapView : public QGraphicsView
{
Q_OBJECT

  public:
    MapView(QWidget* parent=nullptr);
    void fitPixmap();

    void saveZonesToFile(QDataStream& filestream);
    void loadZonesFromFile(QDataStream& filestream);

  private:
    QGraphicsScene scene;

    QGraphicsPixmapItem* occupancy_grid_ptr = nullptr;

    QGraphicsRectItem selection_rect;
    std::vector<QGraphicsRectItem*> zone_rect_ptrs;
    std::vector<QGraphicsSimpleTextItem*> zone_label_ptrs;

    bool is_dragging = false;
    bool is_adding = false;
    QPointF drag_start;
    Zone new_zone;

    QVector<Zone> zones;

    QRect rectFromTwoPoints(QPoint a, QPoint b);
    void drawZone(Zone zone);

    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

  public Q_SLOTS:
    void updateOccupancyGrid(QImage grid);
    void addZone(Zone zone);
    void clearZones();

  Q_SIGNALS:
    void newZone(Zone);
    void clearedZones();
};

}

#endif // MAP_VIEW_H
