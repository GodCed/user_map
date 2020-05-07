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
    void fitScene();

    void saveZonesToFile(QDataStream& filestream);
    void loadZonesFromFile(QDataStream& filestream);

  private:
    QGraphicsScene scene_;

    QGraphicsPixmapItem* occupancy_grid_ptr_ = nullptr;

    QGraphicsRectItem selection_rect_;
    std::vector<QGraphicsRectItem*> zone_rect_ptrs_;
    std::vector<QGraphicsSimpleTextItem*> zone_label_ptrs_;

    bool is_dragging_ = false;
    bool is_adding_ = false;
    QPointF drag_start_;
    UserZone new_zone_;

    QVector<UserZone> zones_;

    QRect rectFromTwoPoints(QPoint a, QPoint b);
    void drawZone(UserZone zone);

    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

  public Q_SLOTS:
    void updateOccupancyGrid(QImage grid);
    void addZone(UserZone zone);
    void addZones(QVector<UserZone> zones);
    void clearZones();

  Q_SIGNALS:
    void newZone(UserZone);
    void clearedZones();
};

}

#endif // MAP_VIEW_H
