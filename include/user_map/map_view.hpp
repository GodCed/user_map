#ifndef MAP_VIEW_H
#define MAP_VIEW_H


#include <QGraphicsView>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsRectItem>
#include <user_map/qzone.hpp>


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
      std::shared_ptr<QZone> selected_zone_ptr_;

      std::vector<QGraphicsSimpleTextItem*> zone_label_ptrs_;

      bool is_dragging_ = false;
      bool is_adding_ = false;
      QPointF drag_start_;

      std::shared_ptr<QZone> new_zone_;
      QVector<std::shared_ptr<QZone>> zones_;

      QRect rectFromTwoPoints(QPoint a, QPoint b);

      void mousePressEvent(QMouseEvent* event);
      void mouseReleaseEvent(QMouseEvent* event);
      void mouseMoveEvent(QMouseEvent* event);

    public Q_SLOTS:
      void updateOccupancyGrid(QImage grid);
      void addZone(std::shared_ptr<QZone> zone);
      void deleteZone();
      void cancelZone();
      void addZones(QVector<std::shared_ptr<QZone>> zones);
      void clearZones();

    Q_SIGNALS:
      void newZone(std::shared_ptr<QZone>);
      void deletedZone(long);
      void clearedZones();
  };


}

#endif // MAP_VIEW_H
