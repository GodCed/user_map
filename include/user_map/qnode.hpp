#ifndef user_map_QNODE_HPP_
#define user_map_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string>
#include <QThread>
#include <QImage>
#include <QStringListModel>
#include <QVector>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <user_map/zone.hpp>

namespace user_map
{
  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char** argv );

    void run();
    void quit();
    void loadMap();

  Q_SIGNALS:
    void rosShutdown();
    void newZones(QVector<UserZone>);
    void mapImageUpdated(QImage image);

  public Q_SLOTS:
    void addZone(UserZone zone);
    void clearZones();
    void getZones();

  private:
    void updateMap(const grid_map_msgs::GridMapConstPtr &msg);

    std::shared_ptr<ros::NodeHandle> nh_;

    ros::Subscriber map_subscriber_;
    ros::ServiceClient srv_client_add_zones_;
    ros::ServiceClient srv_client_clear_zones_;
    ros::ServiceClient srv_client_get_zones;

    grid_map::GridMap map_;
  };

}  // namespace user_map

#endif /* user_map_QNODE_HPP_ */
