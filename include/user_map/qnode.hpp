/**
 * @file /include/user_map/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef user_map_QNODE_HPP_
#define user_map_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QImage>
#include <QStringListModel>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>

#include "zone.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace user_map {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
  bool init(const std::string &map_topic,
            const std::string &occupancy_grid_service);

  bool init(const std::string &master_url,
            const std::string &host_url,
            const std::string &map_topic,
            const std::string &occupancy_grid_service);

	void run();
  void loadMap();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void mapImageUpdated(QImage image);

public Q_SLOTS:
  void addZone(Zone zone);
  void clearZones();

private:
  void updateMap(const nav_msgs::OccupancyGridConstPtr &msg);
  void publishMap();

	int init_argc;
	char** init_argv;
  std::string occupancy_grid_service;
	ros::Publisher chatter_publisher;
  ros::Publisher map_publisher;
  ros::Subscriber map_subscriber;
  QStringListModel logging_model;
  grid_map::GridMap map;
};

}  // namespace user_map

#endif /* user_map_QNODE_HPP_ */
