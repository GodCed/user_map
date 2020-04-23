/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <nav_msgs/GetMap.h>
#include <string>
#include <std_msgs/String.h>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include "../include/user_map/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace user_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init(const std::string &map_topic,
                 const std::string &occupancy_grid_service)
{
	ros::init(init_argc,init_argv,"user_map");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  map_publisher = n.advertise<grid_map_msgs::GridMap>("user_map", 1, true);
  map_subscriber = n.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, &QNode::updateMap, this);
  this->occupancy_grid_service = occupancy_grid_service;

	start();
	return true;
}

bool QNode::init(const std::string &master_url,
                 const std::string &host_url,
                 const std::string &map_topic,
                 const std::string &occupancy_grid_service)
{
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"user_map");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  map_publisher = n.advertise<grid_map_msgs::GridMap>("user_map", 1, true);
  map_subscriber = n.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, &QNode::updateMap, this);
  this->occupancy_grid_service = occupancy_grid_service;

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::updateMap(const nav_msgs::OccupancyGridConstPtr &msg) {
  log(Info, "Received map update.");

  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "occupancy", map);

  auto layers = map.getLayers();
  if (std::find(layers.begin(), layers.end(), std::string("orientation")) == layers.end()) {
    map.add("orientation", 0);
  }

  publishMap();

  cv::Mat map_cvimage;
  QImage map_qimage;

  grid_map::GridMapCvConverter::toImage<unsigned char, 3>(map, "occupancy", CV_8UC3, map_cvimage);

  cv::Mat element = cv::getStructuringElement(
        MorphShapes_c::CV_SHAPE_RECT,
        cv::Size(2*5+1, 2*5+1),
        cv::Point(5,5));
  cv::dilate(map_cvimage, map_cvimage, element);
  cv::bitwise_not(map_cvimage,map_cvimage);

  map_qimage = QImage(map_cvimage.data, map_cvimage.cols, map_cvimage.rows, static_cast<int>(map_cvimage.step), QImage::Format_RGB888);
  Q_EMIT mapImageUpdated(map_qimage);
}

void QNode::publishMap()
{
  grid_map_msgs::GridMap map_msg;
  grid_map::GridMapRosConverter::toMessage(map, map_msg);
  map_publisher.publish(map_msg);
}

void QNode::loadMap() {
  nav_msgs::GetMap map_srv;

  if (ros::service::call(occupancy_grid_service, map_srv)) {
    updateMap(boost::make_shared<nav_msgs::OccupancyGrid>(map_srv.response.map));
  }
}

void QNode::addZone(Zone zone)
{
  int orientation_value = 1000*zone.mode + zone.angle;

  grid_map::Index submap_start_index(zone.rect.top(), zone.rect.left());
  grid_map::Index submap_buffer_size(zone.rect.height(), zone.rect.width());

  for (grid_map::SubmapIterator i(map, submap_start_index, submap_buffer_size); !i.isPastEnd(); ++i) {
    map.at("orientation", *i) = orientation_value;
  }

  publishMap();
}

void QNode::clearZones()
{
  auto layers = map.getLayers();
  if (std::find(layers.begin(), layers.end(), std::string("orientation")) != layers.end()) {
      map["orientation"].setZero();
  }
  publishMap();
}

}  // namespace user_map
