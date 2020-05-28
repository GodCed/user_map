#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <string>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <user_map/qnode.hpp>
#include <user_map/topics_and_layers.hpp>
#include <user_map/AddZones.h>
#include <user_map/RemoveZones.h>
#include <user_map/ClearZones.h>
#include <user_map/GetZones.h>

namespace user_map
{
  QNode::QNode(int argc, char** argv )
  {
    ros::init(argc, argv, "user_map_gui");
    nh_ = std::make_shared<ros::NodeHandle>();
    map_subscriber_ = nh_->subscribe<grid_map_msgs::GridMap>(TOPIC_USER_MAP, 10, &QNode::updateMap, this);

    ros::service::waitForService(SERVICE_GET_ZONES);
    srv_client_get_zones = nh_->serviceClient<user_map::GetZones>(SERVICE_GET_ZONES, true);

    ros::service::waitForService(SERVICE_ADD_ZONES);
    srv_client_add_zones_ = nh_->serviceClient<user_map::AddZones>(SERVICE_ADD_ZONES, true);

    ros::service::waitForService(SERVICE_REMOVE_ZONES);
    srv_client_remove_zones = nh_->serviceClient<user_map::RemoveZones>(SERVICE_REMOVE_ZONES, true);

    ros::service::waitForService(SERVICE_CLEAR_ZONES);
    srv_client_clear_zones_ = nh_->serviceClient<user_map::ClearZones>(SERVICE_CLEAR_ZONES, true);
  }

  void QNode::run()
  {
    ros::spin();
    Q_EMIT rosShutdown();
  }

  void QNode::quit()
  {
    ros::shutdown();
    ros::waitForShutdown();
  }

  void QNode::updateMap(const grid_map_msgs::GridMapConstPtr &msg)
  {
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);

    cv::Mat map_cvimage;
    QImage map_qimage;

    grid_map::GridMapCvConverter::toImage<unsigned char, 3>(map_, LAYER_OCCUPANCY, CV_8UC3, map_cvimage);

    cv::Mat element = cv::getStructuringElement(
          MorphShapes_c::CV_SHAPE_RECT,
          cv::Size(2*5+1, 2*5+1),
          cv::Point(5,5));
    cv::dilate(map_cvimage, map_cvimage, element);
    cv::bitwise_not(map_cvimage,map_cvimage);

    map_qimage = QImage(map_cvimage.data, map_cvimage.cols, map_cvimage.rows, static_cast<int>(map_cvimage.step), QImage::Format_RGB888);
    Q_EMIT mapImageUpdated(map_qimage);
  }

  void QNode::addZone(std::shared_ptr<QZone> user_zone)
  {
    user_map::Zone zone = user_zone->toZoneMsg();

    grid_map::Index submap_start_index(zone.top, zone.left);
    grid_map::Index submap_buffer_size(zone.height, zone.width);

    for (grid_map::SubmapIterator i(map_, submap_start_index, submap_buffer_size); !i.isPastEnd(); ++i) {
      map_.at("orientation", *i) = zone.value;
    }

    user_map::AddZones srv;
    srv.request.zones.push_back(zone);
    srv_client_add_zones_.call(srv);
  }

  void QNode::removeZone(long index)
  {
    user_map::RemoveZones srv;
    srv.request.indexes.push_back(index);
    srv_client_remove_zones.call(srv);
  }

  void QNode::clearZones()
  {
    auto layers = map_.getLayers();
    if (std::find(layers.begin(), layers.end(), std::string("orientation")) != layers.end()) {
        map_["orientation"].setZero();
    }

    user_map::ClearZones srv;
    srv_client_clear_zones_.call(srv);
  }

  void QNode::getZones()
  {
    user_map::GetZones srv;
    if (srv_client_get_zones.call(srv))
    {
      QVector<std::shared_ptr<QZone>> user_zones;
      std::shared_ptr<QZone> user_zone;
      for (auto &zone: srv.response.zones) {
        user_zone = QZone::fromZoneMsg(zone);
        if (user_zone) {
          user_zones.append(user_zone);
        }
      }
      Q_EMIT newZones(user_zones);
    }
  }
}  // namespace user_map
