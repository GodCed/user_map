#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <user_map/orientation_mode.hpp>
#include <user_map/AddZones.h>
#include <user_map/RemoveZones.h>
#include <user_map/GetZones.h>
#include <user_map/ClearZones.h>
#include <user_map/topics_and_layers.hpp>
#include <user_map/zone_type.hpp>
#include <user_map/circulation_mode.hpp>

namespace user_map
{
  class UserMapNode
  {
  public:
    UserMapNode()
    {
      ros::NodeHandle pnh;
      pnh.param(PARAM_FILENAME_ZONES, filename_zones_, std::string(FILENAME_ZONES_DEFAULT));

      loadZones();

      sub_occ_grid_ = nh_.subscribe(TOPIC_OCC_GRID, 1, &UserMapNode::onOccGrid, this);
      pub_grid_map_ = nh_.advertise<grid_map_msgs::GridMap>(TOPIC_USER_MAP, 1, true);
      pub_occ_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>(TOPIC_USER_OCC_GRID, 1, true);

      srv_add_zones_ = nh_.advertiseService(SERVICE_ADD_ZONES, &UserMapNode::onAddZones, this);
      srv_remove_zones_ = nh_.advertiseService(SERVICE_REMOVE_ZONES, &UserMapNode::onRemoveZones, this);
      srv_get_zones_ = nh_.advertiseService(SERVICE_GET_ZONES, &UserMapNode::onGetZones, this);
      srv_clear_zones_ = nh_.advertiseService(SERVICE_CLEAR_ZONES, &UserMapNode::onClearZones, this);

      requestOccGrid();
    }

    ~UserMapNode()
    {
      saveZones();
    }

  private:

    ros::NodeHandle nh_;

    ros::Subscriber sub_occ_grid_;
    ros::Publisher pub_grid_map_;
    ros::Publisher pub_occ_grid_;

    ros::ServiceServer srv_add_zones_;
    ros::ServiceServer srv_remove_zones_;
    ros::ServiceServer srv_get_zones_;
    ros::ServiceServer srv_clear_zones_;

    grid_map::GridMap grid_map_;
    std::vector<user_map::Zone> zones_;

    std::string filename_zones_;

    void onOccGrid(const nav_msgs::OccupancyGridConstPtr &msg)
    {
      ROS_INFO("Received occupancy grid.");

      grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, LAYER_OCCUPANCY, grid_map_);
      createUserLayers();

      publishGridMap();
    }

    bool onAddZones(user_map::AddZonesRequest& req, user_map::AddZonesResponse&)
    {
      zones_.insert(zones_.end(),
                    std::make_move_iterator(req.zones.begin()),
                    std::make_move_iterator(req.zones.end()));

      if (grid_map_.exists(LAYER_OCCUPANCY))
      {
        createUserLayers();
        publishGridMap();
      }
      return true;
    }

    bool onRemoveZones(user_map::RemoveZonesRequest& req, user_map::RemoveZonesResponse&)
    {
      long offset = 0;
      for(auto index: req.indexes) {
        zones_.erase(zones_.begin() + index + offset);
        offset -= 1;
      }

      if (grid_map_.exists(LAYER_OCCUPANCY))
      {
        createUserLayers();
        publishGridMap();
      }
      return true;
    }

    bool onGetZones(user_map::GetZonesRequest&, user_map::GetZonesResponse& res)
    {
      res.zones = zones_;
      return true;
    }

    bool onClearZones(user_map::ClearZonesRequest&, user_map::ClearZonesResponse&)
    {
      zones_.clear();

      if (grid_map_.exists(LAYER_OCCUPANCY))
      {
        createUserLayers();
        publishGridMap();
      }
      return true;
    }

    void requestOccGrid()
    {
      ROS_INFO("Requesting occupancy grid");
      nav_msgs::GetMap map_srv;

      if (ros::service::call("get_map", map_srv)) {
        onOccGrid(boost::make_shared<nav_msgs::OccupancyGrid>(map_srv.response.map));
      }
    }

    void createUserLayers()
    {
      grid_map_.add(LAYER_ORIENTATION, value_from_OrientationMode(OrientationMode::none, 0));
      grid_map_.add(LAYER_CIRCULATION, 20);

      for (auto& zone: zones_)
      {
         grid_map::Index start_index(zone.top, zone.left);
         grid_map::Index buffer_size(zone.height, zone.width);

        for (grid_map::SubmapIterator i(grid_map_, start_index, buffer_size); !i.isPastEnd(); ++i)
        {
          QZoneType type = static_cast<QZoneType>(zone.type);
          switch (type) {
            case orientation: grid_map_.at(LAYER_ORIENTATION, *i) = zone.value; break;
            case circulation: grid_map_.at(LAYER_CIRCULATION, *i) = zone.value; break;
          }
        }
      }

      grid_map_.add(LAYER_COST, grid_map_[LAYER_OCCUPANCY] + grid_map_[LAYER_CIRCULATION]);
    }

    void publishGridMap() const
    {
      grid_map_msgs::GridMap grid_map_msg;
      grid_map::GridMapRosConverter::toMessage(grid_map_, grid_map_msg);

      nav_msgs::OccupancyGrid occ_grid_msg;
      grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, LAYER_COST, -1, 100, occ_grid_msg);

      pub_grid_map_.publish(grid_map_msg);
      pub_occ_grid_.publish(occ_grid_msg);
    }

    void saveZones()
    {
      std::ofstream file(filename_zones_, std::ios::binary | std::ios::trunc);
      if (file.is_open())
      {
        for (auto& zone: zones_)
        {
          file.write(reinterpret_cast<char*>(&zone), sizeof (user_map::Zone));
        }
        file.close();
      }
      else
      {
        ROS_ERROR("Couln't open file %s for write", filename_zones_.c_str());
      }
    }

    void loadZones()
    {
      std::ifstream file(filename_zones_, std::ios::binary);
      if (file.is_open())
      {
        user_map::Zone zone;
        while (file.peek() != EOF)
        {
          file.read(reinterpret_cast<char*>(&zone), sizeof (user_map::Zone));
          zones_.push_back(zone);
        }
        if (grid_map_.exists(LAYER_OCCUPANCY))
        {
          createUserLayers();
          publishGridMap();
        }
      }
      else
      {
        ROS_INFO("Couln't open file %s for read", filename_zones_.c_str());
      }
    }
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "user_map");

  user_map::UserMapNode node;

  ros::spin();
  return 0;
}
