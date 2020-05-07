#ifndef TOPICS_AND_LAYERS_HPP
#define TOPICS_AND_LAYERS_HPP

namespace user_map
{
  constexpr char USER_MAP_NODE_NAME[] = "user_map";
  constexpr char USER_MAP_GUI_NODE_NAME[] = "user_map_gui";

  constexpr char LAYER_OCCUPANCY[] = "occupancy";
  constexpr char LAYER_ORIENTATION[] = "orientation";

  constexpr char TOPIC_OCC_GRID[] = "map";
  constexpr char TOPIC_USER_MAP[] = "user_map";

  constexpr char SERVICE_ADD_ZONES[] = "add_zones";
  constexpr char SERVICE_GET_ZONES[] = "get_zones";
  constexpr char SERVICE_CLEAR_ZONES[] = "clear_zones";

  constexpr char FILENAME_ZONES_DEFAULT[] = "user_zones.usr";
  constexpr char PARAM_FILENAME_ZONES[] = "file_zones";
}

#endif // TOPICS_AND_LAYERS_HPP
