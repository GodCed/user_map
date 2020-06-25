##############################################################################
# CMake configuration
##############################################################################


cmake_minimum_required(VERSION 2.8.3)
project(user_map)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)


##############################################################################
# Dependencies and include paths
##############################################################################


find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_conversions
    grid_map_ros
    message_generation
)

find_package(OpenCV REQUIRED
    core
    imgproc
)

find_package(Qt5 COMPONENTS
    Widgets
    REQUIRED
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


##############################################################################
# Catkin package
##############################################################################


add_message_files(
    FILES
        Zone.msg
)

add_service_files(
    FILES
        AddZones.srv
        RemoveZones.srv
        GetZones.srv
        ClearZones.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        roscpp
        eigen_conversions
        grid_map_ros
        message_runtime
    DEPENDS
        OpenCV
)


##############################################################################
# GUI
##############################################################################


# UI files
set(QT_FORMS
    ui/main_window.ui
)

# Ressources
set(QT_RESOURCES
    resources/images.qrc
)

# Headers of QObject classes (for MOC)
set(QT_HEADERS
    include/user_map/main_window.hpp
    include/user_map/map_view.hpp
    include/user_map/qnode.hpp
)

# Sources
set(QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/map_view.cpp
    src/qnode.cpp
    src/qzone.cpp
    src/qorientation_zone.cpp
    src/qcirculation_zone.cpp
)

# Target
add_executable(user_map_gui ${QT_HEADERS} ${QT_SOURCES} ${QT_RESOURCES} ${QT_FORMS})
target_link_libraries(user_map_gui Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(user_map_gui ${${PROJECT_NAME}_EXPORTED_TARGETS})


##############################################################################
# Node
##############################################################################


# Sources
set(NODE_SOURCES
    src/user_map_node.cpp
)

# Target
add_executable(user_map_node ${NODE_SOURCES})
target_link_libraries(user_map_node ${catkin_LIBRARIES})
add_dependencies(user_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS})


##############################################################################
# Install
##############################################################################


# Binaries
install(
    TARGETS user_map_node user_map_gui
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Headers
install(
    FILES
        include/user_map/orientation_mode.hpp
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

