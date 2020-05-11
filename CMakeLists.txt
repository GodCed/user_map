##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(user_map)

##############################################################################
# Dependencies
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    eigen_conversions
    grid_map_ros
    message_generation
)

find_package(OpenCV REQUIRED
    core
    imgproc
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

##############################################################################
# Catkin
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

# Use this to define what the package will export (e.g. libs, headers).
catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        qt_build
        roscpp
        eigen_conversions
        grid_map_ros
        message_runtime
    DEPENDS
        OpenCV
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
# Add the appropriate components to the component list here
rosbuild_prepare_qt4(
    QtCore
    QtGui
)

##############################################################################
# GUI Sections
##############################################################################

set(QT_FORMS
    ui/main_window.ui
)

set(QT_RESOURCES
    resources/images.qrc
)

set(QT_MOC
    include/user_map/main_window.hpp
    include/user_map/map_view.hpp
    include/user_map/orientation_mode.hpp
    include/user_map/qnode.hpp
)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

# Node
set(NODE_SOURCES
    src/user_map_node.cpp
)

# GUI
set(QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/map_view.cpp
    src/qnode.cpp
)

##############################################################################
# Binaries
##############################################################################

# Node
add_executable(user_map_node ${NODE_SOURCES})
target_link_libraries(user_map_node ${catkin_LIBRARIES})

# GUI
add_executable(user_map_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(user_map_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##############################################################################
# Install
##############################################################################

# GUI
install(
    TARGETS user_map_node user_map_gui
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Headers
install(
    FILES
        include/user_map/orientation_mode.hpp
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
