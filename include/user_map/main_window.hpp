/**
 * @file /include/user_map/main_window.hpp
 *
 * @brief Qt based gui for user_map.
 *
 * @date November 2010
 **/
#ifndef user_map_MAIN_WINDOW_H
#define user_map_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "map_view.hpp"
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace user_map {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
  void resizeEvent(QResizeEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
  void on_button_load_map_clicked();
	void on_checkbox_use_environment_stateChanged(int state);
  void on_button_add_zone_clicked();
  void on_actionLoad_Zones_triggered();
  void on_actionSave_Zones_triggered();

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

protected:
	Ui::MainWindowDesign ui;

	QNode qnode;
  QMap<QString, OrientationMode> orientation_mode_map;
};

}  // namespace user_map

#endif // user_map_MAIN_WINDOW_H
