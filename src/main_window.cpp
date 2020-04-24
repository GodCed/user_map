/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <user_map/main_window.hpp>
#include <user_map/map_view.hpp>
#include <user_map/zone.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace user_map {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
  ** Map View
  **********************/

  orientation_mode_map.insert("Tangent", tangent);
  orientation_mode_map.insert("Parallel", parallel);
  orientation_mode_map.insert("Fixed", fixed);

  ui.combo_orientation_mode->insertItems(0, orientation_mode_map.keys());

  QObject::connect(&qnode, SIGNAL(mapImageUpdated(QImage)), ui.map_view, SLOT(updateOccupancyGrid(QImage)));
  QObject::connect(ui.button_clear_zones, SIGNAL(clicked()), ui.map_view, SLOT(clearZones()));
  QObject::connect(ui.button_clear_zones, SIGNAL(clicked()), &qnode, SLOT(clearZones()));
  QObject::connect(ui.map_view, SIGNAL(newZone(Zone)), &qnode, SLOT(addZone(Zone)));
  QObject::connect(ui.map_view, SIGNAL(clearedZones()), &qnode, SLOT(clearZones()));

  /*********************
  ** Auto Start
  **********************/
  if ( ui.checkbox_remember_settings->isChecked() ) {
      on_button_connect_clicked(true);
  }
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
    if ( !qnode.init(
           ui.line_edit_map_topic->text().toStdString(),
           ui.line_edit_occupancy_grid_service->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
    if ( ! qnode.init(
           ui.line_edit_master->text().toStdString(),
           ui.line_edit_host->text().toStdString(),
           ui.line_edit_map_topic->text().toStdString(),
           ui.line_edit_occupancy_grid_service->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::on_button_load_map_clicked()
{
  qnode.loadMap();
}

void MainWindow::on_button_add_zone_clicked()
{
  Zone zone;
  zone.angle = ui.spin_box_orientation->value();
  zone.mode = orientation_mode_map[ui.combo_orientation_mode->currentText()];

  ui.map_view->addZone(zone);
}

void MainWindow::on_actionSave_Zones_triggered()
{
  QFile save_file("zones.user");
  if(!save_file.open(QIODevice::WriteOnly)) {
    QMessageBox::warning(this, "Save Failed", "Couln't open ~/.ros/zones.user for writing");
    return;
  }

  QDataStream save_stream(&save_file);
  ui.map_view->saveZonesToFile(save_stream);

  save_file.close();
  QMessageBox::information(this, "Zones Saved", "Saved zones to ~/.ros/zones.user");
}

void MainWindow::on_actionLoad_Zones_triggered()
{
  QFile load_file("zones.user");
  if(!load_file.open(QIODevice::ReadOnly)) {
    QMessageBox::warning(this, "Load Failed", "Couln't open ~/.ros/zones.user for reading");
  }

  QDataStream load_stream(&load_file);
  ui.map_view->loadZonesFromFile(load_stream);

  load_file.close();
  QMessageBox::information(this, "Zones Loaded", "Loaded zones from ~/.ros/zones.user");
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
}

void MainWindow::resizeEvent(QResizeEvent *event) {
  QMainWindow::resizeEvent(event);
  ui.map_view->fitPixmap();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "user_map");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);

    QString map_topic = settings.value("map_topic", QString("/map")).toString();
    QString occupancy_grid_service = settings.value("occupancy_grid_service", "/get_map").toString();
    ui.line_edit_map_topic->setText(map_topic);
    ui.line_edit_occupancy_grid_service->setText(occupancy_grid_service);

    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "user_map");

    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());

    settings.setValue("map_topic", ui.line_edit_map_topic->text());
    settings.setValue("occupancy_grid_service", ui.line_edit_occupancy_grid_service->text());

    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace user_map
