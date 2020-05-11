#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <user_map/main_window.hpp>
#include <user_map/map_view.hpp>
#include <user_map/user_zone.hpp>

namespace user_map
{
  MainWindow::MainWindow(int argc, char** argv, QWidget *parent): QMainWindow(parent), qnode(argc,argv)
  {
    ui.setupUi(this);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    setWindowIcon(QIcon(":/images/icon.png"));

    orientation_mode_map.insert("Tangent", tangent);
    orientation_mode_map.insert("Parallel", parallel);
    orientation_mode_map.insert("Fixed", fixed);
    ui.combo_orientation_mode->insertItems(0, orientation_mode_map.keys());

    ui.dial_orientation->setValue(180);

    QObject::connect(&qnode, SIGNAL(mapImageUpdated(QImage)), ui.map_view, SLOT(updateOccupancyGrid(QImage)));
    QObject::connect(&qnode, SIGNAL(newZones(QVector<UserZone>)), ui.map_view, SLOT(addZones(QVector<UserZone>)));
    QObject::connect(ui.button_clear_zones, SIGNAL(clicked()), ui.map_view, SLOT(clearZones()));
    QObject::connect(ui.button_clear_zones, SIGNAL(clicked()), &qnode, SLOT(clearZones()));
    QObject::connect(ui.button_cancel_zone, SIGNAL(clicked()), ui.map_view, SLOT(cancelZone()));
    QObject::connect(ui.map_view, SIGNAL(newZone(UserZone)), &qnode, SLOT(addZone(UserZone)));
    QObject::connect(ui.map_view, SIGNAL(clearedZones()), &qnode, SLOT(clearZones()));
    QObject::connect(ui.map_view, SIGNAL(deletedZone(long)), &qnode, SLOT(removeZone(long)));
    QObject::connect(ui.button_remove_zone, SIGNAL(clicked()), ui.map_view, SLOT(deleteZone()));

    qnode.start();
    qnode.getZones();
  }

  MainWindow::~MainWindow()
  {
    qnode.quit();
  }

  void MainWindow::on_button_add_zone_clicked()
  {
    UserZone zone;
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

  void MainWindow::resizeEvent(QResizeEvent *event)
  {
    QMainWindow::resizeEvent(event);
    ui.map_view->fitScene();
  }

  void MainWindow::on_actionAbout_triggered()
  {
      QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
  }

  void MainWindow::on_dial_orientation_valueChanged(int value)
  {
     int offset_value = (value+180) % 360;
     int reversed_value = 360 - offset_value;
     ui.spin_box_orientation->setValue(reversed_value);
  }

  void MainWindow::on_spin_box_orientation_valueChanged(int value)
  {
    int dial_value = (360 - value + 180) % 360;
    ui.dial_orientation->setValue(dial_value);
  }
}  // namespace user_map


