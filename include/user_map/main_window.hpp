#ifndef user_map_MAIN_WINDOW_HPP
#define user_map_MAIN_WINDOW_HPP

#include <QtGui/QMainWindow>
#include "map_view.hpp"
#include "ui_main_window.h"
#include "qnode.hpp"

namespace user_map
{
  class MainWindow : public QMainWindow
  {
    Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent *event); // Overloaded function

  public Q_SLOTS:
    void on_actionAbout_triggered();
    void on_button_add_zone_clicked();
    void on_actionLoad_Zones_triggered();
    void on_actionSave_Zones_triggered();

  protected:
    Ui::MainWindowDesign ui;

    QNode qnode;
    QMap<QString, OrientationMode> orientation_mode_map;
  };

}  // namespace user_map

#endif // user_map_MAIN_WINDOW_HPP
