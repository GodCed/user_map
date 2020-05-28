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

  private Q_SLOTS:
    void on_actionAbout_triggered();
    void on_button_add_orientation_clicked();
    void on_actionLoad_Zones_triggered();
    void on_actionSave_Zones_triggered();
    void on_dial_orientation_valueChanged(int value);
    void on_spin_box_orientation_valueChanged(int arg1);
    void on_button_add_keep_out_clicked();
    void on_button_add_preferred_clicked();

  protected:
    Ui::MainWindowDesign ui;

    QNode qnode;
    QMap<QString, OrientationMode> orientation_mode_map;

  };

}  // namespace user_map

#endif // user_map_MAIN_WINDOW_HPP
