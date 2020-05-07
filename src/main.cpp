#include <QtGui>
#include <QApplication>
#include <user_map/main_window.hpp>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  user_map::MainWindow w(argc,argv);
  w.show();

  return app.exec();
}
