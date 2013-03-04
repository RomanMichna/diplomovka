/** 
* @file SimRobot/Main.cpp
* Implementation of the main function of SimRobot
* @author Colin Graf
*/

#include <QApplication>

#ifdef _WIN32
#include "qtdotnetstyle.h"
#endif
#include "MainWindow.h"

#ifdef MACOSX
#include "MacStyle.h"
#include <QFileOpenEvent>

class SimRobotApp : public QApplication
{
public:
  SimRobotApp(int &argc, char **argv)
  : QApplication(argc, argv) {}
  
  MainWindow* mainWindow;
  
protected:
  bool event(QEvent *ev)
  {
    if(ev->type() == QEvent::FileOpen)
    {
      mainWindow->openFile(static_cast<QFileOpenEvent*>(ev)->file());
      return true;
    }
    else
      return QApplication::event(ev);
  }
};

#define QApplication SimRobotApp
#endif

int main(int argc, char *argv[])
{
#ifdef _WIN32
  QApplication::setStyle(new QtDotNetStyle(QtDotNetStyle::Standard));
#endif
  QApplication app(argc, argv);
  MainWindow mainWindow(argc, argv);
#ifdef MACOSX
  app.mainWindow = &mainWindow;
  app.setStyle(new MacStyle());
  MacStyle::addFullscreen(&mainWindow);
#endif
  app.setApplicationName("SimRobot");

  // open file from commandline
  for(int i = 1; i < argc; i++)
    if(*argv[i] != '-')
    {
#ifdef MACOSX
      if(strcmp(argv[i], "YES"))
      {
        mainWindow.setWindowOpacity(0);
        mainWindow.show();
        mainWindow.openFile(argv[i]);
        mainWindow.setWindowOpacity(1);
      }
#else
      mainWindow.openFile(argv[i]);
#endif
      break;
    }

  mainWindow.show();
  int result = app.exec();
#ifdef LINUX
  exit(result); // fixes a mysterious segfault
#endif
  return result;
}
