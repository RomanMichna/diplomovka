/** 
* @file SimRobot/SimRobot.h
* Declaration of an interface to the SimRobot GUI
* @author Colin Graf
*/

#pragma once

class QString;
template <typename T> class QVector;
class QIcon;
class QMenu;
class QSettings;
class QPainter;
class QWidget;

namespace SimRobot
{
  class Widget
  {
  public:
    virtual ~Widget() {}
    virtual QWidget* getWidget() = 0;
    virtual void update() {};
    virtual bool canClose() {return true;}
    virtual QMenu* createFileMenu() {return 0;}
    virtual QMenu* createUserMenu() {return 0;}
    virtual QMenu* createEditMenu() {return 0;}
    virtual void paint(QPainter& painter) {}
  };

  /** 
  * An object that can be added to the scene graph
  */
  class Object
  {
  public:
    virtual ~Object() {};
    virtual Widget* createWidget() {return 0;}

    /** Accesses pathname to the object in the scene graph
    * @return The pathname
    */
    virtual const QString& getFullName() const = 0;

    virtual const QIcon* getIcon() const {return 0;};
    virtual int getKind() const {return 0;}
  };

  /** 
  * An object that will be displayed in the status bar
  */
  class StatusLabel
  {
  public:
    virtual ~StatusLabel() {};
    virtual QWidget* getWidget() = 0;
    virtual void update() {};
  };

 /**
  * Flags that can be used for registering modules and objects
  */
  class Flag
  {
  public:

    // flags for registerObject
    static const int hidden = 0x0001; /**< The object will not be listed in the scene graph */
    static const int verticalTitleBar = 0x0002; /**< The object's dock widget has a vertical title bar */
    static const int windowless = 0x0004; /**< The object does not have a widget but will be listed in the scene graph */
    static const int copy = 0x0008; /**< The object's widget has a "copy" entry in its edit menu that can be used to copy a screenshot of the widget to the clipboard */
    static const int exportAsSvg = 0x0010; /**< The object's widget  has an "Export as Svg" entry in its edit menu that can be used to create a svg using the \c paint method of the widget */

    // flags for registerModule
    static const int ignoreReset = 0x1000; /**< The module keeps beeing loaded on scene resets */
  };

  /** 
  * An interface to the SimRobot module
  */
  class Module
  {
  public:

    /** Virtual destructor */
    virtual ~Module() {};

    /**
    * Called to initialize the module. In this phase the module can do the following tasks
    *   - registering its own objects to the scene graph (using \c Application::registerObject) 
    *   - adding status labels to the GUI (using \c Application::addStatusLabel)
    *   - suggest or load further modules (using \c Application::registerModule, \c Application::loadModule)
    * @return Whether an error occurred while initializing the module or not
    */
    virtual bool compile() {return true;}

    /**
    * Called after all modules have been compiled. In this phase the module can update references to scene graph objects of other modules. (using \c resolveObject)
    */
    virtual void link() {}

    /**
    * Called to perform another simulation step
    */
    virtual void update() {};

    /**
    * A handler that will be called when any modules uses \c Application::selectObject
    */
    virtual void selectedObject(const Object& object) {};

    /**
    * A handler that can be used to implement CTRL + SHIFT shortcuts
    */
    virtual void pressedKey(int key, bool pressed) {};
  };

  /** 
  * An interface to the SimRobot GUI
  */
  class Application
  {
  public:
    virtual ~Application() {};
    virtual bool registerObject(const Module& module, Object& object, const Object* parent, int flags = 0) = 0;
    virtual bool unregisterObject(const Object& object) = 0;
    virtual Object* resolveObject(const QString& fullName, int kind = 0) = 0;
    virtual Object* resolveObject(const QVector<QString>& parts, const Object* parent = 0, int kind = 0) = 0;
    virtual int getObjectChildCount(const Object& object) = 0;
    virtual Object* getObjectChild(const Object& object, int index) = 0;
    virtual bool addStatusLabel(const Module& module, StatusLabel* statusLabel) = 0;
    virtual bool registerModule(const Module& module, const QString& displayName, const QString& name, int flags = 0) = 0;
    virtual bool loadModule(const QString& name) = 0;
    virtual bool openObject(const Object& object) = 0;
    virtual bool closeObject(const Object& object) = 0;
    virtual bool selectObject(const Object& object) = 0;
    virtual void showWarning(const QString& title, const QString& message) = 0;
    virtual void setStatusMessage(const QString& message) = 0;
    virtual const QString& getFilePath() const = 0;
    virtual const QString& getAppPath() const = 0;
    virtual QSettings& getSettings() = 0;
    virtual QSettings& getLayoutSettings() = 0;
  };
};

#ifdef WIN32
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif
