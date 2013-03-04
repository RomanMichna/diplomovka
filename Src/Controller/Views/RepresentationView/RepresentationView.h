/*
 * RepresentationView.h
 *
 *  Created on: Mar 22, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#pragma once

#include "SimRobot.h"
#include <QIcon>
#include <QString>
#include <map>
#include <string>
#include <qtvariantproperty.h>
#include "PropertyManager.h"
#include <QObject>
#include "Platform/Thread.h" // for SYNC
#include <Tools/Enum.h>


class RobotConsole;
class InMessage;
class RepresentationWidget;
class QtProperty;
class QtVariantProperty;
class StreamHandler;
class RepresentationWidget;
class QEvent;
class QWidget;
class OutBinaryMessage;
/**
 * A class which can be used to display representations.(Not only representations, anything that uses MODIFY)
 *
 * The view will display any Representation that is given to him via handleMessage.
 * A StreamHandler is used to get the needed specification data.
 *
 */
class RepresentationView : public SimRobot::Object
{

  public:

   /** Creates a new RepresentationView.
    * @param fullName The path to this view in the scene graph
    * @param console The console object. Used to display error messages
    * @param repName name of the representation. This is used to generate get requests
    * @param streamHandler  will be used to get representation specification data while parsing.
    */
    RepresentationView(const QString& fullName,const std::string& repName, RobotConsole& console, StreamHandler& streamHandler);


    virtual SimRobot::Widget* createWidget();
    virtual const QString& getFullName() const {return theFullName;}
    virtual const QIcon* getIcon() const {return &theIcon;}

    const QString& getFullName(){return theFullName;}
    /**
     * Disconnects the current widget from the view.
     */
    void removeWidget();
    /**
    * Parses the specified message and displays its contents.
    * @param msg The message containing the data.
    * @param type The type of the representation contained within the msg.
    * @param repName Name of the representation
    * @return true if the message has been handled. False otherwise.
    */
    bool handleMessage(InMessage& msg,const std::string& type, const std::string& repName);

    /**
     * If set to true, the view will ignore further updates from the RobotConsole.
     */
    void setIgnoreUpdates(bool value);

    /**
     * Sends the current values to the robot.
     */
    void set();

    /**
     * Unset changes
     */
    void setUnchanged();

    /**
     * Enabled/disable auto-set mode.
     * In auto-set mode the view will send data changes to the robot as soon as
     * the user has finished editing a field (the field has lost focus).
     */
    void setAutoSet(bool value);

  protected:

    /**
     * Parses debug messages into a QtProperty-tree.
     *
     * @param msg the msg that should be parsed.
     * @param type the type of the first item in msg.
     * @param fieldName the fieldName of the first item in msg.
     * @param pParent Pointer to the parent Property. Newly created properties will be children of this property. Can be NULL.
     * @param parentFQN Fully qualified name of the parent. (In namespace notation e.g.:this.is.a.fully.qualified.name). Can be empty
     * @return the newly created property or NULL of an error occurred.
     */
    QtProperty* parse(InMessage& msg,const std::string& type, const QString& fieldName,  QtProperty* pParent, const std::string& parentFQN);
    QtProperty* parseArray(InMessage& msg,const std::string& type, const QString& fieldName, QtProperty* pParent, const std::string& parentFQN);
    QtProperty* parseClass(InMessage& msg,const std::string& type, const QString& fieldName, QtProperty* pParent, const std::string& parentFQN);
    QtProperty* parseEnum (InMessage& msg,const std::string& type, const QString& fieldName, QtProperty* pParent, const std::string& parentFQN);
    QtProperty* parseBasicType(InMessage& msg,const std::string& type, const QString& fieldName, QtProperty* pParent, const std::string& parentFQN);

    /**
     * Parses a QtProperty-tree into a debug message.
     * @param pRoot the root node of the tree
     * @param out Queue used to store the debug message.
     * @return true if the parsing was successful
     */
    bool reverseParse(QtProperty* pRoot, OutBinaryMessage& out);
    bool reverseParseClass(QtProperty* pRoot, OutBinaryMessage& out);
    bool reverseParseEnum(QtProperty* pRoot, OutBinaryMessage& out);
    bool reverseParseArray(QtProperty* pRoot, OutBinaryMessage& out);
    bool reverseParseBasicType(QtProperty* pRoot, OutBinaryMessage& out);
    template <class T>
    bool reverseParseBasicTypeValue(QtProperty* pNode, OutBinaryMessage& out);

  private:
    ENUM(GROUP_TYPE,
        CLASS,
        LIST, //list with variable size (size can be different in the next frame)
        ARRAY //list with fixed size
        );

    /**
     * Classes and lists are represented as group-nodes.
     * However there is only one type of group node. Thus upon converting
     * classes and lists to groups we lose a part of the type information.
     * This part is stored here.
     */
    std::map<QtProperty*, GROUP_TYPE> theGroupTypes;

    /*
     * Sets the properties value to value if the value is different from the current one.
     */
    template <class T>
    void setPropertyValue(QtVariantProperty* pProperty,const T& value);

    /**
     * Handles all events coming from the property editors.
     * @param pEditor the editor that is the source of this event.
     * @param pProperty the property which belongs to the editor
     * @return true if the event has been handled, otherwise false.
     *         If false is returned the event will be forwarded to other eventHandlers.
     */
    bool handlePropertyEditorEvent(QWidget* pEditor, QtProperty* pProperty, QEvent* pEvent);

    /**
     * Some types are const.
     * This method removes the const qualifier from the type.
     * @return true if the type was const, false otherwise.
     */
    bool checkEraseConst(std::string& type);

    /**
     * Prints an error message to the console.
     */
    void printError(const std::string& error);

    /**
     * Returns the property with the specified fully qualified name.
     * If a property with the fqn already exists it is returned, else a new property is created.
     * The Property is added to the parent if it is not a child of pParent already.
     * @param propertyType the property type. See documentation of QtVariantPropertyManager for a list of allowed types.
     * @param fqn the fully qualified name of the property.
     */
    QtVariantProperty* getProperty(const std::string& fqn, int propertyType,const QString& name, QtProperty* pParent);

    /**
     * Appends another name to a fully qualified name.
     */
    std::string addToFQN(const std::string& fqn, const std::string& add);

  private:
    DECLARE_SYNC;

    const QString theFullName; /**< The path to this view in the scene graph */
    const QIcon theIcon; /**< The icon used for listing this view in the scene graph */
    RobotConsole& theConsole; /**< A reference to the console object. */
    const std::string theName; /**< The name of the view. And the Representation */
    RepresentationWidget* pTheWidget; /** < The widget which displays the properties */
    StreamHandler& theStreamHandler;
    PropertyManager theVariantManager; /** < responsible for the creation and destruction of QtProperties */
    bool theIgnoreUpdatesFlag; /** < If true handleMessage returns without doing anything */
    QtProperty* pTheCurrentRootNode; /** Pointer to the current root property */
    typedef std::map<std::string, QtVariantProperty*> PropertiesMapType;
    /**
     * This map is used to store all properties that have been created so far.
     * Recreating the properties on on every parser run causes memory leaks and gui bugs.
     * key: the fully qualified name of a property ( e.g. "this.is.a.fully.qualified.name")
     * value: pointer to the property.
     */
    PropertiesMapType theProperties;
    /**
     * Contains a bool for each property indicating whether this property
     * should be updated or not.
     * true = do not update the property.
     */
    std::map<QtProperty*, bool> excludeFromUpdate;

    unsigned char theMessageCounter; /** < Used to handle only every x'th message */
    /**
     * True if auto-set is enabled.
     * In auto-set mode the view will send a set command directly after the user finished editing one value.
     */
    bool theAutoSetModeIsEnabled;

    //The widget needs to access theConsole to synchronize while drawing.
    friend class RepresentationWidget;

    //The EventFilter forwards the propertyEditor events to the view.
    //To do that it needs access to handlePropertyEditorEvent;
    friend class EditorEventFilter;


};

template<class T> inline void RepresentationView::setPropertyValue(QtVariantProperty *pProperty,const T& value)
{
  if (value != theVariantManager.value(pProperty).value<T>())
  {
    pProperty->setValue(value);
  }
}

/**
 * This is a helper method for reverseParseBasicType.
 *
 */
template<class T> bool RepresentationView::reverseParseBasicTypeValue(QtProperty* pNode, OutBinaryMessage& out)
{
  int type = theVariantManager.propertyType(pNode);

  if(TypeDescriptor::getTypeId<T>() == type)
  {
   T value = theVariantManager.value(pNode).value<T>();
   out << value;
   return true;
  }
  else
  {
    return false;
  }
}

