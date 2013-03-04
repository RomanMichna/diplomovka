/*
 * RepresentationView.cpp
 *
 *  Created on: Mar 22, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#include "RepresentationView.h"
#include <QWidget>
#include <algorithm>
#include "Controller/RoboCupCtrl.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Streams/StreamHandler.h"
#include <climits>
#include <sstream>
#include "Controller/RobotConsole.h"
#include "Platform/Thread.h"
#include "RepresentationWidget.h"
#include "TypeDescriptor.h" //for getTypeId()
#include <QEvent>
#include <QString>



using namespace Type;

RepresentationView::RepresentationView(const QString& fullName,
                                       const std::string& repName,
                                       RobotConsole& console,
                                       StreamHandler& streamHandler) :
  theFullName(fullName), theIcon(":/Icons/tag_green.png"),
  theConsole(console), theName(repName), pTheWidget(NULL),
  theStreamHandler(streamHandler), theIgnoreUpdatesFlag(false),
  theMessageCounter(0),
  theAutoSetModeIsEnabled(true)
{} //FIXME destructor

bool RepresentationView::handleMessage(InMessage& msg,const std::string& type, const std::string& repName)
{
  bool result = true;
  theMessageCounter++;

  if(NULL != pTheWidget &&//do nothing if no widget is associated with this view.
    !theIgnoreUpdatesFlag && //Or updates should be ignored.
    (theMessageCounter % 10 == 0)) //only handle every 10th message to reduce the performance impact.
  {

    SYNC;//This sync is here and not inside of parse() because parse() is recursive and the lock not reentrant.
    pTheCurrentRootNode = parse(msg, type, repName.c_str(), NULL,"");// NULL means no parent node, "" is the empty fully qualified name of the parent.

    if(NULL != pTheCurrentRootNode)
    {
      //Tell the widget to display the root property
      pTheWidget->setRootProperty(pTheCurrentRootNode);
      result = true;
    }
    else
    {
      printError("Parsing failed");
      result = false;
    }
  }
  else
  {
    result = false;//this is not an error.
  }
  return result;
}

SimRobot::Widget *RepresentationView::createWidget()
{
  pTheWidget = new RepresentationWidget(*this, theVariantManager);
  return pTheWidget;
}

QtProperty* RepresentationView::parse(InMessage & msg, const std::string & type, const QString& fieldName,  QtProperty *pParent, const std::string& parentFQN)
{

  //This parser is based on RobotConsole::writeConfigMap
  //typical recursive parser :-)
  QtProperty* pProp;

  pProp = parseBasicType(msg, type, fieldName, pParent,parentFQN);
  if(NULL != pProp)
  {
    return pProp;
  }

  pProp = parseClass(msg, type, fieldName, pParent,parentFQN);
  if(NULL != pProp)
  {
    return pProp;
  }
  pProp = parseArray(msg, type, fieldName, pParent,parentFQN);
  if(NULL != pProp)
  {
    return pProp;
  }
  pProp = parseEnum (msg, type, fieldName, pParent,parentFQN);
  if(NULL != pProp)
  {
    return pProp;
  }

  //failure
  return NULL;

}

QtProperty* RepresentationView::parseArray(InMessage & msg, const std::string & type, const QString& fieldName, QtProperty *pParent, const std::string& parentFQN)
{

  //The Property browser does not support lists directly.
  //Therefore they are simulated using a group property and sub-entries
  GROUP_TYPE grpType;
  char last = type[type.size()-1];
  int size = -1;
  std::string typeName;
  //determine size
  if(last == ']') //const size
  {
    grpType = ARRAY;
    // syntax:  name[size]
    size_t openingIndex = type.find_last_of('[');
    size_t closingIndex = type.size()-1;

    if(openingIndex != type.npos && //if'[' was found
       openingIndex < closingIndex-1) //and there are values between '[' and ']'
    {
      std::stringstream ss;
      ss << type.substr(openingIndex+1,closingIndex-openingIndex-1);
      ss >> size;

      typeName = type.substr(0,openingIndex-1);//remove size from typename

    }

  }
  else if(last == '*') //dynamic size
  {
    grpType = LIST;
    //size is streamed before data
    msg.bin >> size;
    typeName = type.substr(0,type.size() -1); //remove * from typename
  }
  else
  { //not an array/list/vector type
    return NULL;
  }

  //trim typename
  typeName.erase(typeName.find_last_not_of(" \n\r\t")+1);


  std::string fqn = addToFQN(parentFQN, fieldName.toAscii().data());
  QtProperty* pListRoot = getProperty(fqn, TypeDescriptor::getGroupType(), fieldName, pParent);
  theGroupTypes[pListRoot] = grpType;

  //The size may vary: If it shrinks we need to remove the children.
  //This is possible because getProperty buffers the property and it's sub-properties
  //Thus after the first call we will get the same property every time.
  while (size < pListRoot->subProperties().size())
  {
    pListRoot->removeSubProperty(pListRoot->subProperties().last());
  }

  //the stream contains size elements of type typename, parse all of them
  for(int i = 0; i < size; i++)
  {
    if (NULL == parse(msg, typeName, QString::number(i), pListRoot, fqn))
    {
      return NULL; //failure while parsing one of the array items
    }
  }

  return pListRoot;
}


QtProperty* RepresentationView::parseClass(InMessage & msg, const std::string & type, const QString& fieldName, QtProperty *pParent, const std::string& parentFQN)
{

  //The type is modified before we can say for sure whether this is a class type
  //or not. Therefore copy it.
  std::string typeName = type;

  bool isConst = checkEraseConst(typeName);

  const char* t = theStreamHandler.getString(typeName);

  //Search the known class types for this type string.
  StreamHandler::Specification::const_iterator i = theStreamHandler.specification.find(t);

  if(i == theStreamHandler.specification.end())
  {//nope, not a known class type.
    return NULL;
  }

  //If we arrive at this position this is a valid class type

  std::string fqn = addToFQN(parentFQN,fieldName.toAscii().data());
  //create root node for this class and add it to our parent
  QtVariantProperty* pClassRoot = getProperty(fqn, TypeDescriptor::getGroupType(), fieldName, pParent);
  pClassRoot->setEnabled(!isConst);
  theGroupTypes[pClassRoot] = CLASS;
  //parse attributes
  for(std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
  {
    QString attrName = j->first.c_str();
    std::string attrType(j->second);

    if(NULL == parse(msg, attrType, attrName, pClassRoot, fqn))
    {
      return NULL; //failure while parsing one of the children
    }
  }

  return pClassRoot;
}



QtProperty* RepresentationView::parseEnum(InMessage & msg, const std::string & type, const QString& fieldName, QtProperty *pParent, const std::string& parentFQN)
{
  //The type is modified before we can say for sure whether this is a class type
  //or not. Therefore copy it.
  std::string typeName = type;

  bool isConst = checkEraseConst(typeName);

  const char* t = theStreamHandler.getString(typeName);

  //Search the known enum types for this type string.
  StreamHandler::EnumSpecification::const_iterator enumSpec = theStreamHandler.enumSpecification.find(t);

  if(enumSpec == theStreamHandler.enumSpecification.end())
  {//nope, not a known enum type
    return NULL;
  }

  //if we reach this position this is a valid enum type
  //enumSpec->second is a vector containing all possible enum names
  unsigned int enumLength = enumSpec->second.size();
  unsigned int value;
  msg.bin >> value;

  std::string fqn = addToFQN(parentFQN, fieldName.toAscii().data());
  QtVariantProperty* pProp = NULL;
  if(value < enumLength)
  {
    pProp = getProperty(fqn, TypeDescriptor::getEnumTypeId(),fieldName, pParent);

    //an enums type definition does not change at runtime.
    //only update the enumNames if they are empty.
    QStringList enumNames = pProp->attributeValue("enumNames").value<QStringList>();

    if(enumNames.empty())
    {
      //Build list with all possible enum values
      std::vector<const char*>::const_iterator enumIt;

      for(enumIt = enumSpec->second.begin(); enumIt != enumSpec->second.end(); enumIt++)
      {
        enumNames << (*enumIt);
      }

      //Give the enum name list to the property
      pProp->setAttribute(QLatin1String("enumNames"), enumNames);
      pProp->setEnabled(!isConst);
    }

    setPropertyValue(pProp, value);
  }
  else
  {
    printError(" error while parsing enum " + typeName + ". Value out of range");
    return NULL;
  }

  return pProp;
}



QtProperty* RepresentationView::parseBasicType(InMessage & msg, const std::string & type, const QString& fieldName, QtProperty *pParent, const std::string& parentFQN)
{
  //The type is modified before we can say for sure whether this is a class type
  //or not. Therefore copy it.
  std::string typeName = type;

  bool isConst = checkEraseConst(typeName);
  const char* t = theStreamHandler.getString(typeName.c_str());

  //check if this is a known basic type
  StreamHandler::BasicTypeSpecification::const_iterator i = theStreamHandler.basicTypeSpecification.find(t);
  if(theStreamHandler.basicTypeSpecification.end() == i)
  {
    return NULL;
  }

  //if we arrive at this position this is a basic type
  std::string fqn = addToFQN(parentFQN, fieldName.toAscii().data());
  QtVariantProperty* pProp = NULL;
  //determine basic type and add it to our parent
  if (typeName == "double")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<double>(),fieldName,pParent);
    double d;
    msg.bin >> d;
    setPropertyValue(pProp, d);

  }
  else if (typeName == "bool")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<bool>(),fieldName,pParent);
    bool b;
    msg.bin >> b;
    setPropertyValue(pProp, b);
  }
  else if (typeName == "float")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<float>(),fieldName,pParent);
    float f;
    msg.bin >> f;
    setPropertyValue(pProp, f);
  }
  else if (typeName == "int")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<int>(),fieldName,pParent);
    int i;
    msg.bin >> i;
    setPropertyValue(pProp, i);
  }
  else if (typeName == "unsigned" || typeName == "unsigned int")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<unsigned int>(),fieldName,pParent);
    unsigned int i;
    msg.bin >> i;
    setPropertyValue(pProp, i);
  }
  else if (typeName == "short")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<short>(),fieldName,pParent);
    short i;
    msg.bin >> i;
    setPropertyValue(pProp, i);
  }
  else if (typeName == "unsigned short")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<unsigned short>(),fieldName,pParent);
    unsigned short i;
    msg.bin >> i;
    setPropertyValue(pProp, i);
  }
  else if (typeName == "char")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<char>(),fieldName,pParent);
    char i;
    msg.bin >> i;
    setPropertyValue(pProp, QChar(i));
  }
  else if (typeName == "unsigned char")
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<unsigned char>(),fieldName,pParent);
    unsigned char c;
    msg.bin >> c;
    setPropertyValue(pProp, c);
  }
  else if (typeName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >" ||
           typeName == "std::string") //FIXME this only works with ascii strings.
  {
    pProp = getProperty(fqn,TypeDescriptor::getTypeId<std::string>(),fieldName,pParent);
    std::string s;
    msg.bin >> s;
    setPropertyValue(pProp, QString::fromAscii(s.c_str()));
  }

  if(NULL != pProp)
  {
    //property should be readonly if the type is const.
    pProp->setEnabled(!isConst);
  }
  return pProp;
}

bool RepresentationView::checkEraseConst(std::string & type)
{
  if(type.size() > 6 && type.substr(type.size() - 6) == " const")
  {
    type = type.substr(0, type.size() - 6);
    return true;
  }
  return false;
}


void RepresentationView::printError(const std::string & error)
{
  std::string msg(theFullName.toAscii().data());
  theConsole.printLn(msg + ": " + error);
}

QtVariantProperty* RepresentationView::getProperty(const std::string & fqn, int propertyType,const QString & name, QtProperty *pParent)
{
  QtVariantProperty* pProp = NULL;
  PropertiesMapType::iterator propIt = theProperties.find(fqn);

  if(propIt == theProperties.end())
  {
    //This is a new property.
    pProp = theVariantManager.addProperty(propertyType,name);
    theProperties[fqn] = pProp;
    excludeFromUpdate[pProp] = false;
  }
  else
  { //property already exists, load it.
    pProp = theProperties[fqn];
  }

  //add to parent if it is not already a child of the parent.
  if(NULL != pParent && !pParent->subProperties().contains(pProp))
  {
    pParent->addSubProperty(pProp);
  }

  return pProp;
}

void RepresentationView::removeWidget()
{
  SYNC;
  pTheWidget = NULL;
  theProperties.clear();
}

void RepresentationView::setIgnoreUpdates(bool value)
{
  SYNC;
  theIgnoreUpdatesFlag = value;
}


void RepresentationView::setUnchanged()
{
  {
    SYNC;

    MessageQueue tempQ; //temp queue used to build the message

    std::string debugRequest = theConsole.getDebugRequest(theName);
    tempQ.out.bin << debugRequest << char(0);//0 means unchanged
    tempQ.out.finishMessage(idDebugDataChangeRequest);

    theConsole.sendDebugMessage(tempQ.in);
    theIgnoreUpdatesFlag = false;
  }
  pTheWidget->setUnchangedButtonEnabled(false);
}

void RepresentationView::set()
{
  {
    SYNC;

    MessageQueue tempQ; //temp queue used to build the message

    std::string debugRequest = theConsole.getDebugRequest(theName);

    tempQ.out.bin << debugRequest << char(1);
    if(NULL != pTheCurrentRootNode)
    {
      reverseParse(pTheCurrentRootNode, tempQ.out.bin);
    }

    tempQ.out.finishMessage(idDebugDataChangeRequest);

    theConsole.sendDebugMessage(tempQ.in);
    theIgnoreUpdatesFlag = false; //setIgnoreUpdates is not called because it syncs again
  }
  pTheWidget->setSetButtonEnabled(false);
  pTheWidget->setUnchangedButtonEnabled(true);
}

bool RepresentationView::reverseParse(QtProperty* pRoot, OutBinaryMessage& out)
{
  if (NULL == pRoot)
  {
    return false;
  }



  return (reverseParseBasicType(pRoot, out) ||
          reverseParseClass(pRoot, out)     ||
          reverseParseArray(pRoot, out)     ||
          reverseParseEnum(pRoot, out)       );

}

bool RepresentationView::reverseParseClass(QtProperty* pNode, OutBinaryMessage& out)
{
  if(theGroupTypes.find(pNode) != theGroupTypes.end())
  {
    GROUP_TYPE type = theGroupTypes[pNode];

    if(CLASS == type)
    {
      //simply parse all sub-items :-)
      bool retValue = true;
      for(int i = 0; i < pNode->subProperties().size(); i++)
      {
        retValue &= reverseParse(pNode->subProperties().at(i), out);
      }

      return retValue;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}


bool RepresentationView::reverseParseBasicType(QtProperty* pNode, OutBinaryMessage& out)
{
  return reverseParseBasicTypeValue <char>(pNode, out)           ||
         reverseParseBasicTypeValue <unsigned char>(pNode, out)  ||
         reverseParseBasicTypeValue <double>(pNode, out)         ||
         reverseParseBasicTypeValue <float>(pNode, out)          ||
         reverseParseBasicTypeValue <bool>(pNode, out)           ||
         reverseParseBasicTypeValue <int>(pNode, out)            ||
         reverseParseBasicTypeValue <unsigned int>(pNode, out)   ||
         reverseParseBasicTypeValue <short>(pNode, out)          ||
         reverseParseBasicTypeValue <unsigned short>(pNode, out) ||
         reverseParseBasicTypeValue <std::string>(pNode, out);
}

bool RepresentationView::reverseParseArray(QtProperty* pNode, OutBinaryMessage& out)
{
  if(theGroupTypes.find(pNode) != theGroupTypes.end())
  {
    GROUP_TYPE type = theGroupTypes[pNode];
    if(ARRAY == type || LIST == type)
    {
      if(LIST == type)
      {
        //lists have dynamic size, therefore the size has to be streamed before the data.
        unsigned int size = pNode->subProperties().size();
        out << size;
      }
      //now parse all sub items.
      bool retValue = true;
      QList<QtProperty*>::iterator i;
      for(int i = 0; i < pNode->subProperties().size(); i++)
      {
        retValue &= reverseParse(pNode->subProperties().at(i), out);
      }
      return retValue;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool RepresentationView::reverseParseEnum(QtProperty* pNode, OutBinaryMessage& out)
{
  if (theVariantManager.propertyType(pNode) == TypeDescriptor::getEnumTypeId())
  {
    int value = theVariantManager.value(pNode).value<int>();
    out << value;
    return true;
  }
  else
  {
    return false;
  }
}


void RepresentationView::setAutoSet(bool value)
{
  SYNC;
  theAutoSetModeIsEnabled = value;
}

std::string RepresentationView::addToFQN(const std::string & fqn, const std::string & add)
{
  if(fqn.empty())
  {
    return add;
  }
  else
  {
    return fqn + "." + add;
  }
}

bool RepresentationView::handlePropertyEditorEvent(QWidget* pEditor, QtProperty* pProperty, QEvent* pEvent)
{
  /**
   * Show is used as indication that the user wants to change the specified property.
   * This works because the property editor creates and shows a new editor widget every time the user
   * wants to edit something.
   * Property updating is interrupted until the user presses set (or until the user finishes his input in case of auto-set)
   */
  if(pEvent->type() == QEvent::Show)
  {
     setIgnoreUpdates(true);
  }
  else if(pEvent->type() == QEvent::Destroy)
  {
    if(theAutoSetModeIsEnabled)
    {
      set();//set current values
      setIgnoreUpdates(false); //continue updating
    }
    else
    {
      //enable the set button
      pTheWidget->setSetButtonEnabled(true);
    }
  }


  return false; //others might want to handle this event as well.
}














