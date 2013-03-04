/*
 * CognitionLogger.cpp
 *
 *  Created on: Feb 10, 2012
 *      Author: arne
 */

#include "CognitionLogger.h"
#include <iostream>


MAKE_MODULE(CognitionLogger,Logging)

// m_processName is set to c because c identifies the Cognition process.
CognitionLogger::CognitionLogger() : Logger("_cognition.log"), theProcessName('c')
{

  //Lookup table to link representation names and their intsances.
  theRepresentationRefs["PotentialRobotPose"] = &thePotentialRobotPose;
  theRepresentationRefs["RobotPose"] = &theRobotPose;
  theRepresentationRefs["BallPercept"] = &theBallPercept;
  theRepresentationRefs["RobotModel"] = &theRobotModel;
  theRepresentationRefs["FallDownState"] = &theFallDownState;
  theRepresentationRefs["RobotPercept"] = &theRobotPercept;
  theRepresentationRefs["RegionPercept"] = &theRegionPercept;
  theRepresentationRefs["LinePercept"] = &theLinePercept;

  theRepresentationRefs["ImageCoordinateSystem"] = &theImageCoordinateSystem;
  theRepresentationRefs["GoalPercept"] = &theGoalPercept;
  theRepresentationRefs["FootPercept"] = &theFootPercept;
  theRepresentationRefs["BodyContour"] = &theBodyContour;
  theRepresentationRefs["RobotsModel"] = &theRobotsModel;
  theRepresentationRefs["USObstacleGrid"] = &theUSObstacleGrid;
  theRepresentationRefs["BallModel"] = &theBallModel;
  theRepresentationRefs["CombinedWorldModel"] = &theCombinedWorldModel;
  theRepresentationRefs["CameraMatrix"] = &theCameraMatrix;
  theRepresentationRefs["ObstacleModel"] = &theObstacleModel;
  theRepresentationRefs["BehaviorControlOutput"] = &theBehaviorControlOutput;
  theRepresentationRefs["GlobalFieldCoverage"] = &theGlobalFieldCoverage;
  theRepresentationRefs["MotionRequest"] = &theMotionRequest;
  theRepresentationRefs["HeadMotionRequest"] = &theHeadMotionRequest;
  theRepresentationRefs["ArmContactModel"] = &theArmContactModel;
  theRepresentationRefs["FieldCoverage"] = &theFieldCoverage;
  theRepresentationRefs["FreePartOfOpponentGoalModel"] = &theFreePartOfOpponentGoalModel;
  theRepresentationRefs["FilteredJointData"] = &theFilteredJointData;
  theRepresentationRefs["FrameInfo"] = &theFrameInfo;
  theRepresentationRefs["SideConfidence"] = &theSideConfidence;
  theRepresentationRefs["FilteredSensorData"] = &theFilteredSensorData;
  theRepresentationRefs["GameInfo"] = &theGameInfo;
  theRepresentationRefs["OwnTeamInfo"] = &theOwnTeamInfo;
  theRepresentationRefs["BehaviorDebugOutput"] = &theBehaviorDebugOutput;
  theRepresentationRefs["MotionInfo"] = &theMotionInfo;

  initRepresentations();

}

CognitionLogger::~CognitionLogger()
{}

list<pair<MessageID, const Streamable*> >& CognitionLogger::provideRepresentations()
{
  return m_representations;
}


void CognitionLogger::update(CognitionLoggerOutput& output)
{
  Logger::update(theGameInfo.state, theRobotInfo.penalty);
}

Logger::ProcessName CognitionLogger::getProcessName()
{

  return 'c';
}



void CognitionLogger::initRepresentations()
{
  map<Logger::ProcessName, list<Logger::RepresentationName> >::const_iterator i;
  i = getActiveRepresentations().find(theProcessName);

  //this occurs if there where no representations defined for the specified process.
  ASSERT(i != getActiveRepresentations().end());


  list<RepresentationName> const& activeReps = i->second;

  list<RepresentationName>::const_iterator it;
  for(it = activeReps.begin(); it != activeReps.end(); it++)
  {

    if (theRepresentationRefs.find(*it) != theRepresentationRefs.end())
    {
      MessageID id = getMessageId(*it);
      const Streamable* pRepresentation = theRepresentationRefs[*it];
      m_representations.push_back(make_pair(id,pRepresentation));

    }
    else
    {
      OUTPUT_WARNING("Logger: Cannot log representation " << *it << ". The reference to the representation is missing.");
    }
  }
}



