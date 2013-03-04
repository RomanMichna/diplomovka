#include <StateMachineBehavior>

option Striker
{
  state playingStriker()
  {
    action
    {
        //sitDown();
      //Stand();
      //theMotionRequest.motion = MotionRequest::specialAction;
      //theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
      //theSoundRequest.sound = SoundRequest::exhausted;
        //theMotionRequest.motion = MotionRequest::walk;
	//theMotionRequest.walkRequest.mode = WalkRequest::speedMode;
	//theMotionRequest.walkRequest.speed = Pose2D(50.0, 0.0);

	//kop bezier
	/*theMotionRequest.motion = MotionRequest::bike;
	theMotionRequest.bikeRequest.bMotionType = BikeRequest::kickForward;
	
	theMotionRequest.bikeRequest.dynamical = true;
  	theMotionRequest.bikeRequest.ballSpecial = true; 
	Vector3<float> strikeOut, kickTo, motionDirection(0.0, 0.0, 0.0);

  	strikeOut = Vector3<float>(-90.f, -60.f, -160.f);
  	kickTo = Vector3<float>(-30.f, -40.f, -160.f);

  	if(theMotionRequest.bikeRequest.dynPoints.size() != 2)
  	{
    		theMotionRequest.bikeRequest.dynPoints.resize(2);
  	}

  DynPoint dynRFoot3(Phase::rightFootTra, 3, 0, strikeOut, motionDirection,  Vector3<> (0.f, 0.f, 0.f)), //strikeout
           dynRFoot4(Phase::rightFootTra, 4, 0, kickTo, motionDirection,  Vector3<> (0.f, 0.f, 0.f)); //kickto
  //rFoot in Phase3
  	theMotionRequest.bikeRequest.dynPoints[0] = dynRFoot3;
  //rFoot in Phase4
  	theMotionRequest.bikeRequest.dynPoints[1] = dynRFoot4;*/
	
	//hladanie lopty
	//theHeadMotionRequest.target = Vector3<>(0.0, 0.0, 0.0);
	//theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
	Vector2<> ball; 
	ball = theBallModel.lastPerception;
	theMotionRequest.motion = MotionRequest::walk;
	theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
	theMotionRequest.walkRequest.speed = Pose2D(20.0, 0.0);
	theMotionRequest.walkRequest.target = Pose2D(ball[0], ball[1]);
    }
  }
};
