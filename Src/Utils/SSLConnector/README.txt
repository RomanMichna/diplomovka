In order to provide synchronized ground truth data you have to broadcast
the SSL vision data with the SSLConnector. The SSL vision server has to be
patched:
  patch -p0 -i sharedmemory.diff
After compiling and starting the SSL vision server, you must start the
SSLConnector. The SSLConnector needs at least cmake 2.8. The SSLConnector
reads the ground truth information from a shared memory and broadcasts it
via team communication to all robots. Make sure that the SSLConnector uses
the same team number and team port as the robots.

The robot usually does not process the data. In order to activate this you
have to connect to the robot via simulator and send the following module
requests:
  mr SSLVisionData TeamDataProvider
  mr GroundTruthBallModel GroundTruthProvider
  mr GroundTruthRobotPose GroundTruthProvider
  mr GroundTruthResult GroundTruthEvaluator

You can plot the error of the models and the delay of the ground truth data
with the following commands:
  vp rotationalError 200 0 1.6
  vpd rotationalError module:GroundTruthEvaluator:rotationalError
  vp translationalError 200 0 3000
  vpd translationalError module:GroundTruthEvaluator:translationalError
  vp ballModelError 200 0 500
  vpd ballModelError module:GroundTruthEvaluator:ballPositionError
  vp groundTruthDelay 200 -200 200
  vpd groundTruthDelay module:GroundTruthEvaluator:evaluationDelay
