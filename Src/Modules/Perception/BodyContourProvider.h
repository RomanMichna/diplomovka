/**
* @file BodyContourProvider.h
* This file declares a module that provides the contour of the robot's body in the image.
* The contour can be used to exclude the robot's body from image processing.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Perception/BodyContour.h"

MODULE(BodyContourProvider)
  REQUIRES(CameraInfo)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(RobotCameraMatrix)
  REQUIRES(RobotModel)
  PROVIDES_WITH_MODIFY_AND_DRAW(BodyContour)
END_MODULE

/**
* @class BodyContourProvider
* A module that provides the contour of the robot's body in the image.
* The contour can be used to exclude the robot's body from image processing.
*/
class BodyContourProvider: public BodyContourProviderBase
{
private:
  class Parameters : public Streamable
  {
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read (if in != 0).
    * @param out The stream to which the object is written (if out != 0).
    */
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(torso);
      STREAM(upperArm);
      STREAM(lowerArm);
      STREAM(upperLeg1);
      STREAM(upperLeg2);
      STREAM(foot);
      STREAM_REGISTER_FINISH;
    }

  public:
    std::vector<Vector3<> > torso, /**< The contour of the torso. */
        upperArm, /**< The contour of the left upper arm. */
        lowerArm, /**< The contour of the left lower arm. */
        upperLeg1, /**< The contour of the left upper leg (part 1). */
        upperLeg2, /**< The contour of the left upper leg (part 2). */
        foot; /**< The contour of the left foot. */
  };

  Parameters parameters; /**< The parameters of this module. */

  Pose3D robotCameraMatrixInverted; /**< The inverse of the current robotCameraMatrix. */

  void update(BodyContour& bodyContour);

  /**
  * The method projects a point in world coordinates into the image using the precomputed
  * inverse of the robot camera matrix.
  *
  * @param pointInWorld The point to be projected into the image in robot relative coordinates.
  * @param pointInImage Guess what?!
  * @return Whether the point lies in front of the image plane.
  */
  bool calculatePointInImage(const Vector3<>& pointInWorld, Vector2<int>& pointInImage) const;

  /**
  * The method add a 3-D contour to the 2-D image contour.
  * @param origin The origin the 3-D coordinates will be interpreted relatively to.
  * @param c The 3-D contour.
  * @param sign The sign for y coordinates. Passing -1 allows to use contours of left
  *             body parts for right body parts.
  * @param bodyContour The 2-D contour in image coordinates the 3-D contour is added to.
  */
  void add(const Pose3D& origin, const std::vector<Vector3<> >& c, float sign, BodyContour& bodyContour);

public:
  /** Default constructor. */
  BodyContourProvider();
};
