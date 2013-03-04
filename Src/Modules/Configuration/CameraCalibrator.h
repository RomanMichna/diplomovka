/**
* @file CameraCalibrator.h
*
* This file implements a module that can provide a semiautomatic camera calibration.
*
* @author Alexander H�rtl
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/ImageInfo.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Math/YaMatrix.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>

MODULE(CameraCalibrator)
  REQUIRES(FieldDimensions)
  REQUIRES(FilteredJointData)
  REQUIRES(RobotDimensions)
  REQUIRES(TorsoMatrix)
  REQUIRES(ImageInfo)
  REQUIRES(CameraInfo)
  USES(RobotPose)
  USES(CameraMatrix)
  PROVIDES_WITH_MODIFY(CameraCalibration)
END_MODULE

class CameraCalibrator : public CameraCalibratorBase
{
private:
  /**
   * A class representing the parameters of the CameraCalibrator
   */
  class Parameters : public Streamable
  {
  public:
    bool calibrateBothCameras; /**< Whether both cameras or only the lower camera should be calibrated */
    bool errorInImage; /**< Whether the error is computed from the distance in the image or on the ground */
    float terminationCriterion; /**< The difference of two succesive parameter sets that are taken as a convergation */
    float aboveHorizonError; /**< The error for a sample the error of which cannot be computed regularly */
    int framesToWait; /**< The number of frames to wait between two iterations (necessary to keep the debug connection alive) */
    int successiveConvergations; /**< The number of consecutive iterations that fulfil the termination criterion to converge */

    Parameters() : calibrateBothCameras(true),
      errorInImage(true),
      terminationCriterion(0.01f),
      aboveHorizonError(1000000.0f),
      framesToWait(15),
      successiveConvergations(5)
    {}

  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(calibrateBothCameras);
      STREAM(errorInImage);
      STREAM(terminationCriterion);
      STREAM(aboveHorizonError);
      STREAM_REGISTER_FINISH;
    }
  } parameters;

  /**
   * The current state of the calibrator.
   */
  ENUM(CalibrationState,
    Idle,
    Accumulate,
    Optimize
  );
  CalibrationState calibrationState;

  /**
   * A class representing a single reference point within the calibration procedure.
   * It contains all information necessary to construct a camera matrix for the point
   * in time the sample was taken using an arbitrary camera calibration.
   */
  class Sample
  {
  public:
    Vector2<int> pointInImage;
    TorsoMatrix torsoMatrix;
    float headYaw, headPitch;
    bool upperCamera;
  };

  /**
   * This enum is used to translate between the indices of the parameter vector used in the
   * optimizer and their actual meaning.
   */
  ENUM(ParameterTranslation,
    cameraTiltCorrection,
    cameraRollCorrection,
    bodyTiltCorrection,
    bodyRollCorrection,
    robotPoseCorrectionX,
    robotPoseCorrectionY,
    robotPoseCorrectionRot,
    numOfParametersLowerCamera,
    upperCameraX = numOfParametersLowerCamera,
    upperCameraY,
    upperCameraZ
  );

  vector<Sample> samples; /**< The set of samples used to calibrate the camera. */

  Vector2<int> lastFetchedPoint; /**< The coordinates of the last fetched point in the image. */

  /**
   * The method to calculate the new camera calibration, depending on the state of the calibrator.
   * @param cameraCalibration The current calibration of the robot's camera.
   */
  void update(CameraCalibration& cameraCalibration);

  /**
   * This method computes the distance of a sampled point to the next field line, either in image
   * image coordinates using the back projection of the field lines into the image, or in field
   * coordinates using the projection of the point onto the field plane. The error is computed
   * using the given camera calibration from which a modified camera matrix is built.
   * @param sample The sample point for which the distance / error should be computed.
   * @param cameraCalibration The camera calibration used to compute the error.
   * @param robotPose The assumed robot pose.
   * @param inImage Whether the distance in image or in field coordinates should be computed.
   * @return The distance.
   */
  float computeError(const Sample& sample, const CameraCalibration& cameraCalibration, const RobotPose& robotPose, bool inImage = true) const;

  /**
   * This method computes the error value for a sample and a parameter vector.
   * @param sample The sample point for which the distance / error should be computed.
   * @param parameters The parameter vector for which the error should be evaluated.
   * @return The error.
   */
  float computeErrorParameterVector(const Sample& sample, const vector<float>& parameters) const;

  /**
   * This method converts a parameter vector to a camera calibration and a robot pose they stand for.
   * @param parameters The parameter vector to be translated.
   * @param cameraCalibration The camera calibration the values of which are set to the corresponding values in the parameter vector.
   * @param robotPose The robot pose the values of which are set to the corresponding values in the parameter vector.
   */
  void translateParameters(const vector<float>& parameters, CameraCalibration& cameraCalibration, RobotPose& robotPose) const;

  /**
   * This method converts a camera calibration and a robot pose to a parameter vector.
   * @param cameraCalibration The camera calibration to be translated.
   * @param robotPose The robot pose to be translated.
   * @param parameters The resulting parameter vector containing the values from the given camera calibration and robot pose.
   */
  void translateParameters(const CameraCalibration& cameraCalibration, const RobotPose& robotPose, vector<float>& parameters) const;

  /**
   * The method to fetch a point from a click in the image view.
   */
  void fetchPoint();

  /**
   * This method projects a line given in robot relative field coordinates into
   * the image using an arbitrary camera matrix.
   * @param lineOnField The field line in robot relative coordinates.
   * @param cameraMatrix The camera matrix used for the projection.
   * @param lineInImage The field line projected into the image, if this is possible.
   * @return Whether a valid result was computed, which is not the case if the field line lies completely behind the camera plane.
   */
  bool projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, Geometry::Line& lineInImage) const;

  /**
   * This method creates a debug drawing in which all field lines are projected into the image.
   */
  void drawFieldLines();

  /**
   * This class implements the Gauss-Newton algorithm.
   * A set of parameters is optimized in regard of the sum of squared errors using
   * a given error function. The jacobian that is computed in each iteration is
   * approximated numerically.
   * @tparam M The class that represents a single measurement / sample.
   * @tparam C The class the error function is a member of.
   */
  template <class M, class C>
  class GaussNewtonOptimizer
  {
  private:
    const unsigned int numOfMeasurements; /**< The number of measurements. */
    const unsigned int numOfParameters; /**< The number of parameters. */
    YaMatrix<float> currentParameters; /**< The vector (Nx1-matrix) containing the current parameters. */
    YaMatrix<float> currentValues; /**< The vector (Nx1-matrix) containing the current error values for all measurements. */
    const vector<M>& measurements; /**< A reference to the vector containing all measurements. */
    const C& object; /**< The object used to call the error function. */
    float(C::*pFunction)(const M& measurement, const vector<float>& parameters) const;  /**< A pointer to the error function. */
    const float delta; /**< The delta used to approximate the partial derivatives of the Jacobian. */

  public:
    GaussNewtonOptimizer(const vector<float>& parameters, const vector<M>& measurements, const C& object, float(C::*pFunction)(const M& measurement, const vector<float>& parameters) const)
      : numOfMeasurements(measurements.size()), numOfParameters(parameters.size()), currentParameters(numOfParameters, 1), currentValues(numOfMeasurements, 1), measurements(measurements), object(object), pFunction(pFunction), delta(0.001f)
    {
      for(unsigned int i = 0; i < numOfParameters; ++i)
      {
        currentParameters[i][0] = parameters[i];
      }
      for(unsigned int i = 0; i < numOfMeasurements; ++i)
      {
        currentValues[i][0] = (object.*pFunction)(measurements[i], currentParameters.transpose()[0]);
      }
    }

    /**
     * This method executes one iteration of the Gauss-Newton algorithm.
     * The new parameter vector is computed by a_i+1 = a_i - (D^T * D)^-1 * D^T * r
     * where D is the Jacobian, a is the parameter vector and r is the vector containing the current error values.
     * @return The sum of absolute differences between the old and the new parameter vector.
     */
    float iterate()
    {
      // build jacobi matrix
      YaMatrix<float> jacobiMatrix(numOfMeasurements, numOfParameters);
      for(unsigned int j = 0; j < numOfParameters; ++j)
      {
        // the first derivative is approximated using values slightly above and below the current value
        const float oldParameter = currentParameters[j][0];
        const float parameterAbove = oldParameter + delta;
        const float parameterBelow = oldParameter - delta;
        for(unsigned int i = 0; i < numOfMeasurements; ++i)
        {
          // approximate first derivation numerically
          currentParameters[j][0] = parameterAbove;
          const float valueAbove = (object.*pFunction)(measurements[i], currentParameters.transpose()[0]);
          currentParameters[j][0] = parameterBelow;
          const float valueBelow = (object.*pFunction)(measurements[i], currentParameters.transpose()[0]);
          const float derivation = (valueAbove - valueBelow) / (2.0f * delta);
          jacobiMatrix[i][j] = derivation;
        }
        currentParameters[j][0] = oldParameter;
      }

      try
      {
        YaMatrix<float> result = (jacobiMatrix.transpose() * jacobiMatrix).inverse() * jacobiMatrix.transpose() * currentValues;
        currentParameters -= result;

        for(unsigned int i = 0; i < numOfMeasurements; ++i)
        {
          currentValues[i][0] = (object.*pFunction)(measurements[i], currentParameters.transpose()[0]);
        }

        float sum = 0;
        for(unsigned int i = 0; i < numOfParameters; ++i)
        {
          sum += abs(result[i][0]);
        }
        return sum;
      }
      catch(...)
      {
        return 0.0f;
      }

    }

    /**
     * The method returns the current parameter vector.
     * @return The current parameter vector.
     */
    vector<float> getParameters() const
    {
      return currentParameters.transpose()[0];
    }
  };

  GaussNewtonOptimizer<Sample, CameraCalibrator>* optimizer; /**< A pointer to the currently used optimizer, or NULL if there is none. */

  int successiveConvergations; /**< The number of consecutive iterations that fulfil the termination criterion. */
  int framesToWait; /**< The remaining number of frames to wait for the next iteration. */

  const CameraCalibration* currentCameraCalibration; /**< A pointer to the current camera calibration, refreshed in every call of the update method. */

public:
  /** Default constructor. */
  CameraCalibrator();

  /** Destructor. */
  ~CameraCalibrator();
};
