/** 
* @file Simulation/Sensors/ApproxDistanceSensor.cpp
* Implementation of class ApproxDistanceSensor
* @author Colin Graf
*/

#include "Platform/OpenGL.h"
#include <cmath>

#include "Simulation/Sensors/ApproxDistanceSensor.h"
#include "Simulation/Geometries/Geometry.h"
#include "Platform/Assert.h"
#include "Tools/OpenGLTools.h"
#include "Tools/ODETools.h"
#include "CoreModule.h"

ApproxDistanceSensor::ApproxDistanceSensor()
{
  sensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  sensor.unit = "m";
}

void ApproxDistanceSensor::createPhysics()
{
  OpenGLTools::convertTransformation(rotation, translation, transformation);

  sensor.tanHalfAngleX = tan(angleX * 0.5f);
  sensor.tanHalfAngleY = tan(angleY * 0.5f);
  float width = sensor.tanHalfAngleX * max * 2.f;
  float height = sensor.tanHalfAngleY * max * 2.f;
  float depth = max;
  sensor.geom = dCreateBox(Simulation::simulation->rootSpace, depth, width, height);
  sensor.min = min;
  sensor.max = max;
  sensor.maxSqrDist = max * max;
  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;
}

void ApproxDistanceSensor::registerObjects()
{
  sensor.fullName = fullName + ".distance";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void ApproxDistanceSensor::addParent(Element& element)
{
  sensor.physicalObject = dynamic_cast< ::PhysicalObject*>(&element);
  ASSERT(sensor.physicalObject);
  Sensor::addParent(element);
}

void ApproxDistanceSensor::DistanceSensor::staticCollisionCallback(ApproxDistanceSensor::DistanceSensor* sensor, dGeomID geom1, dGeomID geom2)
{
  ASSERT(geom1 == sensor->geom);
  ASSERT(!dGeomIsSpace(geom2));
  
  Geometry* geometry = (Geometry*)dGeomGetData(geom2);
  const dReal* pos = dGeomGetPosition(geom2);
  const Vector3<> geomPos(pos[0], pos[1], pos[2]);
  const float approxSqrDist = (geomPos - sensor->pose.translation).squareAbs() - geometry->approxRadiusSqr;
  if(approxSqrDist < sensor->closestSqrDistance)
  {
    Vector3<> relPos = sensor->invertedPose * geomPos;
    if(relPos.x <= 0.f)
      return;

    if(std::max(std::abs(relPos.y) - geometry->approxRadius, 0.f) < sensor->tanHalfAngleX * relPos.x)
      if(std::max(std::abs(relPos.z) - geometry->approxRadius, 0.f) < sensor->tanHalfAngleY * relPos.x)
      {
        sensor->closestSqrDistance = approxSqrDist;
        sensor->closestGeom = geom2;
      }
  }
}

void ApproxDistanceSensor::DistanceSensor::staticCollisionWithSpaceCallback(ApproxDistanceSensor::DistanceSensor* sensor, dGeomID geom1, dGeomID geom2)
{
  ASSERT(geom1 == sensor->geom);
  ASSERT(dGeomIsSpace(geom2));
  dSpaceCollide2(geom1, geom2, sensor, (dNearCallback*)&staticCollisionCallback);
}

void ApproxDistanceSensor::DistanceSensor::updateValue()
{
  pose = physicalObject->pose;
  pose.conc(offset);
  invertedPose = pose.invert();
  Vector3<> boxPos = pose * Vector3<>(max * 0.5f, 0.f, 0.f);
  dGeomSetPosition(geom, boxPos.x, boxPos.y, boxPos.z);
  dMatrix3 matrix3;
  ODETools::convertMatrix(pose.rotation, matrix3);
  dGeomSetRotation(geom, matrix3);
  closestGeom = 0;
  closestSqrDistance = maxSqrDist;
  dSpaceCollide2(geom, (dGeomID)Simulation::simulation->movableSpace, this, (dNearCallback*)&staticCollisionWithSpaceCallback);
  dSpaceCollide2(geom, (dGeomID)Simulation::simulation->staticSpace, this, (dNearCallback*)&staticCollisionCallback);
  if(closestGeom)
  {
    const dReal* pos = dGeomGetPosition(closestGeom);
    Geometry* geometry = (Geometry*)dGeomGetData(closestGeom);
    data.floatValue = (Vector3<>(pos[0], pos[1], pos[2]) - pose.translation).abs() - geometry->approxRadius;
    if(data.floatValue < min)
      data.floatValue = min;
  }
  else
    data.floatValue = max;
}

bool ApproxDistanceSensor::DistanceSensor::getMinAndMax(float& min, float& max) const
{
  min = this->min;
  max = this->max;
  return true;
}

void ApproxDistanceSensor::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showSensors)
  {
    Vector3<> ml(max, -tan(angleX * 0.5f) * max, 0);
    Vector3<> mt(max, 0, tan(angleY * 0.5f) * max);
    Vector3<> tl(max, ml.y, mt.z);
    Vector3<> tr(max, -ml.y, mt.z);
    Vector3<> bl(max, ml.y, -mt.z);
    Vector3<> br(max, -ml.y, -mt.z);

    glBegin(GL_LINE_LOOP);
      glColor3f(0.5f, 0, 0);
      glNormal3f (0, 0, 1.f);
      glVertex3f(tl.x, tl.y, tl.z);
      glVertex3f(tr.x, tr.y, tr.z);
      glVertex3f(br.x, br.y, br.z);
      glVertex3f(bl.x, bl.y, bl.z);
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(tl.x, tl.y, tl.z);
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(tr.x, tr.y, tr.z);
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(bl.x, bl.y, bl.z);
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(br.x, br.y, br.z);
    glEnd();
  }

  Sensor::drawPhysics(flags);
  glPopMatrix();
}
