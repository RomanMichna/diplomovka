#include "TeamMarkerPerceptor.h"

#include "Platform/SystemCall.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Asserts.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Common.h"

MAKE_MODULE(TeamMarkerPerceptor, Perception)

#ifndef POINT_IS_WITHIN_IMAGE
#define POINT_IS_WITHIN_IMAGE(p, cameraInfo) \
  ((p).x >= 0 && (p).x < (cameraInfo).resolutionWidth \
   && (p).y >= 0 && (p).y < (cameraInfo).resolutionHeight)
#endif

#ifndef ABORTABLE_STEP
#define ABORTABLE_STEP(function, coordinates, color) \
  if(!function) \
  { \
    CROSS("module:TeamMarkerPerceptor:rejected", (coordinates).x, (coordinates).y, \
          5, 2, Drawings::ps_solid, color); \
    break; \
  }
#endif

void TeamMarkerPerceptor::init()
{
  InConfigMap stream("teamMarkerPerceptor.cfg");
  ASSERT(stream.exists());
  stream >> params;

  regionLength = 0.0;
  regionHeight = 0.0;
  approxCog = Vector2<int>(0, 0);
  above.clear();
  above.reserve(theImage.resolutionWidth / scanlineDiff);
  below.clear();
  below.reserve(theImage.resolutionWidth / scanlineDiff);
  cbStart = 0;
  crStart = 0;
  width = 0.0;
  scanlineSegments.clear();
  scanlineSegments.reserve(theImage.resolutionWidth / scanlineDiff);
  polygon.clear();
  polygon.reserve(2 * theImage.resolutionWidth / scanlineDiff);
}

void TeamMarkerPerceptor::update(TeamMarkerSpots& teamMarkerSpots)
{
  DECLARE_DEBUG_DRAWING("module:TeamMarkerPerceptor:grid", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:TeamMarkerPerceptor:polygon", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:TeamMarkerPerceptor:rejected", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:TeamMarkerPerceptor:ellipse", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:TeamMarkerPerceptor:bb", "drawingOnImage");
  MODIFY("parameters:TeamMarkerPerceptor", params);

  teamMarkerSpots.teamMarkers.clear();
  extractTeamMarkers(teamMarkerSpots);
}

void TeamMarkerPerceptor::extractTeamMarkers(TeamMarkerSpots& teamMarkerSpots)
{
  typedef const RegionPercept::Region* R;

  unsigned start = SystemCall::getRealSystemTime();
  for(R region = theRegionPercept.regions, end = region + theRegionPercept.regionsCounter; region < end; ++region)
  {
    switch(region->color)
    {
    case ColorClasses::blue:
    case ColorClasses::robotBlue:
    case ColorClasses::red:
    {
      unsigned duration = SystemCall::getRealSystemTime() - start;
      if(duration > params.maxDuration)
      {
        DEBUG_RESPONSE("module:TeamMarkerPerceptor:time",
        {
          OUTPUT_WARNING("SKIPPED team marker perception (duration: " << duration << " ms)");
        });
        return;
      }

      TeamMarkerSpots::TeamMarkerSpot teamMarker;
      teamMarker.color = region->color;

      ABORTABLE_STEP(regionCheck(region), *region->childs[0], ColorClasses::red)
      ABORTABLE_STEP(regionGrowing(teamMarker), approxCog, ColorClasses::green)
      ABORTABLE_STEP(scanlineRegionArea(teamMarker), approxCog, ColorClasses::yellow)
      ABORTABLE_STEP(scanlineRegionPrincipalAxis(teamMarker), approxCog, ColorClasses::orange)

      polygonDebugDrawing(teamMarker);
      teamMarkerSpots.teamMarkers.push_back(teamMarker);

      break;
    }
    default:
      break;
    }
  }
}

bool TeamMarkerPerceptor::regionCheck(const RegionPercept::Region* region)
{
  if(region->size < params.minRegionSize)
    return false;
  if((region->max_y - region->min_y) > params.maxHeight)
    return false;

  if(region->childs.size() < 2)
    return false;
  regionLength = float(region->childs[region->childs.size() - 1]->x - region->childs[0]->x);
  ASSERT(regionLength > 0.0);
  regionHeight = float(region->max_y - region->min_y);
  ASSERT(regionHeight > 0.0);

  const int m00 = region->calcMoment00();
  if(m00 == 0)
    return false;

  approxCog = Vector2<int>(region->calcMoment10() / m00, region->calcMoment01() / m00);
  if(!POINT_IS_WITHIN_IMAGE(approxCog, theImage))
    return false;

  return true;
}

bool TeamMarkerPerceptor::regionGrowing(TeamMarkerSpots::TeamMarkerSpot& teamMarker)
{
  above.clear();
  below.clear();

  ASSERT_POINT_WITHIN_IMAGE(approxCog, theImage);
  cbStart = theImage.image[approxCog.y][approxCog.x].cb;
  crStart = theImage.image[approxCog.y][approxCog.x].cr;

  Vector2<int> current = approxCog;
  scanDistance(Vector2<int>(-1, 0), approxCog, current);
  if(!POINT_IS_WITHIN_IMAGE(current, theImage))
    return false;
  ASSERT_COORDINATES_WITHIN_IMAGE(current.x, current.y, theImage);
  teamMarker.minX = current.x;

  current = approxCog;
  scanDistance(Vector2<int>(1, 0), approxCog, current);

  if(!POINT_IS_WITHIN_IMAGE(current, theImage))
    return false;
  ASSERT_COORDINATES_WITHIN_IMAGE(current.x, current.y, theImage);
  teamMarker.maxX = current.x;

  if(float(teamMarker.maxX - teamMarker.minX) > params.clippingWidthScale * regionLength)
    return false;

  int xLength = teamMarker.maxX - teamMarker.minX;
  if(xLength <= 3 || xLength > theImage.resolutionWidth / 3)
    return false;

  teamMarker.minY = approxCog.y;
  teamMarker.maxY = approxCog.y;
  int outsideImage = 0;

  for(int x = teamMarker.minX + 1; x < teamMarker.maxX; x += scanlineDiff)
  {
    int yUp;
    int yDown;

    Vector2<int> start(x, approxCog.y);
    current = start;
    scanDistance(Vector2<int>(0, -1), start, current, &above);
    if(!POINT_IS_WITHIN_IMAGE(current, theImage))
      outsideImage++;
    if(current.y < teamMarker.minY)
      teamMarker.minY = current.y;
    yUp = current.y;

    start = Vector2<int>(x, approxCog.y);
    current = start;
    scanDistance(Vector2<int>(0, 1), start, current, &below);
    if(!POINT_IS_WITHIN_IMAGE(current, theImage))
      outsideImage++;
    if(current.y > teamMarker.maxY)
      teamMarker.maxY = current.y;
    yDown = current.y;

    if(float(yDown - yUp) > params.clippingHeightScale * regionHeight)
      return false;
  }

  if(float(outsideImage) / float(above.size()) > params.maxOutsideImageRatio)
    return false;

  const int yLength = teamMarker.maxY - teamMarker.minY;
  teamMarker.standing = xLength > yLength;

  const float height = teamMarker.standing ? (float)yLength : (float)xLength;
  width = teamMarker.standing ? (float)xLength : (float)yLength;
  const float sideLengthRatio = width / height;
  if(sideLengthRatio < params.sideLengthRatio.x || sideLengthRatio > params.sideLengthRatio.y)
    return false;

  return true;
}

void TeamMarkerPerceptor::scanDistance(Vector2<int> dir, const Vector2<int>& start,
                                       Vector2<int>& current, TeamMarkerPerceptor::Polygon* p)
{
  ASSERT(abs(dir.x) == 1 || abs(dir.y) == 1);
  current = start;
  Vector2<int> tempEnd = start;
  bool unsure = false;
  int y = params.maxYOnScanline, cr = crStart, cb = cbStart;
  while(true)
  {
    if(abs(cr - crStart) >= params.teamMarkerCrDiff || abs(cb - cbStart) >= params.teamMarkerCbDiff || y > params.maxYOnScanline)
    {
      if(unsure)
      {
        current += tempEnd;
        current /= 2;
        break;
      }
      else
      {
        unsure = true;
        tempEnd = current;
      }
    }
    current += dir;
    if(!POINT_IS_WITHIN_IMAGE(current, theImage))
      break; // current points to a pixel outside the image!
    ASSERT_COORDINATES_WITHIN_IMAGE(current.x, current.y, theImage);
    y = theImage.image[(size_t) current.y][(size_t) current.x].y;
    cr = theImage.image[(size_t) current.y][(size_t) current.x].cr;
    cb = theImage.image[(size_t) current.y][(size_t) current.x].cb;
  }
  if(p)
  {
    p->push_back(current);
    LINE("module:TeamMarkerPerceptor:grid", start.x, start.y, current.x, current.y,
         1, Drawings::ps_solid, ColorClasses::black);
  }
}

bool TeamMarkerPerceptor::scanlineRegionArea(TeamMarkerSpots::TeamMarkerSpot& teamMarker)
{
  scanlineSegments.clear();
  for(Vertex va = above.begin(), vb = below.begin(); va != above.end(); va++, vb++)
    scanlineSegments.push_back(std::make_pair(*va, *vb));

  int area = 0;
  if(scanlineSegments.size() == 1)
    area = scanlineSegments.begin()->second.y - scanlineSegments.begin()->first.y;
  else if(scanlineSegments.size() > 1)
  {
    std::vector<ScanlineSegment>::const_iterator current = scanlineSegments.begin();
    std::vector<ScanlineSegment>::const_iterator next = scanlineSegments.begin();
    next++;
    const int stepSize = next->first.x - current->first.x;
    for(; next != scanlineSegments.end(); current++, next++)
      area += (((current->second.y + next->second.y) / 2) - ((current->first.y + next->first.y) / 2)) * stepSize;
  }

  teamMarker.area = area;
  return teamMarker.area > params.minTeamMarkerArea;
}

#ifndef GAUSS_SUM
#define GAUSS_SUM(x) ( x * ( x + 1 ) / 2 )
#endif
#ifndef GAUSS_SUM2
#define GAUSS_SUM2(x) ( x * ( x + 1 ) * ( 2 * x + 1 ) / 6 )
#endif

bool TeamMarkerPerceptor::scanlineRegionPrincipalAxis(TeamMarkerSpots::TeamMarkerSpot& teamMarker)
{
  if(scanlineSegments.size() < 2)
    return false;

  int m00 = 0, m10 = 0, m01 = 0;
  for(std::vector<ScanlineSegment>::iterator s = scanlineSegments.begin(); s != scanlineSegments.end(); s++)
  {
    const int length = s->second.y - s->first.y;
    m00 += length;
    m10 += s->first.x * length;
    m01 += (s->second.y * (s->second.y - 1) - s->first.y * (s->first.y - 1)) / 2;
  }

  int comX = m10 / m00;
  int comY = m01 / m00;

  teamMarker.centerOfGravity = Vector2<int>(comX, comY);

  int m11 = 0, m20 = 0, m02 = 0;
  for(std::vector<ScanlineSegment>::iterator s = scanlineSegments.begin(); s != scanlineSegments.end(); s++)
  {
    const int length = s->second.y - s->first.y;
    const int translatedX = s->first.x - comX;
    const int ySum = GAUSS_SUM(s->second.y) - GAUSS_SUM(s->first.y);
    m11 += s->first.x * ySum - length * s->first.x * comY - comX * ySum + length * comX * comY;
    m20 += length * translatedX * translatedX;
    m02 += GAUSS_SUM2(s->second.y) - GAUSS_SUM2(s->first.y) - 2 * comY * (GAUSS_SUM(s->second.y) - GAUSS_SUM(s->first.y)) + length * comY * comY;
  }

  float denominator = float(m02 - m20), nominator = -2.0f * m11;
  if(denominator != 0.0f || nominator != 0.0f)
    orientation = atan2(nominator, denominator) / 2.0f;
  else
    orientation = 0.0f;

  const float c = cosf(orientation);
  const float s = sinf(orientation);
  const float c2 = c * c, s2 = s * s, cs = c * s;

  register const float commonSummand = 2.0f * cs * m11;
  const float ev1 = c2 * m20 + commonSummand + s2 * m02;
  const float ev2 = s2 * m20 - commonSummand + c2 * m02;

  COMPLEX_DRAWING("module:TeamMarkerPerceptor:ellipse",
  {
    ELLIPSE("module:TeamMarkerPerceptor:ellipse", teamMarker.centerOfGravity, 0.167f * sqrt(ev1), 0.167f * sqrt(ev2), orientation,
    2, Drawings::ps_solid, ColorRGBA(255, 100, 100, 100), Drawings::bs_solid, ColorRGBA(255, 100, 100, 100));
  });

  if(ev1 < ev2 && orientation > 0.0f)
    orientation -= pi;

  const int axisWidth = (int) width * 5 / 12;
  teamMarker.area = m00;
  teamMarker.left = teamMarker.centerOfGravity - Vector2<int>(0, axisWidth).rotate(orientation);
  teamMarker.right = teamMarker.centerOfGravity + Vector2<int>(0, axisWidth).rotate(orientation);

  return true;
}

void TeamMarkerPerceptor::polygonDebugDrawing(TeamMarkerSpots::TeamMarkerSpot& teamMarker)
{
  COMPLEX_DRAWING("module:TeamMarkerPerceptor:polygon",
  {
    polygon.clear();

    ASSERT(below.size() == above.size());
    if(below.size() == 0)
      return;

    Vertex u = below.begin();
    polygon.push_back(*u);
    if(below.size() == 1)
    {
      polygon.push_back(*(above.rbegin()));
      return;
    }
    Vertex v = below.begin();
    v++;
    if(below.size() == 2)
    {
      polygon.push_back(*v);
      VertexR x = above.rbegin();
      polygon.push_back(*x);
      polygon.push_back(*(++x));
      return;
    }
    Vertex w = below.begin();
    w++; w++;

    for(; w != below.end(); u++, v++, w++)
    {
      if((w->y - v->y) != (v->y - u->y)) // omits collinear pixels
        polygon.push_back(*v);
    }
    polygon.push_back(*v);
    ASSERT(u != v); ASSERT(v != w); ASSERT(u != w);
    ASSERT(++u == v); ASSERT(++v == w); ASSERT(w == below.end());

    VertexR x = above.rbegin();
    VertexR y = above.rbegin();
    y++;
    VertexR z = above.rbegin();
    z++; z++;

    polygon.push_back(*x);
    for(; z != above.rend(); x++, y++, z++)
    {
      if((z->y - y->y) != (y->y - x->y)) // omits collinear pixels
        polygon.push_back(*y);
    }
    polygon.push_back(*y);
    ASSERT(x != y); ASSERT(y != z); ASSERT(x != z);
    ASSERT(++x == y); ASSERT(++y == z); ASSERT(z == above.rend());

    Vertex prev = polygon.begin();
    for(Vertex v = polygon.begin(); v != polygon.end(); v++)
    {
      DOT("module:TeamMarkerPerceptor:polygon", v->x, v->y, ColorClasses::red, ColorClasses::red);
      LINE("module:TeamMarkerPerceptor:polygon", prev->x, prev->y, v->x, v->y,
      1, Drawings::ps_solid, ColorClasses::white);
      prev = v;
    }
    LINE("module:TeamMarkerPerceptor:polygon", prev->x, prev->y, polygon.begin()->x, polygon.begin()->y,
    1, Drawings::ps_solid, ColorClasses::white);
  });

  COMPLEX_DRAWING("module:TeamMarkerPerceptor:bb",
  {
    Vector2<> cog((float)teamMarker.centerOfGravity.x, (float)teamMarker.centerOfGravity.y);
    int minX = 1000, maxX = -1, minY = 1000, maxY = -1;
    Matrix2x2<> rotation(cosf(orientation), -sinf(orientation), sinf(orientation), cosf(orientation));
    for(Polygon::iterator it = polygon.begin(); it != polygon.end(); it++)
    {
      Vector2<> rotatedF = rotation * (Vector2<>((float)it->x, (float)it->y) - cog);
      Vector2<int> rotated((int)rotatedF.x, (int)rotatedF.y);
      if(rotated.x < minX)
        minX = rotated.x;
      if(rotated.x > maxX)
        maxX = rotated.x;
      if(rotated.y < minY)
        minY = rotated.y;
      if(rotated.y > maxY)
        maxY = rotated.y;
    }
    Vector2<> rotatedLT((float)minX, (float)minY);
    Vector2<> rotatedLB((float)minX, (float)maxY);
    Vector2<> rotatedRT((float)maxX, (float)minY);
    Vector2<> rotatedRB((float)maxX, (float)maxY);
    Matrix2x2<> rotationInv = rotation.transpose();
    Vector2<> lt = rotationInv* rotatedLT + cog;
    Vector2<> lb = rotationInv* rotatedLB + cog;
    Vector2<> rt = rotationInv* rotatedRT + cog;
    Vector2<> rb = rotationInv* rotatedRB + cog;
    QUADRANGLE("module:TeamMarkerPerceptor:bb",
    (int)lt.x, (int)lt.y, (int)lb.x, (int)lb.y, (int)rb.x, (int)rb.y, (int)rt.x, (int)rt.y,
    2, Drawings::ps_solid, ColorClasses::black);
  });
}

