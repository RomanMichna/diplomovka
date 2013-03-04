#include "RunLengthImage.h"
#include "Platform/BHAssert.h"

void RunLengthImage::toImage(Image& image, const ImageGrid& imageGrid) const
{
  image.timeStamp = this->timestamp;
  const Run* p = this->image;
  const ImageGrid::LineInformation* pGrid = imageGrid.lineInformation;
  int x(0), y(0);
  while(p->runLength != 0)
  {
    for(int yy = y; yy < y + pGrid->rowInterval; yy++)
    {
      for(int i = x; i < x + p->runLength; i++)
      {
        image.image[yy][i] = p->color;
      }
    }
    x += p->runLength;
    if(x >= cameraResolutionWidth)
    {
      ASSERT(x == cameraResolutionWidth); // runs should always build a complete line
      x = 0;
      y += pGrid->rowInterval;
      pGrid++;
      ASSERT(y <= cameraResolutionHeight);
    }
    p++;
  }
  ASSERT(p - this->image == numberOfRuns); // is the zero-marker really the last run?
  ASSERT(x == 0 && y == cameraResolutionHeight);
}

bool RunLengthImage::Run::operator<(const RunLengthImage::Run& other) const
{
  return y < other.y || (y == other.y && xStart < other.xStart);
}

Out& operator<<(Out& stream, const RunLengthImage::Run& run)
{
  ASSERT(false);
  return stream;
}

In& operator>>(In& stream, RunLengthImage::Run& run)
{
  ASSERT(false);
  return stream;
}

void RunLengthImage::draw(const ImageGrid& imageGrid) const
{
  COMPLEX_DEBUG_IMAGE(runLengthImage,
    toImage(runLengthImageImage, imageGrid);
    SEND_DEBUG_IMAGE(runLengthImage);
  );
}
