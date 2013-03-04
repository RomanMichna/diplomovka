#include "ImageGrid.h"

void ImageGrid::draw(const Image& image) const
{
  COMPLEX_DEBUG_IMAGE(grid, 
    int y(0);
    const LineInformation* p = lineInformation;
    while(y < cameraResolutionHeight)
    {
      for(int x = 0; x < cameraResolutionWidth; x += p->columnInterval)
      {

        Image::Pixel pixel = image.image[y][x];
        DEBUG_IMAGE_SET_PIXEL_YUV(grid, x, y, pixel.y, pixel.cb, pixel.cr);
        for(int xx = x + 1; xx < x + p->columnInterval; xx++)
        {
          DEBUG_IMAGE_SET_PIXEL_YUV(grid, xx, y, image.image[y][xx].y / 2, 127, 127);
        }
      }
      for(int yy = y + 1; yy < y + p->rowInterval && yy < cameraResolutionHeight; yy++)
      {
        for(int x = 0; x < cameraResolutionWidth; x++)
        {
          DEBUG_IMAGE_SET_PIXEL_YUV(grid, x, yy, image.image[yy][x].y / 2, 127, 127);
        }
      }
      y += p->rowInterval;
      p++;
    }
    SEND_DEBUG_IMAGE(grid);
  );
  COMPLEX_DEBUG_IMAGE(grid2, 
    int y(0);
    const LineInformation* p = lineInformation;
    while(y < cameraResolutionHeight)
    {
      for(int x = 0; x < cameraResolutionWidth; x += p->columnInterval)
      {

        Image::Pixel pixel = image.image[y][x];
        for(int yy = y; yy < y + p->rowInterval && yy < cameraResolutionHeight; ++yy)
        {
          for(int xx = x; xx < x + p->columnInterval && xx < cameraResolutionWidth; ++xx)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(grid2, xx, yy, pixel.y, pixel.cb, pixel.cr);
          }
        }
      }
      y += p->rowInterval;
      p++;
    }
    SEND_DEBUG_IMAGE(grid2);
  );
}

int ImageGrid::getNumOfSamples() const
{
  int numOfSamples = 0;
  for(int i = 0; i < numberOfLines; ++i)
  {
    numOfSamples += (cameraResolutionWidth - 1) / lineInformation[i].columnInterval + 1;
  }
  return numOfSamples;
}

Out& operator<<(Out& stream, const ImageGrid::LineInformation& value)
{
  return stream << value.rowInterval << value.columnInterval;
}

In& operator>>(In& stream, ImageGrid::LineInformation& value)
{
  return stream >> value.rowInterval >> value.columnInterval;
}
