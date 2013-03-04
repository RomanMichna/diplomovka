/**
* @file Controller/LogPlayer.cpp
*
* Implementation of class LogPlayer
*
* @author Martin Lötzsch
*/

#include <QImage>
#include "LogPlayer.h"
#include "Representations/Perception/JPEGImage.h"
#include "Representations/Infrastructure/Legacy/FrameInfo_83e22.h"
#include "Representations/Infrastructure/Legacy/JointData_83e22.h"
#include "Platform/SystemCall.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"

//If we don't have information on time per frame, assume we're running at 30hz
const int FALLBACK_TIME_PER_FRAME = 1000 / 30;

LogPlayer::LogPlayer(MessageQueue& targetQueue) :
  targetQueue(targetQueue)
{
  init();
}

void LogPlayer::init()
{
  clear();
  stop();
  numberOfFrames = 0;
  numberOfMessagesWithinCompleteFrames = 0;
  replayOffset = 0;
  lastRealTimestamp = SystemCall::getCurrentSystemTime();
  state = initial;
  loop = true; //default: loop enabled
}

bool LogPlayer::open(const char* fileName)
{
  InBinaryFile file(fileName);
  if(file.exists())
  {
    clear();
    file >> *this;
    stop();
    countFrames();
    return true;
  }
  return false;
}

void LogPlayer::play()
{
  if(state != playing)
    lastRealTimestamp = SystemCall::getCurrentSystemTime();
  state = playing;
}

void LogPlayer::stop()
{
  if(state == recording)
  {
    recordStop();
    return;
  }
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
  lastImageFrameNumber = -1;
  currentLogTimestamp = 0;
  lastRealTimestamp = 0;
}

void LogPlayer::pause()
{
  if(getNumberOfMessages() == 0)
    state = initial;
  else
    state = paused;
}

void LogPlayer::stepBackward()
{
  pause();
  if(state == paused && currentFrameNumber > 0)
  {
    do
      queue.setSelectedMessageForReading(--currentMessageNumber);
    while(currentMessageNumber > 0 && queue.getMessageID() != idProcessFinished);
    --currentFrameNumber;
    stepRepeat();
  }
}

void LogPlayer::stepImageBackward()
{
  pause();
  if(state == paused && currentFrameNumber > 0)
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepBackward();
    while(lastImageFrameNumber == this->lastImageFrameNumber && currentFrameNumber > 0);
  }
}

void LogPlayer::stepForward()
{
  pause();
  if(state == paused && currentFrameNumber < numberOfFrames - 1 && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1)
  {
    do
    {
      copyMessage(++currentMessageNumber, targetQueue);
      unsigned tmpTime = getTimeInformation();
      if(tmpTime != 0)
        currentLogTimestamp = tmpTime;
      if(queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage)
        lastImageFrameNumber = currentFrameNumber;
    }
    while(queue.getMessageID() != idProcessFinished);
    ++currentFrameNumber;
  }
}

void LogPlayer::stepImageForward()
{
  pause();
  if(state == paused && currentFrameNumber < numberOfFrames - 1)
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepForward();
    while(lastImageFrameNumber == this->lastImageFrameNumber && currentFrameNumber < numberOfFrames - 1);
  }
}

void LogPlayer::stepRepeat()
{
  pause();
  if(state == paused && currentFrameNumber >= 0)
  {
    do
      queue.setSelectedMessageForReading(--currentMessageNumber);
    while(currentMessageNumber > 0 && queue.getMessageID() != idProcessFinished);
    --currentFrameNumber;
    stepForward();
  }
}

void LogPlayer::gotoFrame(int frame)
{
  pause();
  if(state == paused && frame < numberOfFrames)
  {
    currentFrameNumber = -1;
    currentMessageNumber = -1;
    currentLogTimestamp = logBeginTimestamp;
    while(++currentMessageNumber < getNumberOfMessages() && frame > currentFrameNumber + 1)
    {
      queue.setSelectedMessageForReading(currentMessageNumber);
      unsigned tmpTime = getTimeInformation();
      if(tmpTime != 0)
        currentLogTimestamp = tmpTime;
      if(queue.getMessageID() == idProcessFinished)
        ++currentFrameNumber;
    }
    stepForward();
  }
}

bool LogPlayer::save(const char* fileName)
{
  if(state == recording)
    recordStop();

  if(!getNumberOfMessages())
    return false;

  OutBinaryFile file(fileName);
  if(file.exists())
  {
    file << *this;
    return true;
  }
  return false;
}

bool LogPlayer::saveImages(const bool raw, const char* fileName)
{
  int i = 0;
  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    Image image;
    if(queue.getMessageID() == idImage)
    {
      in.bin >> image;
    }
    else if(queue.getMessageID() == idJPEGImage)
    {
      JPEGImage jpegImage;
      in.bin >> jpegImage;
      jpegImage.toImage(image);
    }
    else
      continue;

    if(!saveImage(image, fileName, i++, !raw))
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

void LogPlayer::recordStart()
{
  state = recording;
}

void LogPlayer::recordStop()
{
  while(getNumberOfMessages() > numberOfMessagesWithinCompleteFrames)
    removeLastMessage();
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
}


void LogPlayer::setLoop(bool loop_)
{
  loop = loop_;
}

void LogPlayer::handleMessage(InMessage& message)
{
  if(state == recording)
  {
    message >> *this;
    if(message.getMessageID() == idProcessFinished)
    {
      numberOfMessagesWithinCompleteFrames = getNumberOfMessages();
      ++numberOfFrames;
    }
  }
}

bool LogPlayer::replay(bool realtime)
{
  doTick();
  if(state == playing)
  {
    if(currentFrameNumber < numberOfFrames - 1)
    {
      bool firstRun = true;
      unsigned timeToReplay = (SystemCall::getCurrentSystemTime() - lastRealTimestamp) - replayOffset;
      unsigned targetTimestamp = currentLogTimestamp + timeToReplay;
      while((!realtime && firstRun)
            || (realtime && currentLogTimestamp < targetTimestamp && currentFrameNumber < numberOfFrames - 1 && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1))
      {
        firstRun = false;
        do
        {
          copyMessage(++currentMessageNumber, targetQueue);
          unsigned tmpTime = getTimeInformation();
          if(tmpTime != 0)
            currentLogTimestamp = tmpTime;
          if(queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage)
            lastImageFrameNumber = currentFrameNumber;
        }
        while(queue.getMessageID() != idProcessFinished && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1);
        ++currentFrameNumber;
      }
      replayOffset = currentLogTimestamp - targetTimestamp;
      lastRealTimestamp = SystemCall::getCurrentSystemTime();
      if(currentFrameNumber == numberOfFrames - 1)
      {
        if(loop)  //restart in loop mode
        {
          gotoFrame(0);
          play();
        }
        else
          stop();
      }
      return true;
    }
    else
    {
      if(loop)  //restart in loop mode
      {
        gotoFrame(0);
        play();
      }
      else
        stop();
    }
  }
  return false;
}

void LogPlayer::keep(MessageID* messageIDs)
{
  LogPlayer temp((MessageQueue&) *this);
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while(*m)
    {
      if(temp.queue.getMessageID() == *m ||
         temp.queue.getMessageID() == idProcessBegin ||
         temp.queue.getMessageID() == idProcessFinished)
      {
        temp.copyMessage(temp.currentMessageNumber, *this);
        break;
      }
      ++m;
    }
  }
}

void LogPlayer::remove(MessageID* messageIDs)
{
  LogPlayer temp((MessageQueue&) *this);
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while(*m)
    {
      if(temp.queue.getMessageID() == *m)
        break;
      ++m;
    }
    if(!*m)
      temp.copyMessage(temp.currentMessageNumber, *this);
  }
}

void LogPlayer::statistics(int frequency[numOfDataMessageIDs])
{
  for(int i = 0; i < numOfDataMessageIDs; ++i)
    frequency[i] = 0;

  if(getNumberOfMessages() > 0)
  {
    int current = queue.getSelectedMessageForReading();
    for(int i = 0; i < getNumberOfMessages(); ++i)
    {
      queue.setSelectedMessageForReading(i);
      ASSERT(queue.getMessageID() < numOfDataMessageIDs);
      ++frequency[queue.getMessageID()];
    }
    queue.setSelectedMessageForReading(current);
  }
}

void LogPlayer::countFrames()
{
  fallbackTimings = false;
  numberOfFrames = 0;
  unsigned lastTime = 0;
  logBeginTimestamp = 0;
  for(int i = 0; i < getNumberOfMessages(); ++i)
  {
    queue.setSelectedMessageForReading(i);
    unsigned tmpTime = getTimeInformation();
    lastTime = tmpTime != 0 ? tmpTime : lastTime;
    if(logBeginTimestamp == 0 && lastTime != 0)
      logBeginTimestamp = lastTime;
    if(queue.getMessageID() == idProcessFinished)
    {
      ++numberOfFrames;
      numberOfMessagesWithinCompleteFrames = i + 1;
    }
  }
  if(lastTime == 0 || lastTime == logBeginTimestamp)
  {
    fallbackTimings = true;
    logBeginTimestamp = 0;
    logEndTimestamp = FALLBACK_TIME_PER_FRAME * numberOfFrames;
  }
  else
    logEndTimestamp = lastTime;
}

unsigned LogPlayer::getTimeInformation()
{
  if(fallbackTimings)
  {
    return currentFrameNumber == -1 ? 0 : FALLBACK_TIME_PER_FRAME * currentFrameNumber;
  }
  if(queue.getMessageID() == idFrameInfo)
  {
    FrameInfo_83e22 fi; // hack, usually we should read a FrameInfo object
    in.bin >> fi;
    queue.resetReadPosition();
    return fi.time;
  }
  else if(queue.getMessageID() == idImage)
  {
    Image img;
    in.bin >> img;
    queue.resetReadPosition();
    return img.timeStamp;
  }
  else if(queue.getMessageID() == idJPEGImage)
  {
    JPEGImage img;
    in.bin >> img;
    queue.resetReadPosition();
    return img.timeStamp;
  }
  else if(queue.getMessageID() == idJointData)
  {
    JointData_83e22 jd; // hack, usually we should read a JointData object
    in.bin >> jd;
    queue.resetReadPosition();
    return jd.timeStamp;
  }
  return 0;
}

void LogPlayer::setTime(int time)
{
  ASSERT(time >= 0);
  LogPlayerState stateBck = state;
  if(fallbackTimings)
    gotoFrame(time / FALLBACK_TIME_PER_FRAME);
  else
  {
    unsigned targetTime = (unsigned)time + logBeginTimestamp;
    if(targetTime < currentLogTimestamp)
    {
      currentFrameNumber = -1;
      currentMessageNumber = -1;
      currentLogTimestamp = logBeginTimestamp;
    }
    while(++currentMessageNumber < getNumberOfMessages() && targetTime > currentLogTimestamp)
    {
      queue.setSelectedMessageForReading(currentMessageNumber);
      unsigned tmpTime = getTimeInformation();
      if(tmpTime != 0)
        currentLogTimestamp = tmpTime;
      if(queue.getMessageID() == idProcessFinished)
        ++currentFrameNumber;
    }
    stepForward();
  }
  state = stateBck;
}

int LogPlayer::getTime()
{
  return (currentLogTimestamp - logBeginTimestamp);
}

int LogPlayer::getLength()
{
  return (logEndTimestamp - logBeginTimestamp);
}

void LogPlayer::setPause(bool pause)
{
  if(pause)
    this->pause();
  else
    this->play();
}

std::string LogPlayer::expandImageFileName(const char* fileName, int imageNumber)
{
  std::string name(fileName);
  std::string ext(".bmp");
  std::string::size_type p = (int) name.rfind('.');
  if((int) p > (int) name.find_last_of("\\/"))
  {
    ext = name.substr(p);
    name = name.substr(0, p);
  }
  
  if(imageNumber >= 0)
  {
    char num[12];
    sprintf(num, "%03d", imageNumber);
    name = name + "_" + num + ext;
  }
  else
    name += ext;
  
  for(unsigned i = 0; i < name.size(); ++i)
    if(name[i] == '\\')
      name[i] = '/';
  if(name[0] != '/' && (name.size() < 2 || name[1] != ':'))
    name = File::getBHDir() + std::string("/Config/") + name;
  return name;
}

bool LogPlayer::saveImage(const Image& image, const char* fileName, int imageNumber, bool YUV2RGB)
{
  std::string name = expandImageFileName(fileName, imageNumber);
  const Image* r = &image;
  if(YUV2RGB)
  {
    r = new Image;
    const_cast<Image*>(r)->convertFromYCbCrToRGB(image);
  }
  QImage img(image.resolutionWidth, image.resolutionHeight, QImage::Format_RGB888);
  for(int y = 0; y < image.resolutionHeight; ++y)
  {
    const Image::Pixel* pSrc = &r->image[y][0];
    unsigned char* p = img.scanLine(y),
                 * pEnd = p + 3 * image.resolutionWidth;
    if(YUV2RGB)
      while(p != pEnd)
      {
        *p++ = pSrc->r;
        *p++ = pSrc->g;
        *p++ = pSrc->b;
        ++pSrc;
      }
    else
      while(p != pEnd)
      {
        *p++ = pSrc->y;
        *p++ = pSrc->cb;
        *p++ = pSrc->cr;
        ++pSrc;
      }
  }
  if(YUV2RGB)
    delete r;
  return img.save(name.c_str());
}
