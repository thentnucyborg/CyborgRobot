/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2015 Adept Technology, Inc.
Copyright (C) 2016-2018 Omron Adept Technologies, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "ArExport.h"

#include "ariaOSDef.h"
#include "ArIrrfDevice.h"
#include "ArCommands.h"

AREXPORT ArIrrfDevice::ArIrrfDevice(size_t currentBufferSize,
                             size_t cumulativeBufferSize, const char *name) :
  ArRangeDevice(currentBufferSize, cumulativeBufferSize, name, 5000),
  myPacketHandler(this, &ArIrrfDevice::packetHandler)
{
  int i;
  myRobot = NULL;
  // These numbers haven't been proven, yet.  It will take experimentation to maximize the best results
  myCumulativeMaxRange = 10000;
  myMaxRange = 5000;
  myFilterFarDist = 7000;
  myFilterNearDist = 50;

  myPacketHandler.setName("ArIrrfDevice");
  // The 91 readings start at -81 degrees, and move in 1.8 degree steps
  myRawReadings = new std::list<ArSensorReading *>;
  for(i=0;i<91;i++)
    myRawReadings->push_back(new ArSensorReading(0, 0, (1.8*i - 81)));
}

AREXPORT ArIrrfDevice::~ArIrrfDevice()
{
  if (myRobot != NULL)
  {
    myRobot->remPacketHandler(&myPacketHandler);
    myRobot->remRangeDevice(this);
  }
}

AREXPORT void ArIrrfDevice::setRobot(ArRobot *robot)
{
  myRobot = robot;
  if (myRobot != NULL)
    myRobot->addPacketHandler(&myPacketHandler, ArListPos::LAST);
}

AREXPORT void ArIrrfDevice::processReadings(void)
{
  int i;
  double rx, ry, nx, ny, dx, dy, dist;
  ArSensorReading *reading;
  std::list<ArSensorReading *>::iterator rawIt;
  std::list<ArPoseWithTime *> *readingList;
  std::list<ArPoseWithTime *>::iterator readIt;
  lockDevice();

  rx = myRobot->getX();
  ry = myRobot->getY();

  i=0;
  for (rawIt = myRawReadings->begin();rawIt != myRawReadings->end();rawIt++)
  {
    reading = (*rawIt);
    nx = reading->getX();
    ny = reading->getY();
    dx = nx - rx;
    dy = nx - ry;
    dist = (dx*dx) + (dy*dy);
    if (!reading->isNew(myRobot->getCounter()))
      continue;

    if (dist < (myMaxRange * myMaxRange))
      myCurrentBuffer.addReading(nx, ny);

    if (dist < (myCumulativeMaxRange * myCumulativeMaxRange))
    {
      myCumulativeBuffer.beginInvalidationSweep();
      readingList = myCumulativeBuffer.getBuffer();

      if (readingList != NULL)
      {
        for (readIt = readingList->begin(); 
	     readIt != readingList->end();
	     readIt++)
        {
          dx = (*readIt)->getX() - nx;
          dy = (*readIt)->getY() - ny;
          if ((dx*dx + dy*dy) < (myFilterNearDist * myFilterNearDist))
            myCumulativeBuffer.invalidateReading(readIt);
        }
      }
      myCumulativeBuffer.endInvalidationSweep();
      myCumulativeBuffer.addReading(nx, ny);
    }
  }
 
  readingList = myCumulativeBuffer.getBuffer();

  rx = myRobot->getX();
  ry = myRobot->getY();

  myCumulativeBuffer.beginInvalidationSweep();
  if (readingList != NULL)
  {
    for (readIt = readingList->begin(); readIt != readingList->end();readIt++)
    {
      dx = (*readIt)->getX() - rx;
      dy = (*readIt)->getY() - ry;
      if ((dx*dx + dy*dy) > (myFilterFarDist * myFilterFarDist))
        myCumulativeBuffer.invalidateReading(readIt);
    }
  }
  myCumulativeBuffer.endInvalidationSweep();

  unlockDevice();
}

/**
  This is the packet handler for the PB9 data, which is sent via the micro
  controller, to the client.  This will read the data from the packets,
  and then call processReadings to filter add the data to the current and
  cumulative buffers.
*/
AREXPORT bool ArIrrfDevice::packetHandler(ArRobotPacket *packet)
{
  int portNum, i, dist, packetCounter;
  double conv;
  ArTransform packetTrans;
  std::list<ArSensorReading *>::iterator it;
  ArSensorReading *reading;
  ArPose pose;
  ArTransform encoderTrans;
  ArPose encoderPose;

  pose = myRobot->getPose();
  conv = 2.88;

  packetTrans.setTransform(pose);
  packetCounter = myRobot->getCounter();

  if (packet->getID() != 0x10)
    return false;

  // Which Aux port the IRRF is connected to
  portNum = packet->bufToByte2();
  encoderTrans = myRobot->getEncoderTransform();
  encoderPose = encoderTrans.doInvTransform(pose);

  i = 0;
  for (i=0, it = myRawReadings->begin();it != myRawReadings->end();it++, i++)
  {
    reading = (*it);
    dist = (int) ((packet->bufToUByte2()) / conv);
    reading->newData(dist, pose, encoderPose, packetTrans, packetCounter, packet->getTimeReceived());
  }

  myLastReading.setToNow();

  processReadings();

  return true;
}
