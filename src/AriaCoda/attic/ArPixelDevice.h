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
#ifndef ARPIXELDEVICE_H
#define ARPIXELDEVICE_H

#include <stdio.h>
#include "Aria.h"
/*!
  @class ArPixelDevice.
  @brief Holds data from a sensor that provides data arranged in a 2d array.

  Subclasses are used for specific sensor implementations, such
  as the ArFocusPixelDevice for the Focus Robotics nDepth 
  stereocamera.  

  The data are arranged in an x,y grid, with the origin being
  in the upper left corner, from the perspective of the robot.
  It's in a row-major format.

  The Field of View (FOV) is assumed to be centered with zero
  being the center of the area, such that the permitted x angles
  are between ((-x_fov/2) to (+x_fov/2)) and 
  ((-y_fov/2) to (+y_fov/2)).  Negative angles are to the lower
  left of the grid from the perspective of the robot.

  @param x_size dimension of data grid in x direction

  @param y_size dimension of data grid in y direction

  @param x_fov Field of View of sensor in X direction (angle in degrees)

  @param y_fov Field of View of sensor in Y direction (angle in degrees)

  @param name the name of this device
*/
template<class DataObject>
class ArPixelDevice
{
 public:
  /// Base Constructor
  AREXPORT ArPixelDevice(int x_size, int y_size, double x_fov, double y_fov,
		      const char *name)
  {
    myDeviceMutex.setLogName("ArPixelDevice::myDeviceMutex");
    myXSize = x_size;
    myYSize = y_size;
    myXFOV = x_fov;
    myYFOV = y_fov;
    myName = name;

    mySensorData = NULL;
    
    if (!allocateSensorDataMemory()) 
    {
      ArLog::log(ArLog::Terse, "Failed to allocate memory for ArPixelDevice %s", getName());
    }
    else
    {
      ArLog::log(ArLog::Verbose, "Allocated memory for ArPixelDevice %s", getName());
    }

    if (!allocateSensorXYZMemory()) 
    {
      ArLog::log(ArLog::Terse, "Failed to allocate XYZ memory for ArPixelDevice %s", getName());
    }
    else
    {
      ArLog::log(ArLog::Verbose, "Allocated XYZ memory for ArPixelDevice %s", getName());
    }
  }
  /// Base destructor
  AREXPORT virtual ~ArPixelDevice()
  {
    if (mySensorData != NULL)
    {
      for (int i=0; i < myXSize; i++)
      {
	for (int j=0; j < myYSize; j++)
	{
	  delete mySensorData[i][j];
	}
	delete [] (mySensorData[i]);
      }
      delete [] (mySensorData);
    }
    if (mySensorXYZ != NULL)
    {
      for (int i=0; i < myXSize; i++)
      {
	for (int j=0; j < myYSize; j++)
	{
	  delete [] mySensorXYZ[i][j];
	}
	delete [] (mySensorXYZ[i]);
      }
      delete [] (mySensorXYZ);
    }

  }
  /// Get the value of the sensor at the (x,y) coords
  DataObject *getSensorData(int x, int y)
  {
    if ((x >= 0) && (x < myXSize) && (y >= 0) && (y < myYSize)) 
    {
      return mySensorData[x][y];
    }
    else 
    {
      return NULL;
    }
  }
  /// Get the xyz array of the sensor at the (x,y) pizels.
  DataObject* getSensorXYZ(int x, int y)
  {
    if ((x >= 0) && (x < myXSize) && (y >= 0) && (y < myYSize)) 
    {
      return mySensorXYZ[x][y];
    }
    else 
    {
      return NULL;
    }
  }
  /// Get the dimension of the grid in the x direction
  int getXDimension(void) { return myXSize; }
  /// Get the dimension of the grid in the y direction
  int getYDimension(void) { return myYSize; }
  /// Get the X direction Field of View, in degrees
  double getXFOV(void) { return myXFOV; }
  /// Get the Y direction Field of View, in degrees
  double getYFOV(void) { return myYFOV; }
  /// Get the name of the device
  const char *getName(void) { return myName.c_str(); }
  /// Gets the raw sensor data
  DataObject ***getRawSensorData(void) { return mySensorData; }
  /// Gets the raw XYZ data
  DataObject ***getRawSensorXYZ(void) { return mySensorXYZ; }
  /// Lock this device
  AREXPORT virtual int lockDevice() { return(myDeviceMutex.lock()); }
  /// Try to lock this device
  AREXPORT virtual int tryLockDevice() { return(myDeviceMutex.tryLock()); }
  /// Unlock this device
  AREXPORT virtual int unlockDevice() { return(myDeviceMutex.unlock()); }
protected:
  std::string myName;
  int myXSize;
  int myYSize;
  double myXFOV;
  double myYFOV;
  DataObject ***mySensorData;
  DataObject ***mySensorXYZ;
  ArMutex myDeviceMutex;

  bool allocateSensorDataMemory()
  {    
    if ((myXSize < 1) || (myYSize < 1)) 
    {
      ArLog::log(ArLog::Normal, "Bad array size for ArPixelDevice %s", getName());
      return false;
    }
    mySensorData = new DataObject**[myXSize];
    if (mySensorData == NULL)
    {
      ArLog::log(ArLog::Normal, "Cannot allocate memory for ArPixelDevice %s", getName());
      return false;
    }
    for (int i = 0; i < myXSize; i++)
    {
      if ((mySensorData[i] = new DataObject*[myYSize]) == NULL)
      {
	ArLog::log(ArLog::Normal, "Cannot allocate memory for ArPixelDevice %s", getName());
	return false;
      }
      for (int j = 0; j < myYSize; j++) 
      {
	mySensorData[i][j] = new DataObject;
      }      
    }
    return true;
  }

  bool allocateSensorXYZMemory()
  {    
    if ((myXSize < 1) || (myYSize < 1)) 
    {
      ArLog::log(ArLog::Normal, "Bad array size for ArPixelDevice %s", getName());
      return false;
    }
    mySensorXYZ = new DataObject**[myXSize];
    if (mySensorXYZ == NULL)
    {
      ArLog::log(ArLog::Normal, "Cannot allocate memory for ArPixelDevice %s", getName());
      return false;
    }
    for (int i = 0; i < myXSize; i++)
    {
      if ((mySensorXYZ[i] = new DataObject*[myYSize]) == NULL)
      {
	ArLog::log(ArLog::Normal, "Cannot allocate memory for ArPixelDevice %s", getName());
	return false;
      }
      for (int j = 0; j < myYSize; j++) 
      {
	mySensorXYZ[i][j] = new DataObject[3];
      }      
    }
    return true;
  }

};

#endif // ARPIXELDEVICE_H
