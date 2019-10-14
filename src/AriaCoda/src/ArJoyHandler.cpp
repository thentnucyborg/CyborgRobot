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
#include "ariaOSDef.h"
#include "ArExport.h"
#include "ArJoyHandler.h"
#include "ariaUtil.h"

/**
   @param useOSCal if this is set then the joystick will just rely on
   the OS to calibrate, otherwise it will keep track of center min and
   max and use those values for calibration
   
   @param useOld use the old linux interface to the joystick
**/

AREXPORT ArJoyHandler::ArJoyHandler(bool useOSCal, bool useOld)
{
  myInitialized = false;
  myUseOSCal = useOSCal;
  myUseOld = useOld;
  myHaveZ = false;
  myFirstData = true;
}

AREXPORT ArJoyHandler::~ArJoyHandler()
{
}


/**
   @param useOSCal if this is set then the joystick will just rely on
   the OS to calibrate, otherwise it will keep track of center min and
   max and use those values for calibration
**/

AREXPORT void ArJoyHandler::setUseOSCal(bool useOSCal)
{
  myUseOSCal = useOSCal;
}

/**
   @return if useOSCal is set then the joystick will just rely on
   the OS to calibrate, otherwise it will keep track of center min and
   max and use those values for calibration
**/

AREXPORT bool ArJoyHandler::getUseOSCal(void)
{
  return myUseOSCal;
}

/**
   Starts the calibration, which resets all the min and max variables as well
   as the center variables.
   @see endCal */
AREXPORT void ArJoyHandler::startCal(void)
{
  int x, y;
  getUnfiltered(&x, &y);
  myMaxX = x;
  myMinX = x;
  myMaxY = y;
  myMinY = y;
  myCenX = x;
  myCenY = y;
}

/** 
    Ends the calibration, which also sets the center to where the joystick is
    when the function is called... the center is never reset except in this 
    function, whereas the min and maxes are constantly checked
    @see startCal
*/
    
AREXPORT void ArJoyHandler::endCal(void)
{
  int x, y;
  
  getUnfiltered(&x, &y);
  myCenX = x;
  myCenY = y;
}

AREXPORT void ArJoyHandler::getStats(int *maxX, int *minX, int *maxY, 
				     int *minY, int *cenX, int *cenY)
{
  *maxX = myMaxX;
  *minX = myMinX;
  *maxY = myMaxY;
  *minY = myMinY;
  *cenX = myCenX;
  *cenY = myCenY;
}

AREXPORT void ArJoyHandler::setStats(int maxX, int minX, int maxY, int minY, 
			   int cenX, int cenY)
{
  myMaxX = maxX;
  myMinX = minX;
  myMaxY = maxY;
  myMinY = minY;
  myCenX = cenX;
  myCenY = cenY;
}

/**
   @swignote   Use the version of this method returning ArJoyVec3i
*/
AREXPORT void ArJoyHandler::getSpeeds(int *x, int *y, int *z)
{
  *x = myTopX;
  *y = myTopY;
  if (z != NULL)
    *z = myTopZ;
}

/**
   if useOSCal is true then this returns the readings as calibrated
   from the OS.  If useOSCal is false this finds the percentage of the
   distance between center and max (or min) then takes this percentage
   and multiplies it by the speeds given the class, and returns the
   values computed from this.
   
   @param x pointer to an integer in which to store the x value, which
   will be within the range [-1 * x given in setSpeeds(), x given in setSpeeds()]
   @param y pointer to an integer in which to store the y value, which
   will be within the range [-1 * y given in setSpeeds(), y given in setSpeeds()]
   @param z pointer to an integer in which to store the z value, which
   will be within the range [-1 * z given in setSpeeds(), z given in setSpeeds()]
   @java   Use the version of this method returning ArJoyVec3i
   @python returns a tuple of (x, y, z) instead
**/

AREXPORT void ArJoyHandler::getAdjusted(int *x, int *y, int *z)
{
  int curX, curY, curZ;

  getUnfiltered(&curX, &curY, &curZ);
  if (myUseOSCal)
  {
    *x = ArMath::roundInt(((double)curX) / 128.0 * ((double)myTopX));
    *y = ArMath::roundInt(((double)curY) / 128.0 * ((double)myTopY));
    if (z != NULL)
      *z = ArMath::roundInt(((double)curZ) / 128.0 * ((double)myTopZ));
    return;
  }

  if (curX > myCenX && myMaxX - myCenX != 0 ) {
    *x = (int)((double)(curX - myCenX)/(double)(myMaxX - myCenX)*
	       (double)myTopX);
  } else if (curX <= myCenX && myCenX - myMinX != 0) {
    *x = (int)((double)(myCenX - curX)/(double)(myCenX - myMinX)*
	       (double)-myTopX);
  } else
    *x = 0;
  if (curY > myCenY && myMaxY - myCenY != 0) {
    *y = (int)((double)(curY - myCenY)/(double)(myMaxY - myCenY)*
	       (double)myTopY);
  } else if (curY <= myCenY && myCenY - myMinY != 0) {
    *y = (int)((double)(myCenY - curY)/(double)(myCenY - myMinY)*
	       (double)-myTopY);
  } else 
    *y = 0;
  if (z != NULL)
    *z = ArMath::roundInt(((double)curZ) / 128.0 * ((double)myTopZ));
  
}

/**
   If useOSCal is true then this gets normalized values 
   (between -1.0 and 1.0)  as calibrated
   by the OS. In other words, the range [-1.0,1.0] is scaled
   to the calibrated range determined by the set maximum.   If useOSCal is false this finds the percentage of the
   distance between center and max (or min) then takes this percentage
   and multiplies it by the speeds given the class, and returns the
   values computed from this.
   
   @param x pointer to an integer in which to store the x value.
    Will be within the range [-1.0, 1.0]
   @param y pointer to an integer in which to store the y value.
    Will be within the range [-1.0, 1.0]
   @param z pointer to an integer in which to store the z value.
    Will be within the range [-1.0, 1.0]

   @python Returns a tuple of (x, y, z) instead
   @java   Use the version of this method returning ArJoyVec3f
   @python Returns a tuple of (x, y, z) instead
**/

AREXPORT void ArJoyHandler::getDoubles(double *x, double *y, double *z)
{
  int curX, curY, curZ;

  getUnfiltered(&curX, &curY, &curZ);
  if (myUseOSCal)
  {
    *x = ((double)curX) / 128.0;
    *y = ((double)curY) / 128.0;
    if (z != NULL)
      *z = ((double)curZ) / 128.0;
    return;
  }

  if (curX > myCenX && myMaxX - myCenX != 0 ) {
    *x = (int)((double)(curX - myCenX)/(double)(myMaxX - myCenX));
  } else if (curX <= myCenX && myCenX - myMinX != 0) {
    *x = (int)((double)(myCenX - curX)/(double)(myCenX - myMinX));
  } else
    *x = 0;
  if (curY > myCenY && myMaxY - myCenY != 0) {
    *y = (int)((double)(curY - myCenY)/(double)(myMaxY - myCenY));
  } else if (curY <= myCenY && myCenY - myMinY != 0) {
    *y = (int)((double)(myCenY - curY)/(double)(myCenY - myMinY));
  } else 
    *y = 0;
  if (z != NULL)
    *z = curZ;
}

/**
   This returns the raw value from the joystick... with X and Y 
   varying between -128 and 128. This data normally shouldn't be used 
   except in calibration since it can give very uncentered or inconsistent readings.  
   For example its not uncommon for a joystick to move 10 to the right
   but 50 or 100 to the left, so if you aren't adjusting for this you get
   a robot (or whatever) that goes left really fast, but will hardly go right.
   Instead you should use getAdjusted() exclusively except for calibration,
   or informational purposes.
   @param x pointer to an integer in which to store x value
   @param y pointer to an integer in which to store y value
   @param z pointer to an integer in which to store z value
   @java   Use the version of this method returning ArJoyVec3i
   @python Returns a tuple of (x, y, z) instead
*/
AREXPORT void ArJoyHandler::getUnfiltered(int *x, int* y, int *z)
{
  getData();
  *x = myAxes[1];
  *y = myAxes[2];
  if (z != NULL)
    *z = myAxes[3];
}

/**
   @param axis axis to get, should range from 1 through getNumAxes()
**/

AREXPORT double ArJoyHandler::getAxis(unsigned int axis)
{
  // make sure we have that axis
  if (axis < 1 || axis > myAxes.size())
    return 0;

  std::map<unsigned int, int>::iterator iter = myAxes.find(axis);
  if (iter != myAxes.end()) {
    return (iter->second) / 128.0;
  }
  else {
    return 0;
  }
  /**
  // now make sure its in there
  if (myAxes.find(axis) == myAxes.end())
    return 0;
  return myAxes[axis]/128.0;
  **/
}

/**
   @param button button to test for pressed, within the range 1 through
   getNumButtons()
 
   @return true if the button is pressed, false otherwise
**/

AREXPORT bool ArJoyHandler::getButton(unsigned int button)
{
  getData();
  // make sure we have that axis
  if (button < 1 || button > myButtons.size())
    return 0;

  // now make sure its in there
  if (myButtons.find(button) == myButtons.end())
    return 0;
  
  return myButtons[button];
}

/**
   @return the number of axes (axes are indexed as 1 through this number)
**/

AREXPORT unsigned int ArJoyHandler::getNumAxes(void)
{
  return (unsigned int) myAxes.size();
}

/**
   @return the number of buttons (buttons are indexed as 1 through this number)
**/

AREXPORT unsigned int ArJoyHandler::getNumButtons(void)
{
  return (unsigned int) myButtons.size();
}

