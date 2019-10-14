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
#ifndef ARROBOTJOYHANDLER_H
#define ARROBOTJOYHANDLER_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

class ArRobot;
class ArRobotPacket;

/// Interfaces to a joystick on the robot's microcontroller
/** 
    This is largely meant to be about the same as the normal joy
    handler but gets the data back from the robot about the joystick,
    but this sameness is why it reports things as it does.

    Also note that x is usually rotational velocity (since it right/left),
    whereas Y is translational (since it is up/down).

  When created, this class requests continuous joystick packets from the robot if already
  connected, or if not, requests joystick packets upon robot connection.  In
  requests that the joystick data be stopped upon normal robot disconnection, ARIA
  program exit, or when this object is destroyed.

  @ingroup OptionalClasses
**/
class ArRobotJoyHandler
{
 public:
  /// Constructor
  AREXPORT ArRobotJoyHandler(ArRobot *robot);
  /// Destructor
  AREXPORT ~ArRobotJoyHandler();
  /// Gets the adjusted reading, as floats
  AREXPORT void getDoubles(double *x, double *y, double *z);
  /// Gets the first button 
  bool getButton1(void) { return myButton1; }
  /// Gets the second button 
  bool getButton2(void) { return myButton2; }
  /// Gets the time we last got information back
  AREXPORT ArTime getDataReceivedTime(void) { return myDataReceived; }
  /// If we've ever gotten a packet back
  AREXPORT bool gotData(void) { return myGotData; }
  /// Adds to a section in a config
  AREXPORT void addToConfig(ArConfig *config, const char *section);
  /// Gets the X value (only use for information, or with the robot locked, getDoubles is preferred)
  int getRawX(void) { return myRawX; }
  /// Gets the Y value (only use for information, or with the robot locked, getDoubles is preferred)
  int getRawY(void) { return myRawY; }
  /// Gets the throttle value (only use for information, or with the robot locked, getDoubles is preferred)
  int getRawThrottle(void) { return myRawThrottle; }

 protected:
  AREXPORT bool handleJoystickPacket(ArRobotPacket *packet);
  AREXPORT void connectCallback(void);

  ArRobot *myRobot;
  ArTime myDataReceived;
  bool myButton1;
  bool myButton2;
  double myJoyX;
  double myJoyY;
  double myThrottle;
  bool myGotData;

  int myJoyXCenter;
  int myJoyYCenter;

  int myRawX;
  int myRawY;
  int myRawThrottle;

  ArTime myStarted;
  ArRetFunctor1C<bool, ArRobotJoyHandler,
      ArRobotPacket *> myHandleJoystickPacketCB;
  ArFunctorC<ArRobotJoyHandler> myConnectCB;

  ArFunctorC<ArRobotJoyHandler> myStopPacketsCB;

  void stopPackets();

};


#endif // ARJOYHANDLER_H

