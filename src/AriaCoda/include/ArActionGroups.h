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
#ifndef ARACTIONGROUPS_H
#define ARACTIONGROUPS_H

#include "ariaTypedefs.h"
#include "ArActionGroup.h"
#include "ArPTZ.h"

class ArActionStop;
class ArActionInput;
class ArActionJoydrive;
class ArActionDeceleratingLimiter;
class ArActionRatioInput;
class ArRatioInputKeydrive;
class ArRatioInputJoydrive;
class ArRatioInputRobotJoydrive;

/// Action group to use to drive the robot with input actions (keyboard, joystick, etc.)
/** 
   This class is just useful for teleoping the robot under your own
   joystick and keyboard control... Note that you the predefined
   ArActionGroups in ARIA are made only to be used exclusively... only
   one can be active at once.

   This class is largely now obsolete (it is used by ArServerModeDrive
   but that is now obsolete and was replaced by a class that just
   makes its own action group)

   ArActionGroupRatioDrive is better.
**/
class ArActionGroupInput : public ArActionGroup
{
public:
  AREXPORT ArActionGroupInput(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupInput();
  AREXPORT void setVel(double vel);
  AREXPORT void setRotVel(double rotVel);
  AREXPORT void setHeading(double heading);
  AREXPORT void deltaHeadingFromCurrent(double delta);
  AREXPORT void clear(void);
  AREXPORT ArActionInput *getActionInput(void);
protected:
  ArActionInput *myInput;
};

/// Action group to stop the robot
/** 
   This class is just useful for having the robot stopped... Note that
   you the predefined ArActionGroups in ARIA are made only to be used
   exclusively... they won't combine.
**/
class ArActionGroupStop : public ArActionGroup
{
public:
  AREXPORT ArActionGroupStop(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupStop();
  AREXPORT ArActionStop *getActionStop(void);
public:
  ArActionStop *myActionStop;
};

/// Action group to teleopoperate the robot using ArActionJoydrive, and the Limiter actions to avoid collisions.
/** 
   This class is just useful for teleoping the robot and having these
   actions read the joystick and keyboard... Note that you the
   predefined ArActionGroups in ARIA are made only to be used
   exclusively... only one can be active at once.
**/
class ArActionGroupTeleop : public ArActionGroup
{
public:
  AREXPORT ArActionGroupTeleop(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupTeleop();
  AREXPORT void setThrottleParams(int lowSpeed, int highSpeed);
protected:
  ArActionJoydrive *myJoydrive;
};

/// Action group to teleoperate the robot using ArActionJoydrive, but without any Limiter actions to avoid collisions.
/** 
   This class is just useful for teleoping the robot in an unguarded
   and unsafe manner and having these actions read the joystick and
   keyboard... Note that you the predefined ArActionGroups in ARIA are
   made only to be used exclusively... only one can be active at once.
**/
class ArActionGroupUnguardedTeleop : public ArActionGroup
{
public:
  AREXPORT ArActionGroupUnguardedTeleop(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupUnguardedTeleop();
  AREXPORT void setThrottleParams(int lowSpeed, int highSpeed);
protected:
  ArActionJoydrive *myJoydrive;
};

/// Action group to make the robot wander, avoiding obstacles.
/** 
   This class is useful for having the robot wander... Note that
   you the predefined ArActionGroups in ARIA are made only to be used
   exclusively... only one can be active at once.
**/
class ArActionGroupWander : public ArActionGroup
{
public:
  AREXPORT ArActionGroupWander(ArRobot *robot, int forwardVel = 400, int avoidFrontDist = 450, int avoidVel = 200, int avoidTurnAmt = 15);
  AREXPORT virtual ~ArActionGroupWander();
};

/// Use keyboard and joystick input to to drive the robot, with Limiter actions to avoid obstacles.
/** 
   This class is just useful for teleoping the robot under your own
   joystick and keyboard control... Note that you the predefined
   ArActionGroups in ARIA are made only to be used exclusively (one at
   a time)... only one can be active at once.
**/
class ArActionGroupRatioDrive : public ArActionGroup
{
public:
  AREXPORT ArActionGroupRatioDrive(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupRatioDrive();
  AREXPORT ArActionRatioInput *getActionRatioInput(void);
  AREXPORT void addToConfig(ArConfig *config, const char *section);
protected:
  ArActionDeceleratingLimiter *myDeceleratingLimiterForward;
  ArActionDeceleratingLimiter *myDeceleratingLimiterBackward;
  ArActionDeceleratingLimiter *myDeceleratingLimiterLateralLeft;
  ArActionDeceleratingLimiter *myDeceleratingLimiterLateralRight;
  ArActionRatioInput *myInput;
  ArRatioInputKeydrive *myKeydrive;
  ArRatioInputJoydrive *myJoydrive;
  ArRatioInputRobotJoydrive *myRobotJoydrive;

};


/// Use keyboard and joystick input to to drive the robot, but without Limiter actions to avoid obstacles.
/** 
   This class is just useful for teleoping the robot under your own
   joystick and keyboard control... Note that you the predefined
   ArActionGroups in ARIA are made only to be used exclusively (one at
   a time)... only one can be active at once.
**/
class ArActionGroupRatioDriveUnsafe : public ArActionGroup
{
public:
  AREXPORT ArActionGroupRatioDriveUnsafe(ArRobot *robot);
  AREXPORT virtual ~ArActionGroupRatioDriveUnsafe();
  AREXPORT ArActionRatioInput *getActionRatioInput(void);
  AREXPORT void addToConfig(ArConfig *config, const char *section);
protected:
  ArActionRatioInput *myInput;
  ArRatioInputKeydrive *myKeydrive;
  ArRatioInputJoydrive *myJoydrive;
  ArRatioInputRobotJoydrive *myRobotJoydrive;

};

#endif // ARACTIONGROUPS_H
