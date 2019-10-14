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
#ifndef ARACTIONDRIVEDISTANCE_H
#define ARACTIONDRIVEDISTANCE_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

/// This action drives the robot specific distances

/**
   This action naively drives a fixed distance. The action stops the
   robot when it has travelled the appropriate distance. It
   travels at 'speed' mm/sec.

   You can give it a distance with setDistance(), cancel its movement
   with cancelDistance(), and see if it got there with
   haveAchievedDistance().

   You can tell it to go backwards by calling setDistance with a
   negative value.

   This doesn't avoid obstacles or anything, you could add have an
   limiting ArAction at a higher priority to try to do this (so you
   don't smash things). (For truly intelligent navigation, see
   the ARNL or SONARNL software libraries.)
  @ingroup ActionClasses
**/


class ArActionDriveDistance : public ArAction
{
public:
  AREXPORT ArActionDriveDistance(const char *name = "driveDistance", 
				double speed = 400, double deceleration = 200);
  AREXPORT virtual ~ArActionDriveDistance();

  /// Sees if the goal has been achieved
  AREXPORT bool haveAchievedDistance(void);
  /// Cancels the goal the robot has
  AREXPORT void cancelDistance(void);
  /// Sets a new goal and sets the action to go there
  AREXPORT void setDistance(double distance, bool useEncoders = true);
  /// Gets whether we're using the encoder position or the normal position
  bool usingEncoders(void) { return myUseEncoders; }
  /// Sets the speed the action will travel at (mm/sec)
  void setSpeed(double speed = 400) { mySpeed = speed; }
  /// Gets the speed the action will travel at (mm/sec)
  double getSpeed(void) { return mySpeed; }
  /// Sets the deceleration the action will use (mm/sec/sec)
  void setDeceleration(double deceleration = 200) 
    { myDeceleration = deceleration; }
  /// Gets the deceleration the action will use (mm/sec/sec)
  double getDeceleration(void) { return myDeceleration; }
  /// Sets if we're printing or not
  void setPrinting(bool printing) { myPrinting = printing; }
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  double myDistance;
  bool myUseEncoders;
  double mySpeed;
  double myDeceleration;
  ArActionDesired myDesired;
  bool myPrinting;
  double myLastVel;

  double myDistTravelled;
  ArPose myLastPose;
  
  enum State
  {
    STATE_NO_DISTANCE, 
    STATE_ACHIEVED_DISTANCE,
    STATE_GOING_DISTANCE
  };
  State myState;
};

#endif // ARACTIONDRIVE
