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
#include "ArPriorityResolver.h"
#include "ArAction.h"
#include "ArRobot.h"

AREXPORT ArPriorityResolver::ArPriorityResolver() :
  ArResolver("ArPriorityResolver", "Resolves strictly by using priority, the highest priority action to act is the one that gets to go.  Does no mixing of any variety.")
{
}


AREXPORT ArPriorityResolver::~ArPriorityResolver()
{
}

AREXPORT ArActionDesired *ArPriorityResolver::resolve(
	ArResolver::ActionMap *actions, ArRobot *robot, bool logActions)
{
  ArResolver::ActionMap::reverse_iterator it;
  ArAction *action;
  ArActionDesired *act;
  ArActionDesired averaging;
  bool first = true;
  int lastPriority;
  bool printedFirst = true;
  int printedLast;
  
  if (actions == NULL)
    return NULL;

  myActionDesired.reset();
  averaging.reset();
  averaging.startAverage();
  for (it = actions->rbegin(); it != actions->rend(); ++it)
  {
    action = (*it).second;
    if (action != NULL && action->isActive())
    {
      /// MPL jan 7 '12 moved this next code block up 'from here'
      if (first || (*it).first != lastPriority)
      {
	averaging.endAverage();
	myActionDesired.merge(&averaging);
	
	averaging.reset();
	averaging.startAverage();
	first = false;
	lastPriority = (*it).first;
      }
      act = action->fire(myActionDesired);
      if (robot != NULL && act != NULL)
	act->accountForRobotHeading(robot->getTh());
      if (act != NULL)
	act->sanityCheck(action->getName());
      // from here
      averaging.addAverage(act);
      if (logActions && act != NULL && act->isAnythingDesired())
      {
	if (printedFirst || printedLast != (*it).first)
	{
	  ArLog::log(ArLog::Terse, "Priority %d:", (*it).first);
	  printedLast = (*it).first;
	  printedFirst = false;
	}
	ArLog::log(ArLog::Terse, "Action: %s", action->getName());
	act->log();
      }
	
	
    }
  }
  averaging.endAverage();
  myActionDesired.merge(&averaging);
  /*
  printf(
      "desired delta %.0f strength %.3f, desired speed %.0f strength %.3f\n",
      myActionDesired.getDeltaHeading(), myActionDesired.getHeadingStrength(), 
      myActionDesired.getVel(), myActionDesired.getVelStrength());
  */
  return &myActionDesired;
}
