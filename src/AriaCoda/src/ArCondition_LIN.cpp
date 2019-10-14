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
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <string.h>
#include "ariaOSDef.h"
#include "ArCondition.h"
#include "ArLog.h"

#include <time.h>


ArStrMap ArCondition::ourStrMap;


AREXPORT ArCondition::ArCondition() :
  myFailedInit(false),
  myCond(),
  myMutex(false)
{
  myMutex.setLogName("ArCondition::myMutex");
  pthread_condattr_t attr;

  pthread_condattr_init(&attr);
  if (pthread_cond_init(&myCond, &attr) != 0)
  {
    ArLog::log(ArLog::Terse, "ArCondition::ArCondition: Unknown error trying to create the condition.");
    myFailedInit=true;
  }

  pthread_condattr_destroy(&attr);

  ourStrMap[STATUS_FAILED]="General failure";
  ourStrMap[STATUS_FAILED_DESTROY]=
  "Another thread is waiting on this condition so it can not be destroyed";
  ourStrMap[STATUS_FAILED_INIT] =
  "Failed to initialize thread. Requested action is imposesible";
  ourStrMap[STATUS_MUTEX_FAILED_INIT]="The underlying mutex failed to init";
  ourStrMap[STATUS_MUTEX_FAILED]="The underlying mutex failed in some fashion";
}

AREXPORT ArCondition::~ArCondition()
{
  int ret;

  ret=pthread_cond_destroy(&myCond);
  if (ret == EBUSY)
    ArLog::log(ArLog::Terse, "ArCondition::~ArCondition: Trying to destroy a condition that another thread is waiting on.");
  else if (ret != 0)
    ArLog::log(ArLog::Terse, "ArCondition::~ArCondition: Unknown error while trying to destroy the condition.");
}

AREXPORT int ArCondition::signal()
{
  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::signal: Initialization of condition failed, failed to signal");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_cond_signal(&myCond) != 0)
  {
    ArLog::log(ArLog::Terse, "ArCondition::signal: Unknown error while trying to signal the condition.");
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::broadcast()
{
  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::broadcast: Initialization of condition failed, failed to broadcast");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_cond_broadcast(&myCond) != 0)
  {
    ArLog::log(ArLog::Terse, "ArCondition::broadcast: Unknown error while trying to broadcast the condition.");
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::wait()
{
  int ret;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::wait: Initialization of condition failed, failed to wait");
    return(STATUS_FAILED_INIT);
  }

  ret=myMutex.lock();
  if (ret != 0)
  {
    if (ret == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  ret=pthread_cond_wait(&myCond, &myMutex.getMutex());
  if (ret != 0)
  {
    if (ret == EINTR)
      return(STATUS_WAIT_INTR);
    else
    {
      ArLog::log(ArLog::Terse, "ArCondition::wait: Unknown error while trying to wait on the condition.");
      return(STATUS_FAILED);
    }
  }

  ret=myMutex.unlock();
  if (ret != 0)
  {
    if (ret == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::timedWait(unsigned int msecs)
{
  int ret;
  int retUnlock;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::wait: Initialization of condition failed, failed to wait");
    return(STATUS_FAILED_INIT);
  }

  ret=myMutex.lock();
  if (ret != 0)
  {
    if (ret == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  /*
  gettimeofday(&tp, NULL);
  // convert time of day to pthread time structure
  spec.tv_sec = tp.tv_sec;
  spec.tv_nsec = tp.tv_usec * 1000;

  // add on time specified by msecs
  spec.tv_sec += (long int)rint(((float)msecs)/1000.0);
  // 1 millisecond = 1000 micro seconds = 1000000 nanoseconds
  spec.tv_nsec += (long int)( ( msecs % 1000 ) * 1000000);
//   printf("input millisecond=%d :: sec=%ld nsec=%ld curtime=%ld %ld\n", msecs, spec.tv_sec, spec.tv_nsec, tp.tv_sec, tp.tv_usec * 1000);
*/
  struct timespec spec;
#if defined(_POSIX_TIMERS) && !defined(__MACH__)
  clock_gettime(CLOCK_REALTIME, &spec);
#else
#warning posix realtime timer not available so using gettimeofday instead of clock_gettime
  struct timeval tp;
  gettimeofday(&tp, NULL);
  spec.tv_sec = tp.tv_sec;
  spec.tv_nsec = tp.tv_usec * 1000;
#endif

  spec.tv_sec += msecs / 1000;
  spec.tv_nsec += msecs * 1000 * 1000;
  if (spec.tv_nsec > 1000 * 1000 * 1000)
  {
    spec.tv_sec += 1;
    spec.tv_nsec -= 1000 * 1000 * 1000;
  }
  

  int timedWaitErrno;
  
  ret=pthread_cond_timedwait(&myCond, &myMutex.getMutex(), &spec);
  timedWaitErrno = errno;

  /*
  if (ret != 0)
    ArLog::logErrorFromOS(ArLog::Terse, "ArCondition::timedWait: Unknown error while trying to wait on the condition. Ret %d, %s", ret, strerror(ret));
  */

  // must unlock the mutex, even if we fail, since we reacquire lock
  // after timedwait times out
  retUnlock=myMutex.unlock();
  
  if (ret != 0)
  {
    if (ret == EINTR)
      return(STATUS_WAIT_INTR);
    else if (ret == ETIMEDOUT)
      return(STATUS_WAIT_TIMEDOUT);
    else
    {
      ArLog::logErrorFromOS(ArLog::Terse, "ArCondition::timedWait: Unknown error while trying to wait on the condition. Ret %d, %s.  Errno %d, %s.", 
			    ret, strerror(ret),
			    timedWaitErrno, strerror(timedWaitErrno));

      return(STATUS_FAILED);
    }
  }

  if (retUnlock != 0)
  {
    if (retUnlock == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  return(0);
}

AREXPORT const char * ArCondition::getError(int messageNumber) const
{
  ArStrMap::const_iterator it;
  if ((it = ourStrMap.find(messageNumber)) != ourStrMap.end())
    return (*it).second.c_str();
  else
    return NULL;
}
