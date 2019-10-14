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
#ifndef ARSYNCTASK_H
#define ARSYNCTASK_H

#include <string>
#include <map>
#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include "ArTaskState.h"

/// Class used internally to manage the tasks that are called every cycle
/**
   This is used internally, no user should normally have to create one, but 
   serious developers may want to use the members.  Most users will be able to 
   add user tasks via the ArRobot class.

   The way it works is that each instance is a node in a tree.  The only node 
   that should ever be created with a new is the top one.  The run and print
   functions both call the run/print on themselves, then on all of their 
   children, going from lowest numbered position to highest numbered, lower
   going first.  There are no hard limits to the position, it can be any 
   integer.  ARIA uses the convention of 0 to 100, when you add things of
   your own you should leave room to add in between.  Also you can add things
   with the same position, the only effect this has is that the first addition
   will show up first in the run or print.

   After the top one is created, every other task should be created with 
   either addNewBranch() or addNewLeaf().  Each node can either be a branch node 
   or a list node.  The list (a multimap) of branches/nodes is ordered
   by the position passed in to the add function.  addNewBranch() adds a new 
   branch node to the instance it is called on, with the given name and
   position.  addNewLeaf() adds a new leaf node to the instance it is called on,
   with the given name and position, and also with the ArFunctor given, this 
   functor will be called when the leaf is run.  Either add creates the new 
   instance and puts it in the list of branches/nodes in the approriate spot.

   The tree takes care of all of its own memory management and list management,
   the "add" functions put into the list and creates the memory, conversely
   if you delete an ArSyncTask (which is the correct way to get rid of one)
   it will remove itself from its parents list.

   If you want to add something to the tree the proper way to do it is to get
   the pointer to the root of the tree (ie with ArRobot::getSyncProcRoot) and
   then to use find on the root to find the branch you want to travel down,
   then continue this until you find the node you want to add to.  Once there
   just call addNewBranch or addNewLeaf and you're done.

   The state of a task can be stored in the target of a given ArTaskState::State pointer,
   or if NULL than ArSyncTask will use its own member variable.

  @internal
*/

class ArSyncTask
{
public:
  /// Constructor, shouldn't ever do a new on anything besides the root node
  AREXPORT ArSyncTask(const char *name, ArFunctor * functor = NULL, 
		      ArTaskState::State *state = NULL,
		      ArSyncTask * parent = NULL);
  /// Destructor
  AREXPORT virtual ~ArSyncTask();

  /// Runs the node, which runs all children of this node as well
  AREXPORT void run(void);
  /// Prints the node, which prints all the children of this node as well
  AREXPORT void log(int depth = 0);

  /// Gets the state of the task
  AREXPORT ArTaskState::State getState(void);
  /// Sets the state of the task
  AREXPORT void setState(ArTaskState::State state);
    
  /// Finds the task in the instances list of children, by name
  AREXPORT ArSyncTask *findNonRecursive(const char *name);
  /// Finds the task in the instances list of children, by functor
  AREXPORT ArSyncTask *findNonRecursive(ArFunctor *functor);

  /// Finds the task recursively down the tree by name
  AREXPORT ArSyncTask *find(const char *name); 
  /// Finds the task recursively down the tree by functor
  AREXPORT ArSyncTask *find(ArFunctor *functor);

  /// Returns what this is running, if anything (recurses)
  AREXPORT ArSyncTask *getRunning(void);

  /// Adds a new branch to this instance
  AREXPORT void addNewBranch(const char *nameOfNew, int position, 
			     ArTaskState::State *state = NULL);
  /// Adds a new leaf to this instance
  AREXPORT void addNewLeaf(const char *nameOfNew, int position, 
			   ArFunctor *functor, 
			   ArTaskState::State *state = NULL);

  /// Gets the name of this task
  AREXPORT std::string getName(void);

  /// Gets the functor this instance runs, if there is one
  AREXPORT ArFunctor *getFunctor(void);

  /// Sets the functor called to get the cycle warning time (should only be used from the robot)
  AREXPORT void setWarningTimeCB(
	  ArRetFunctor<unsigned int> *functor);
  /// Gets the functor called to get the cycle warning time (should only be used from the robot)
  AREXPORT ArRetFunctor<unsigned int> *getWarningTimeCB(void);

  /// Sets the functor called to check if there should be a time warning this cycle (should only be used from the robot)
  AREXPORT void setNoTimeWarningCB(
	  ArRetFunctor<bool> *functor);
  /// Gets the functor called to check if there should be a time warning this cycle (should only be used from the robot)
  AREXPORT ArRetFunctor<bool> *getNoTimeWarningCB(void);
  
  // removes this task from the map
  AREXPORT void remove(ArSyncTask * proc);

  // returns whether this node is deleting or not
  AREXPORT bool isDeleting(void);
protected:
  std::multimap<int, ArSyncTask *> myMultiMap;
  ArTaskState::State *myStatePointer;
  ArTaskState::State myState;
  ArFunctor *myFunctor;
  std::string myName;
  ArSyncTask *myParent;
  bool myIsDeleting;
  ArRetFunctor<unsigned int> *myWarningTimeCB;
  ArRetFunctor<bool> *myNoTimeWarningCB;
  // variables for introspection
  bool myRunning;
  // this is just a pointer to what we're invoking so we can know later
  ArSyncTask *myInvokingOtherFunctor;
};



#endif




