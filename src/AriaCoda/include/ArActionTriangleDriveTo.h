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
#ifndef ARACTIONTRIANGLEDRIVETO
#define ARACTIONTRIANGLEDRIVETO

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArLineFinder.h"

/// Action to drive up to a triangle target (e.g. docking station) found from an ArLineFinder
/**
 * This action uses ArLineFinder to find continuous "lines" in
 * laser range finder data that meet at an angle, forming the point of a
 * triangular shape, towards which the robot is driven and aligned.
 * If an ArLineFinder object is not given in the constructor, then it will
 * search for an ArRangeDevice on the ArRobot with a name "laser" and create its
 * own ArLineFinder object using that range device.

   The parameters describing the shape of the triangle target may be set with setTriangleParams(). The default values are for a trianrgle target consisting of two lines of at least 254 mm. meeting
   at a 135 degree angle. This is the shape of the triangular
   target on docking stations. Targets also may easily be constructed out of any material well-sensed by the SICK Laser, such as wood, cardboard, or metal.

   If setTwoStageApproach() is called with true
   (default) it will first drive to a half meter away from the desired
   spot (distFromVertex from the point of the vertex along the
   bisection of the angle between segments) and then after it reachs
   that point it will turn in towards the vertex and drive to the
   final spot.  If setTwoStageApproach() is called with false it'll just drive to
   the final spot.  It will stop at closeDist away unless a large turn
   is required where it'll stop 2 * closeDist away.  It drives in at
   the speed given in the constructor.  If the robot is closer to the vertex
   than it should be for the approach it'll skip the approach, if the
   robot is closer to the vertex than to the final spot it will back
   up (if its facing away from the vertex it won't see the vertex).

   If you want the action to drive straight to the vertex instead of
   to a point in front of it call setGotoVertex() with a true
   argument...  It will no longer do this if finalDistFromVertex is
   0... you would do this if you want to drive until a bumper is hit
   for instance.

   If you want the action to position using the front of the robot
   then use setPositionFront().

   @ingroup ActionClasses
 **/

class ArActionTriangleDriveTo : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionTriangleDriveTo(const char *name = "triangleDriveTo", 
				   double finalDistFromVertex = 400,
				   double approachDistFromVertex = 1000,
				   double speed = 200, 
				   double closeDist = 100,
				   double acquireTurnSpeed = 30);
  /// Destructor
  AREXPORT virtual ~ArActionTriangleDriveTo();
  /// Sets if we will turn to see if we can acquire the triangle
  void setAcquire(bool acquire = false) { myAcquire = acquire; }
  /// Gets if we will turn to see if we can acquire the triangle
  bool getAcquire(void) { return myAcquire; }
  /// Sets the final distance from vertex
  void setFinalDistFromVertex(double dist) { myFinalDistFromVertex = dist; }
  /// Sets the final distance from vertex
  double getFinalDistFromVertex(void) { return myFinalDistFromVertex; }
  /// Sets the parameter of the triangle we're looking for
  void setTriangleParams(double line1Length = 254, 
			 double angleBetween = 135,
			 double line2Length = 254)
    { myLine1Length = line1Length; myAngleBetween = angleBetween; myLine2Length = line2Length; }
  AREXPORT void setParameters(double finalDistFromVertex = 400,
			      double approachDistFromVertex = 1000,
			      double speed = 200, 
			      double closeDist = 100,
			      double acquireTurnSpeed = 30);

  /// Gets whether we adjust the vertex or not
  bool getAdjustVertex(void) { return myAdjustVertex; }
  /// Sets whether we adjust the vertex or not
  void setAdjustVertex(bool adjustVertex) { myAdjustVertex = adjustVertex; }
  /// Sets the vertex offset
  void setVertexOffset(int localXOffset, int localYOffset, double thOffset) 
    { myLocalXOffset = localXOffset; myLocalYOffset = localYOffset; 
    myThOffset = thOffset; } 
  /// Sets if we should use the legacy vertex mode or not
  void setUseLegacyVertexOffset(bool useLegacyVertexOffset)
    { myUseLegacyVertexOffset = useLegacyVertexOffset; }
  /// Gets if we are use the legacy vertex mode or not
  bool getUseLegacyVertexOffset(void) { return myUseLegacyVertexOffset; }
  /// Gets whether it always goto the vertex and not the point in front
  bool getGotoVertex(void) { return myGotoVertex; }
  /// Sets whether it always goto the vertex and not the point in front
  void setGotoVertex(bool gotoVertex) 
    { myGotoVertex = gotoVertex; }
  /// Sets the distance to the triangle at which we start ignoring it
  void setIgnoreTriangleDist(double dist = 250, 
			     bool useIgnoreInGotoVertexMode = false) 
    { myIgnoreTriangleDist = dist; 
    myUseIgnoreInGoto = useIgnoreInGotoVertexMode; }
  /// Gets the distance to the triangle at which we start ignoring it
  double getIgnoreTriangleDist(void) { return myIgnoreTriangleDist; }
  /// Gets if we're ignoring the triangle in goto vertex mode
  bool getUseIgnoreInGotoVertexMode(void) { return myUseIgnoreInGoto; }
  /// How long to keep going without having seen the vertex (0 is no timeout)
  void setVertexUnseenStopMSecs(int vertexUnseenStopMSecs = 4000)
    { myVertexUnseenStopMSecs = vertexUnseenStopMSecs; }
  /// How long to keep going without having seen the vertex (0 is no timeout)
  int getVertexUnseenStopMSecs(void) { return myVertexUnseenStopMSecs; }
  /// Sets the maximum distance between points in a line
  void setMaxDistBetweenLinePoints(int maxDistBetweenLinePoints = 0) 
    { myMaxDistBetweenLinePoints = maxDistBetweenLinePoints; }
  /// Sets the maximum lateral distance from the robot to the triangle line
  void setMaxLateralDist(int maxLateralDist = 0)
    { myMaxLateralDist = maxLateralDist; }
  /// Sets the maximum angle misalignment from the robot to the triangle line
  void setMaxAngleMisalignment(int maxAngleMisalignment = 0)
    { myMaxAngleMisalignment = maxAngleMisalignment; }
  enum State { 
    STATE_INACTIVE, ///< we aren't trying
    STATE_ACQUIRE, ///< finding the target
    STATE_SEARCHING, ///< Turning trying to find the target
    STATE_GOTO_APPROACH, ///< driving to the approach point
    STATE_ALIGN_APPROACH, ///< aligning to the target the first time
    STATE_GOTO_VERTEX, ///< driving to the vertex
    STATE_GOTO_FINAL, ///< driving to the final point
    STATE_ALIGN_FINAL, ///< aligning to the target the final time
    STATE_SUCCEEDED, ///< pointing at the target
    STATE_FAILED ///< if we're not acquiring and we lost the vertex we fail
  };
  /// Gets the state
  State getState(void) { return myState; }
  /// Gets if we've seen the vertex ever for this attempted drive (it gets reset in searching, but that's the only time, so will only be set once an activation by default (unless setAcquire is called)
  bool getVertexSeen(void) { return myVertexSeen; }
  /// Sets the line finder to use
  AREXPORT void setLineFinder(ArLineFinder *lineFinder);
  /// Sets the line finder used
  AREXPORT ArLineFinder *getLineFinder(void) { return myLineFinder; }
  
  /// Sets whether we're logging the driving or not
  void setLogging(bool logging) { myPrinting = logging; }
  /// Gets whether we're logging the driving or not
  bool setLogging(void) { return myPrinting; }
  AREXPORT virtual void activate(void);
  AREXPORT virtual void deactivate(void);
  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif

#ifndef SWIG
  /**
    This is for some internal debugging, don't try to use it, it won't
    work in windows (memory allocation issues).  If there's desire for
    a feature like this I can add it later.
    @internal
  **/
  class Data 
  {
    public:
    Data() 
    { myFinalSet = false; myApproachSet = false; myLinesAndVertexSet = false; 
    myVertexSeenThisCycle = false; myVertexSeen = false; }
    ~Data() {}
    void setLinesAndVertex(std::map<int, ArLineFinderSegment *> *lines, 
			   ArPose vertex)
    {
      myLines.clear();
      std::map<int, ArLineFinderSegment *>::iterator it;
      for (it = lines->begin(); it != lines->end(); it++)
      {
	myLines.push_front(*(*it).second);
      }
      myVertex = vertex;
      myLinesAndVertexSet = true;
    }
    void setFinal(ArPose final) { myFinalSet = true; myFinal = final; }
    void setApproach(ArPose approach) 
    { myApproachSet = true; myApproach = approach; }
    void setVertexSeenThisCycle(bool vertexSeenThisCycle)
    { myVertexSeenThisCycle = vertexSeenThisCycle; }
    void setVertexSeen(bool vertexSeen)
    { myVertexSeen = vertexSeen; }


    bool myLinesAndVertexSet;
    std::list<ArLineSegment> myLines;
    ArPose myVertex;
    bool myFinalSet;
    ArPose myFinal;
    bool myApproachSet;
    ArPose myApproach;
    bool myVertexSeenThisCycle;
    bool myVertexSeen;
  };
  /** Internal use only, sets if we're saving data or not
   *  @internal
   */
  void setSaveData(bool saveData) { mySaveData = saveData; }
  /** Internal use only, gets if we're saving data or not
   *  @internal
   */
  bool getSaveData(void) { return mySaveData; }
  /** Interal use only, gets the data we've saved
   *  @internal
   */
  Data *getData(void);
#endif // ifndef SWIG
protected:
  // Find our triangle, Internal function
  AREXPORT void findTriangle(bool initial, bool goStraight = false);
  // Finds a pose this dist from the vertex along the line from the vertex
  AREXPORT ArPose findPoseFromVertex(double distFromVertex);
  // where the vertex was in encoder coords
  ArPose myVertex;

  ArRangeDevice *myLaser;
  //ArLaser *myLaser;
  ArMutex myDataMutex;
  bool mySaveData;
  Data *myData;
  std::map<int, ArLineFinderSegment *> *myLines;
  unsigned int myGotLinesCounter;
  bool myVertexSeen;
  bool myPrinting;
  ArTime myVertexSeenLast;
  State myState;
  double myOriginalAngle;
  bool myAcquire;
  double myFinalDistFromVertex;
  double myApproachDistFromVertex;
  double mySpeed;
  double myCloseDist;
  double myAcquireTurnSpeed;
  bool myTwoStageApproach;
  double myLine1Length;
  double myAngleBetween;
  double myLine2Length;
  int myVertexUnseenStopMSecs;
  ArActionDesired myDesired;
  ArLineFinder *myLineFinder;
  bool myOwnLineFinder;
  bool myAdjustVertex;
  bool myGotoVertex;
  int myLocalXOffset;
  int myLocalYOffset;
  double myThOffset;
  bool myUseLegacyVertexOffset;
  double myIgnoreTriangleDist;
  bool myUseIgnoreInGoto;
  int myMaxDistBetweenLinePoints;
  int myMaxLateralDist;
  int myMaxAngleMisalignment;
};

#endif // ARACTIONTRIANGLEDRIVETO
