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
#ifndef ARP2ARM_H
#define ARP2ARM_H

#include "ArRobot.h"
#include "ariaTypedefs.h"
#include "ArSerialConnection.h"
#include "ariaOSDef.h"

// P2 Arm classes: ArP2Arm for control and ArP2ArmJoints for P2 Arm joint data

/// P2 Arm joint info
class P2ArmJoint
{
public:

  AREXPORT P2ArmJoint();
  AREXPORT virtual ~P2ArmJoint();

  ArTypes::UByte myPos;
  ArTypes::UByte myVel;
  ArTypes::UByte myHome;
  ArTypes::UByte myMin;
  ArTypes::UByte myCenter;
  ArTypes::UByte myMax;
  ArTypes::UByte myTicksPer90;
};

/**
   ArP2Arm is the interface to the AROS/P2OS-based Pioneer 2 Arm servers,
   by means of which the robot microcontroller firmware can control the
   original 5-DOF Pioneer 2 Arm manipulator.
   The P2 Arm is attached to the robot's microcontroller via an auxiliary
   serial port.

   To use ArmP2, you must first set up an ArRobot and have it connect
   with the robot. The ArRobot needs to be run so that it reads and writes
   packets to and from server. The easiest way is ArRobot::runAsync()
   which runs the ArRobot in its own thread.

   Then call ArP2Arm::setRobot() with ArRobot, and finally initialized
   with ArmP2::init().  Once initialized, use the various ArP2Arm
   methods to power the P2 Arm servos, move joints, and so on.

   For simple examples on how to use ArP2Arm, look in the Aria/examples
   directory for P2ArmSimple.cpp and P2ArmJoydrive.cpp.

   For additional information about the original 5-DOF Pioneer 2 Arm,
   see the robot operations manual and the arm manual, available at
   <a href="http://robots.mobilerobots.com">http://robots.mobilerobots.com</a>.

   @ingroup DeviceClasses
   @ingroup OptionalClasses
**/
/// Arm Control class
class ArP2Arm
{
public:

  /// General error conditions possible from most of the arm related functions
  typedef enum {
    SUCCESS, ///< Succeded
    ALREADY_INITED, ///< The class is already initialized
    NOT_INITED, ///< The class is not initialized
    ROBOT_NOT_SETUP, ///< The ArRobot class is not setup properly
    NO_ARM_FOUND, ///< The arm can not be found
    COMM_FAILED, ///< Communications has failed
    COULD_NOT_OPEN_PORT, ///< Could not open the communications port
    COULD_NOT_SET_UP_PORT, ///< Could not setup the communications port
    ALREADY_CONNECTED, ///< Already connected to the arm
    NOT_CONNECTED, ///< Not connected with the arm, connect first
    INVALID_JOINT, ///< Invalid joint specified
    INVALID_POSITION ///< Invalid position specified
    } State;

  /// Type of arm packet identifiers. Used in ArP2Arm::setPacketCB().
  typedef enum {
    StatusPacket, ///< The status packet type
    InfoPacket ///< The info packet type
  } PacketType;

  /// Type of status packets to request for. Used in ArP2Arm::requestStatus()
  typedef enum
  {
    StatusOff=0, ///< Stop sending status packets
    StatusSingle=1, ///< Send a single status packets
    StatusContinuous=2 ///< Send continous packets. Once every 100ms.
  } StatusType;

  /// Bit for joint 1 in arm status byte
  AREXPORT static const int ArmJoint1;
  /// Bit for joint 2 in arm status byte
  AREXPORT static const int ArmJoint2;
  /// Bit for joint 3 in arm status byte
  AREXPORT static const int ArmJoint3;
  /// Bit for joint 4 in arm status byte
  AREXPORT static const int ArmJoint4;
  /// Bit for joint 5 in arm status byte
  AREXPORT static const int ArmJoint5;
  /// Bit for joint 6 in arm status byte
  AREXPORT static const int ArmJoint6;
  /// Bit for arm good state in arm status byte
  AREXPORT static const int ArmGood;
  /// Bit for arm initialized in arm status byte
  AREXPORT static const int ArmInited;
  /// Bit for arm powered on in arm status byte
  AREXPORT static const int ArmPower;
  /// Bit for arm homing in arm status byte
  AREXPORT static const int ArmHoming;
  /// Number of joints that the arm has
  AREXPORT static int NumJoints;

  /// Constructor
  AREXPORT ArP2Arm();

  /// Destructor
  AREXPORT virtual ~ArP2Arm();

  /// Set the robot to use to talk to the arm
  AREXPORT void setRobot(ArRobot *robot) {myRobot=robot;}

  /// Init the arm class
  AREXPORT virtual State init();

  /// Uninit the arm class
  AREXPORT virtual State uninit();

  /// Power on the arm
  AREXPORT virtual State powerOn(bool doWait=true);

  /// Power off the arm
  AREXPORT virtual State powerOff();

  /// Request the arm info packet
  AREXPORT virtual State requestInfo();

  /// Request the arm status packet
  AREXPORT virtual State requestStatus(StatusType status);

  /// Request arm initialization
  AREXPORT virtual State requestInit();

  /// Check to see if the arm is still connected
  AREXPORT virtual State checkArm(bool waitForResponse=true);

  /// Home the arm
  AREXPORT virtual State home(int joint=-1);

  /// Home the arm and power if off
  AREXPORT virtual State park();

  /// Move a joint to a position in degrees
  AREXPORT virtual State moveTo(int joint, float pos, unsigned char vel=0);

  /// Move a joint to a position in low level arm controller ticks
  AREXPORT virtual State moveToTicks(int joint, unsigned char pos);

  /// Move a joint step degrees
  AREXPORT virtual State moveStep(int joint, float pos, unsigned char vel=0);

  /// Move a joint step ticks
  AREXPORT virtual State moveStepTicks(int joint, signed char pos);

  /// Set the joint to move at the given velocity
  AREXPORT virtual State moveVel(int joint, int vel);

  /// Stop the arm
  AREXPORT virtual State stop();

  /// Set the auto park timer value
  AREXPORT virtual State setAutoParkTimer(int waitSecs);

  /// Set the gripper park timer value
  AREXPORT virtual State setGripperParkTimer(int waitSecs);

  /// Set the arm stopped callback
  AREXPORT virtual void setStoppedCB(ArFunctor *func) {myStoppedCB=func;}

  /// set the arm packet callback
  AREXPORT virtual void setPacketCB(ArFunctor1<PacketType> *func)
    {myPacketCB=func;}

  /// Get the arm version
  AREXPORT virtual std::string getArmVersion() {return(myVersion);}

  /// Get the joints position in degrees
  AREXPORT virtual float getJointPos(int joint);

  /// Get the joints position in ticks
  AREXPORT virtual unsigned char getJointPosTicks(int joint);

  /// Check to see if the arm is moving
  AREXPORT virtual bool getMoving(int joint=-1);

  /// Check to see if the arm is powered
  AREXPORT virtual bool isPowered();

  /// Check to see if the arm is communicating
  AREXPORT virtual bool isGood();

  /// Get the two byts of status info from P2OS
  AREXPORT virtual int getStatus() {return(myStatus);}

  /// Get when the last arm status packet came in
  AREXPORT virtual ArTime getLastStatusTime() {return(myLastStatusTime);}

  /// Get the robot that the arm is on
  AREXPORT virtual ArRobot * getRobot() {return(myRobot);}

  /// Get the joints data structure
  AREXPORT virtual P2ArmJoint * getJoint(int joint);

  /// Converts degrees to low level arm controller ticks
  AREXPORT virtual bool convertDegToTicks(int joint, float pos,
					  unsigned char *ticks);

  /// Converts low level arm controller ticks to degrees
  AREXPORT virtual bool convertTicksToDeg(int joint, unsigned char pos,
					  float *degrees);


protected:

  // AROS/P2OS parameters
  static const unsigned int ARMpac;
  static const unsigned int ARMINFOpac;
  static const unsigned char ComArmInfo;
  static const unsigned char ComArmStats;
  static const unsigned char ComArmInit;
  static const unsigned char ComArmCheckArm;
  static const unsigned char ComArmPower;
  static const unsigned char ComArmHome;
  static const unsigned char ComArmPark;
  static const unsigned char ComArmPos;
  static const unsigned char ComArmSpeed;
  static const unsigned char ComArmStop;
  static const unsigned char ComArmAutoPark;
  static const unsigned char ComArmGripperPark;

  bool comArmInfo();
  bool comArmStats(StatusType stats=StatusSingle);
  bool comArmInit();
  bool comArmCheckArm();
  bool comArmPower(bool on);
  bool comArmHome(unsigned char joint=0xff);
  bool comArmPark();
  bool comArmPos(unsigned char joint, unsigned char pos);
  bool comArmSpeed(unsigned char joint, unsigned char speed);
  bool comArmStop(unsigned char joint=0xff);
  bool comArmAutoPark(int waitSecs);
  bool comArmGripperPark(int waitSecs);

  bool armPacketHandler(ArRobotPacket *packet);

  bool myInited;
  ArRobot *myRobot;
  //  ArmP2Model myModel;
  ArTime myLastStatusTime;
  ArTime myLastInfoTime;
  std::string myVersion;
  StatusType myStatusRequest;
  ArTypes::UByte2 myLastStatus;
  ArTypes::UByte2 myStatus;
  ArSerialConnection myCon;
  ArRetFunctorC<State, ArP2Arm> myAriaUninitCB;
  ArRetFunctor1C<bool, ArP2Arm, ArRobotPacket*> myArmPacketHandler;
  ArFunctor1<PacketType> *myPacketCB;
  ArFunctor *myStoppedCB;

  // We have 6 joints. Including the gripper. It's here so that we can
  // store its position even though its not really a joint.
  P2ArmJoint myJoints[6];
};

#endif // _ARP2ARM_H
