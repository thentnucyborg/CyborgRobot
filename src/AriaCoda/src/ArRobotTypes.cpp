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
#include "ArRobotTypes.h"
#include "ArLog.h"

/** @cond INCLUDE_INTERNAL_ROBOT_PARAM_CLASSES */

// Generic robot class

AREXPORT ArRobotGeneric::ArRobotGeneric(const char *dir)
{
}

// AmigoBot robot class

AREXPORT ArRobotAmigo::ArRobotAmigo(const char *dir)
{
  sprintf(mySubClass, "amigo");
  myRobotRadius = 180;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1000;
  myDistConvFactor = 0.5083;
  myVelConvFactor = 0.6154;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.011;
  myVel2Divisor = 20;
  myRobotWidth = 279;
  myRobotLength = 330;
  myRobotLengthFront = 160;
  myRobotLengthRear = 170;

  myNumSonar = 8;
  internalSetSonar(0, 76, 100, 90);
  internalSetSonar(1, 125, 75, 41);
  internalSetSonar(2, 150, 30, 15);
  internalSetSonar(3, 150, -30, -15);
  internalSetSonar(4, 125, -75, -41);
  internalSetSonar(5, 76, -100, -90);
  internalSetSonar(6, -140, -58, -145);
  internalSetSonar(7, -140, 58, 145);
}

// AmigoBot robot class

AREXPORT ArRobotAmigoSh::ArRobotAmigoSh(const char *dir)
{
  sprintf(mySubClass, "amigo-sh");
  myRobotRadius = 180;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1000;
  myDistConvFactor = 1;
  myVelConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.011;
  myVel2Divisor = 20;
  myRobotWidth = 279;
  myRobotLength = 330;
  myRobotLengthFront = 160;
  myRobotLengthRear = 170;

  myNumSonar = 8;
  internalSetSonar(0, 70, 100, 90);
  internalSetSonar(1, 125, 75, 41);
  internalSetSonar(2, 144, 30, 15);
  internalSetSonar(3, 144, -30, -15);
  internalSetSonar(4, 120, -75, -41);
  internalSetSonar(5, 70, -100, -90);
  internalSetSonar(6, -146, -58, -145);
  internalSetSonar(7, -146, 58, 145);
}

AREXPORT ArRobotAmigoShTim5xxWibox::ArRobotAmigoShTim5xxWibox(const char* dir)
  : ArRobotAmigoSh(dir)
{
  sprintf(mySubClass, "amigo-sh-tim3xx");
  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "tim5XX");
    sprintf(getLaserData(1)->myLaserPortType, "tcp");
    sprintf(getLaserData(1)->myLaserPort, "10.0.126.11:8102");
    getLaserData(1)->myLaserX = 110;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserZ = 200;
    getLaserData(1)->myLaserAutoConnect = true;
  }
}

// P2AT robot class

AREXPORT ArRobotP2AT::ArRobotP2AT(const char *dir)
{
  sprintf(mySubClass, "p2at");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300; 
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.32;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 160;
    getLaserData(1)->myLaserY = 7;
  }


  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}

// P2AT8 robot class

AREXPORT ArRobotP2AT8::ArRobotP2AT8(const char *dir)
{
  sprintf(mySubClass, "p2at8");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.32;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 160;
    getLaserData(1)->myLaserY = 7;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif  
  myVideoParams[0].setConnect(true);
}


// P2IT robot class

AREXPORT ArRobotP2IT::ArRobotP2IT(const char *dir)
{
  sprintf(mySubClass, "p2it");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.136;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0032;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 160;
    getLaserData(1)->myLaserY = 7;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}


// P2DX robot class

AREXPORT ArRobotP2DX::ArRobotP2DX(const char *dir)
{
  sprintf(mySubClass, "p2dx");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.84;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 17;
    getLaserData(1)->myLaserY = 8;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}

// P2DXe robot class

AREXPORT ArRobotP2DXe::ArRobotP2DXe(const char *dir)
{
  sprintf(mySubClass, "p2de");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.969;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {    
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 17;
    getLaserData(1)->myLaserY = 8;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}

// P2DF robot class

AREXPORT ArRobotP2DF::ArRobotP2DF(const char *dir)
{
  sprintf(mySubClass, "p2df");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = .0060;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 17;
    getLaserData(1)->myLaserY = 8;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif
  myVideoParams[0].setConnect(true);
}

// P2D8 robot class

AREXPORT ArRobotP2D8::ArRobotP2D8(const char *dir)
{
  sprintf(mySubClass, "p2d8");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 18;
    getLaserData(1)->myLaserY = 0;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}



// P2CE robot class

AREXPORT ArRobotP2CE::ArRobotP2CE(const char *dir)
{
  sprintf(mySubClass, "p2ce");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.826;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0057;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif
  myVideoParams[0].setConnect(true);
}


// P2PP robot class

AREXPORT ArRobotP2PP::ArRobotP2PP(const char *dir)
{
  sprintf(mySubClass, "p2pp");
  myRobotRadius = 300;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0060;
  myRobotWidth = 425;
  myRobotLength = 513;

  myTableSensingIR = true;
  myNewTableSensingIR = false;
  myFrontBumpers = true;
  myRearBumpers = true;

  myNumSonar = 24;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 16;
    getLaserData(1)->myLaserY = 1;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif
  myVideoParams[0].setConnect(true);
}

// P2PB robot class

AREXPORT ArRobotP2PB::ArRobotP2PB(const char *dir)
{
  sprintf(mySubClass, "p2pb");
  myRobotRadius = 300;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.424;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0056;
  myFrontBumpers = true;
  myRearBumpers = true;
  myRobotWidth = 425;
  myRobotLength = 513;
  
  myNumSonar = 24;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 17;
    getLaserData(1)->myLaserY = 8;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif
  myVideoParams[0].setConnect(true);
}

// PerfPB robot class

AREXPORT ArRobotPerfPB::ArRobotPerfPB(const char *dir)
{
  sprintf(mySubClass, "perfpb");
  myRobotRadius = 340;
  myRobotDiagonal = 120; 
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.006;

  myRequestIOPackets = true;
  myTableSensingIR = true;
  myNewTableSensingIR = true;
  myFrontBumpers = true;
  myRearBumpers = true;
  myRobotWidth = 425;
  myRobotLength = 513;

  myNumIR = 4;
  internalSetIR(0, 1, 2, 333, -233);
  internalSetIR(1, 1, 2, 333, 233);
  internalSetIR(2, 1, 2, -2, -116);
  internalSetIR(3, 1, 2, -2, 116);

  myNumSonar = 32;

  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  internalSetSonar(24, -191, -136, -90);
  internalSetSonar(25, -237, -119, -130);
  internalSetSonar(26, -271, -78, -150);
  internalSetSonar(27, -290, -27, -170);
  internalSetSonar(28, -290, 27, 170);
  internalSetSonar(29, -271, 78, 150);
  internalSetSonar(30, -237, 119, 130);
  internalSetSonar(31, -191, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 21;
    getLaserData(1)->myLaserY = 0;
  }




  myPTZParams[0].setType("sony");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("pxc");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}



AREXPORT ArRobotPion1M::ArRobotPion1M(const char *dir)
{
  sprintf(mySubClass, "pion1m");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}

AREXPORT ArRobotPsos1M::ArRobotPsos1M(const char *dir)
{
  sprintf(mySubClass, "psos1m");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}

AREXPORT ArRobotPsos43M::ArRobotPsos43M(const char *dir)
{
  sprintf(mySubClass, "psos43m");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}


// PionAT robot class

AREXPORT ArRobotPionAT::ArRobotPionAT(const char *dir)
{
  sprintf(mySubClass, "pionat");
  myRobotRadius = 330;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 500;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.07;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}


AREXPORT ArRobotPion1X::ArRobotPion1X(const char *dir)
{
  sprintf(mySubClass, "pion1x");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}

AREXPORT ArRobotPsos1X::ArRobotPsos1X(const char *dir)
{
  sprintf(mySubClass, "psos1x");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}


AREXPORT ArRobotMapper::ArRobotMapper(const char *dir)
{
  sprintf(mySubClass, "mappr");
  myRobotRadius = 180;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 0;
  myAbsoluteMaxVelocity = 0;
  myHaveMoveCommand = false;
  myDistConvFactor = 1.00;
  //myDistConvFactor = 6.77; // solid foam tires
  // myDistConvFactor = 7.39; // pneumatic tires at 40 psi
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .011;
  myGyroScaler = 1.626; // the default used on Pioneers
  myVelConvFactor = 0.615400;
  mySwitchToBaudRate = 0;
  mySettableAccsDecs = false;
  mySettableVelMaxes = false;
  
  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 312; // the normal laser low position
    //getLaserData(1)->myLaserX = 237; // the high laser position
    getLaserData(1)->myLaserY = 0;
    
    getLaserData(1)->myLaserFlipped = false; // the normal configuration
    //getLaserData(1)->myLaserFlipped = true; // for low-inverted and high configurations
    getLaserData(1)->myLaserPowerControlled = false;
  }
}

// PowerBot robot class

AREXPORT ArRobotPowerBot::ArRobotPowerBot(const char *dir)
{

  sprintf(mySubClass, "powerbot");
  myRobotRadius = 550;
  myRobotDiagonal = 240; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 0.5813;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 911;
  myRobotLengthFront = 369;
  myRobotLengthRear = 542;
  
  myFrontBumpers = true;
  myNumFrontBumpers = 7;
  myRearBumpers = true;
  myNumRearBumpers = 5;
  myNumSonar = 32;
  internalSetSonar(0, 152, 278, 90);
  internalSetSonar(1, 200, 267, 65);
  internalSetSonar(2, 241, 238, 45);
  internalSetSonar(3, 274, 200, 35);
  internalSetSonar(4, 300, 153, 25);
  internalSetSonar(5, 320, 96, 15);
  internalSetSonar(6, 332, 33, 5);
  internalSetSonar(7, 0, 0, -180);

  internalSetSonar(8, 332, -33, -5);
  internalSetSonar(9, 320, -96, -15);
  internalSetSonar(10, 300, -153, -25);
  internalSetSonar(11, 274, -200, -35);
  internalSetSonar(12, 241, -238, -45);
  internalSetSonar(13, 200, -267, -65);
  internalSetSonar(14, 152, -278, -90);
  internalSetSonar(15, 0, 0, -180);

  internalSetSonar(16, -298, -278, -90);
  internalSetSonar(17, -347, -267, -115);
  internalSetSonar(18, -388, -238, -135);
  internalSetSonar(19, -420, -200, -145);
  internalSetSonar(20, -447, -153, -155);
  internalSetSonar(21, -467, -96, -165);
  internalSetSonar(22, -478, -33, -175);
  internalSetSonar(23, 0, 0, -180);

  internalSetSonar(24, -478, 33, 175);
  internalSetSonar(25, -467, 96, 165);
  internalSetSonar(26, -447, 153, 155);
  internalSetSonar(27, -420, 200, 145);
  internalSetSonar(28, -388, 238, 135);
  internalSetSonar(29, -347, 267, 115);
  internalSetSonar(30, -298, 278, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM2");
    getLaserData(1)->myLaserX = 251;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserFlipped = true;
  }

  sprintf(myGPSPort, "COM3"); // swap laser and hypothetical gps




  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("vfw");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}

AREXPORT ArRobotP2D8Plus::ArRobotP2D8Plus(const char *dir)
{
  sprintf(mySubClass, "p2d8+");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 18;
    getLaserData(1)->myLaserY = 0;
  }




  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}


AREXPORT ArRobotP2AT8Plus::ArRobotP2AT8Plus(const char *dir)
{
  sprintf(mySubClass, "p2at8+");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 0.465;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 160;
    getLaserData(1)->myLaserY = 7;
  }




  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}

AREXPORT ArRobotP3AT::ArRobotP3AT(const char *dir)
{
  sprintf(mySubClass, "p3at");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 0.465;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 160;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }

  myGPSX = -160;
  myGPSY = 120;
  sprintf(myGPSType, "novatel");




  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}


AREXPORT ArRobotP3DX::ArRobotP3DX(const char *dir)
{
  sprintf(mySubClass, "p3dx");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);


  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 18;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }


  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif
  myVideoParams[0].setConnect(true);

}



AREXPORT ArRobotPerfPBPlus::ArRobotPerfPBPlus(const char *dir)
{
  sprintf(mySubClass, "perfpb+");
  myRobotRadius = 340;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myRequestIOPackets = true;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .006;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;
  
  myRequestIOPackets = true;
  myTableSensingIR = true;
  myNewTableSensingIR = true;
  myFrontBumpers = true;
  myRearBumpers = true;

  myNumIR = 4;
  internalSetIR(0, 1, 2, 333, -233);
  internalSetIR(1, 1, 2, 333, 233);
  internalSetIR(2, 1, 2, -2, -116);
  internalSetIR(3, 1, 2, -2, 116);

  myNumSonar = 32;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  internalSetSonar(24, -191, -136, -90);
  internalSetSonar(25, -237, -119, -130);
  internalSetSonar(26, -271, -78, -150);
  internalSetSonar(27, -290, -27, -170);
  internalSetSonar(28, -290, 27, 170);
  internalSetSonar(29, -271, 78, 150);
  internalSetSonar(30, -237, 119, 130);
  internalSetSonar(31, -191, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 21;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }




  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setRobotAuxPort(1);
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}


AREXPORT ArRobotP3DXSH::ArRobotP3DXSH(const char *dir)
{
  sprintf(mySubClass, "p3dx-sh");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 21;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }

  myGPSX = -160;
  myGPSY = 120;
  sprintf(myGPSType, "novatel");



  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true);
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}


AREXPORT ArRobotP3ATSH::ArRobotP3ATSH(const char *dir)
{
  sprintf(mySubClass, "p3at-sh");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 125;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }
  myGPSX = -160;
  myGPSY = 120;
  sprintf(myGPSType, "novatel");




  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true);
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}


AREXPORT ArRobotP3ATIWSH::ArRobotP3ATIWSH(const char *dir)
{
  sprintf(mySubClass, "p3atiw-sh");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 490;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 125;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }

  myGPSX = -160;
  myGPSY = 120;
  sprintf(myGPSType, "novatel");




  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true);
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}


AREXPORT ArRobotPatrolBotSH::ArRobotPatrolBotSH(const char *dir)
{
  sprintf(mySubClass, "patrolbot-sh");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 510;
  myRobotLengthFront = 255;
  myRobotLengthRear = 255;

  myFrontBumpers = true;
  myNumFrontBumpers = 6;
  myRearBumpers = true;
  myNumRearBumpers = 6;

  myNumSonar = 16;
  internalSetSonar(0, 83, 229, 90);
  internalSetSonar(1, 169, 202, 55);
  internalSetSonar(2, 232, 134, 30);
  internalSetSonar(3, 263, 46, 10);
  internalSetSonar(4, 263, -46, -10);
  internalSetSonar(5, 232, -134, -30);
  internalSetSonar(6, 169, -202, -55);
  internalSetSonar(7, 83, -229, -90);

  internalSetSonar(8, -83, -229, -90);
  internalSetSonar(9, -169, -202, -125);
  internalSetSonar(10, -232, -134, -150);
  internalSetSonar(11, -263, -46, -170);
  internalSetSonar(12, -263, 46, 170);
  internalSetSonar(13, -232, 134, 150);
  internalSetSonar(14, -169, 202, 125);
  internalSetSonar(15, -83, 229, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 37;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
    getLaserData(1)->myLaserFlipped = true;
    getLaserData(1)->myLaserPowerControlled = false;
    strcpy(getLaserData(1)->myLaserIgnore, 
	   "-76--72 72-76");
  }

  if (getLaserData(2) != NULL)
  {
    sprintf(getLaserData(2)->myLaserType, "urg");
    sprintf(getLaserData(2)->myLaserPortType, "serial");
    sprintf(getLaserData(2)->myLaserPort, "COM5");
    getLaserData(2)->myLaserX = 200;
    getLaserData(2)->myLaserY = 0;
    getLaserData(2)->myLaserFlipped = true;
    sprintf(getLaserData(2)->myLaserStartDegrees, "-55");
    sprintf(getLaserData(2)->myLaserEndDegrees, "55");
    sprintf(getLaserData(2)->myLaserIncrement, "1.0");
  }


  myRequestIOPackets = true;




  myPTZParams[0].setType("vcc4");
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true);
}


AREXPORT ArRobotPeopleBotSH::ArRobotPeopleBotSH(const char *dir)
{
  sprintf(mySubClass, "peoplebot-sh");
  myRobotRadius = 340;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myRequestIOPackets = true;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .006;
  myRobotWidth = 425;
  myRobotLength = 513;
  
  myRequestIOPackets = true;
  myTableSensingIR = true;
  myNewTableSensingIR = true;
  myFrontBumpers = true;
  myRearBumpers = true;

  myNumIR = 4;
  internalSetIR(0, 1, 2, 333, -233);
  internalSetIR(1, 1, 2, 333, 233);
  internalSetIR(2, 1, 2, -2, -116);
  internalSetIR(3, 1, 2, -2, 116);

  myNumSonar = 32;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  internalSetSonar(16, -20, 136, 90);
  internalSetSonar(17, 24, 119, 50);
  internalSetSonar(18, 58, 78, 30);
  internalSetSonar(19, 77, 27, 10);
  internalSetSonar(20, 77, -27, -10);
  internalSetSonar(21, 58, -78, -30);
  internalSetSonar(22, 24, -119, -50);
  internalSetSonar(23, -20, -136, -90);

  internalSetSonar(24, -191, -136, -90);
  internalSetSonar(25, -237, -119, -130);
  internalSetSonar(26, -271, -78, -150);
  internalSetSonar(27, -290, -27, -170);
  internalSetSonar(28, -290, 27, 170);
  internalSetSonar(29, -271, 78, 150);
  internalSetSonar(30, -237, 119, 130);
  internalSetSonar(31, -191, 136, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 21;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
  }




  myPTZParams[0].setType("vapix");
  myPTZParams[0].setInverted(true);
  myVideoParams[0].setType("vapix");  
  myVideoParams[0].setConnect(true);
  myVideoParams[0].setInverted(true);
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}


AREXPORT ArRobotPowerBotSH::ArRobotPowerBotSH(const char *dir)
{

  sprintf(mySubClass, "powerbot-sh");
  myRobotRadius = 550;
  myRobotDiagonal = 240; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 911;
  myRobotLengthFront = 369;
  myRobotLengthRear = 542;

  myFrontBumpers = true;
  myNumFrontBumpers = 7;
  myRearBumpers = true;
  myNumRearBumpers = 5;
  myNumSonar = 32;
  internalSetSonar(0, 152, 278, 90);
  internalSetSonar(1, 200, 267, 65);
  internalSetSonar(2, 241, 238, 45);
  internalSetSonar(3, 274, 200, 35);
  internalSetSonar(4, 300, 153, 25);
  internalSetSonar(5, 320, 96, 15);
  internalSetSonar(6, 332, 33, 5);
  internalSetSonar(7, 0, 0, -180);

  internalSetSonar(8, 332, -33, -5);
  internalSetSonar(9, 320, -96, -15);
  internalSetSonar(10, 300, -153, -25);
  internalSetSonar(11, 274, -200, -35);
  internalSetSonar(12, 241, -238, -45);
  internalSetSonar(13, 200, -267, -65);
  internalSetSonar(14, 152, -278, -90);
  internalSetSonar(15, 0, 0, -180);

  internalSetSonar(16, -298, -278, -90);
  internalSetSonar(17, -347, -267, -115);
  internalSetSonar(18, -388, -238, -135);
  internalSetSonar(19, -420, -200, -145);
  internalSetSonar(20, -447, -153, -155);
  internalSetSonar(21, -467, -96, -165);
  internalSetSonar(22, -478, -33, -175);
  internalSetSonar(23, 0, 0, -180);

  internalSetSonar(24, -478, 33, 175);
  internalSetSonar(25, -467, 96, 165);
  internalSetSonar(26, -447, 153, 155);
  internalSetSonar(27, -420, 200, 145);
  internalSetSonar(28, -388, 238, 135);
  internalSetSonar(29, -347, 267, 115);
  internalSetSonar(30, -298, 278, 90);
  internalSetSonar(31, 0, 0, -180);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM2");
    getLaserData(1)->myLaserX = 251;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserFlipped = true;
    getLaserData(1)->myLaserAutoConnect = true;
  }

  sprintf(myGPSPort, "COM3"); // swap laser and hypothetical GPS




  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true);
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}


AREXPORT ArRobotPowerBotSHuARCS::ArRobotPowerBotSHuARCS(const char *dir)
{

  sprintf(mySubClass, "powerbot-sh-uarcs");
  myRobotRadius = 550;
  myRobotDiagonal = 240; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 911;
  myRobotLengthFront = 369;
  myRobotLengthRear = 542;

  myFrontBumpers = true;
  myNumFrontBumpers = 7;
  myRearBumpers = true;
  myNumRearBumpers = 5;
  myNumSonar = 16;

  internalSetSonar(0, -298, -278, -90);
  internalSetSonar(1, -347, -267, -115);
  internalSetSonar(2, -388, -238, -135);
  internalSetSonar(3, -420, -200, -145);
  internalSetSonar(4, -447, -153, -155);
  internalSetSonar(5, -467, -96, -165);
  internalSetSonar(6, -478, -33, -175);
  internalSetSonar(7, 0, 0, -180);

  internalSetSonar(8, -478, 33, 175);
  internalSetSonar(9, -467, 96, 165);
  internalSetSonar(10, -447, 153, 155);
  internalSetSonar(11, -420, 200, 145);
  internalSetSonar(12, -388, 238, 135);
  internalSetSonar(13, -347, 267, 115);
  internalSetSonar(14, -298, 278, 90);
  internalSetSonar(15, 0, 0, -180);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM2");
    getLaserData(1)->myLaserX = 251;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserFlipped = true;
    getLaserData(1)->myLaserAutoConnect = true;
  }

  sprintf(myGPSPort, "COM3"); // swap laser and hypothetical GPS
}

AREXPORT ArRobotWheelchairSH::ArRobotWheelchairSH(const char *dir)
{
  sprintf(mySubClass, "wheelchair-sh");
  myRobotRadius = 550;
  myRobotDiagonal = 300; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 1340;
  
  myFrontBumpers = true;
  myNumFrontBumpers = 4;
  myRearBumpers = true;
  myNumRearBumpers = 3;
  myNumSonar = 0;

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM2");
    getLaserData(1)->myLaserX = -418;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserFlipped = true;
    getLaserData(1)->myLaserPowerControlled = true;
    getLaserData(1)->myLaserAutoConnect = true;
  }

  mySettableAccsDecs = true;
  mySettableVelMaxes = false;
  myTransVelMax = 0;
  myRotVelMax = 0;
  myTransAccel = 0;
  myTransDecel = 0;
  myRotAccel = 0;
  myRotDecel = 0;

  sprintf(myGPSPort, "COM3"); // swap laser and hypothetical GPS
}

AREXPORT ArRobotSeekur::ArRobotSeekur(const char *dir)
{
  sprintf(mySubClass, "seekur");
  myRobotRadius = 833;
  myRobotDiagonal = 400; 
  myAbsoluteMaxRVelocity = 190;
  myAbsoluteMaxVelocity = 2200;
  myAbsoluteMaxLatVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0056;
  myRobotWidth = 1270;
  myRobotLength = 1410;
  myRobotLengthFront = myRobotLength/2.0;
  myRobotLengthRear = myRobotLength/2.0;

  myHaveMoveCommand = 0;  
  myFrontBumpers = true;
  myNumFrontBumpers = 5;
  myRearBumpers = true;
  myNumRearBumpers = 3;
  myNumSonar = 0;

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 690;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserFlipped = false;
    getLaserData(1)->myLaserPowerControlled = true;
    getLaserData(1)->myLaserAutoConnect = true;

  }

  mySettableAccsDecs = true;
  myTransVelMax = 0;
  myRotVelMax = 0;
  myTransAccel = 0;
  myTransDecel = 0;
  myRotAccel = 0;
  myRotDecel = 0;

  myHasLatVel = true;

  myGPSX = -200;
  myGPSY = 0;
  sprintf(myGPSPort, "COM2"); 
  sprintf(myGPSType, "trimble");
  myGPSBaud = 38400;




  myPTZParams[0].setType("rvision");
  myPTZParams[0].setConnect(true);
  myPTZParams[0].setSerialPort("COM4");
  myPTZParams[0].setAddress("none");
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif  
  myVideoParams[0].setConnect(true); 
}

AREXPORT ArRobotMT400::ArRobotMT400(const char *dir)
{
  sprintf(mySubClass, "mt400");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 510;
  myRobotLengthFront = 255;
  myRobotLengthRear = 255;

  myFrontBumpers = true;
  myNumFrontBumpers = 6;
  myRearBumpers = true;
  myNumRearBumpers = 6;

  myNumSonar = 16;
  internalSetSonar(0, 83, 229, 90);
  internalSetSonar(1, 169, 202, 55);
  internalSetSonar(2, 232, 134, 30);
  internalSetSonar(3, 263, 46, 10);
  internalSetSonar(4, 263, -46, -10);
  internalSetSonar(5, 232, -134, -30);
  internalSetSonar(6, 169, -202, -55);
  internalSetSonar(7, 83, -229, -90);

  internalSetSonar(8, -83, -229, -90);
  internalSetSonar(9, -169, -202, -125);
  internalSetSonar(10, -232, -134, -150);
  internalSetSonar(11, -263, -46, -170);
  internalSetSonar(12, -263, 46, 170);
  internalSetSonar(13, -232, 134, 150);
  internalSetSonar(14, -169, 202, 125);
  internalSetSonar(15, -83, 229, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 37;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
    getLaserData(1)->myLaserFlipped = true;
    getLaserData(1)->myLaserPowerControlled = false;
    strcpy(getLaserData(1)->myLaserIgnore, 
	   "-76--72 72-76");
  }

  if (getLaserData(2) != NULL)
  {
    sprintf(getLaserData(2)->myLaserType, "urg");
    sprintf(getLaserData(2)->myLaserPortType, "serial");
    sprintf(getLaserData(2)->myLaserPort, "COM5");
    getLaserData(2)->myLaserX = 200;
    getLaserData(2)->myLaserY = 0;
    getLaserData(2)->myLaserFlipped = true;
    sprintf(getLaserData(2)->myLaserStartDegrees, "-120");
    sprintf(getLaserData(2)->myLaserEndDegrees, "120");
    sprintf(getLaserData(2)->myLaserIncrement, "1.0");
  }


  myRequestIOPackets = true;




  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true); 
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}

AREXPORT ArRobotResearchPB::ArRobotResearchPB(const char *dir)
{
  sprintf(mySubClass, "researchPB");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 510;
  myRobotLengthFront = 255;
  myRobotLengthRear = 255;

  myFrontBumpers = true;
  myNumFrontBumpers = 6;
  myRearBumpers = true;
  myNumRearBumpers = 6;

  myNumSonar = 16;
  internalSetSonar(0, 83, 229, 90);
  internalSetSonar(1, 169, 202, 55);
  internalSetSonar(2, 232, 134, 30);
  internalSetSonar(3, 263, 46, 10);
  internalSetSonar(4, 263, -46, -10);
  internalSetSonar(5, 232, -134, -30);
  internalSetSonar(6, 169, -202, -55);
  internalSetSonar(7, 83, -229, -90);

  internalSetSonar(8, -83, -229, -90);
  internalSetSonar(9, -169, -202, -125);
  internalSetSonar(10, -232, -134, -150);
  internalSetSonar(11, -263, -46, -170);
  internalSetSonar(12, -263, 46, 170);
  internalSetSonar(13, -232, 134, 150);
  internalSetSonar(14, -169, 202, 125);
  internalSetSonar(15, -83, 229, 90);

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms2xx");
    sprintf(getLaserData(1)->myLaserPortType, "serial");
    sprintf(getLaserData(1)->myLaserPort, "COM3");
    getLaserData(1)->myLaserX = 37;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserAutoConnect = true;
    getLaserData(1)->myLaserFlipped = true;
    getLaserData(1)->myLaserPowerControlled = false;
    strcpy(getLaserData(1)->myLaserIgnore, 
	   "-76--72 72-76");
  }

  if (getLaserData(2) != NULL)
  {
    sprintf(getLaserData(2)->myLaserType, "urg");
    sprintf(getLaserData(2)->myLaserPortType, "serial");
    sprintf(getLaserData(2)->myLaserPort, "COM5");
    getLaserData(2)->myLaserX = 200;
    getLaserData(2)->myLaserY = 0;
    getLaserData(2)->myLaserFlipped = true;
    sprintf(getLaserData(2)->myLaserStartDegrees, "-55");
    sprintf(getLaserData(2)->myLaserEndDegrees, "55");
    sprintf(getLaserData(2)->myLaserIncrement, "1.0");
  }


  myRequestIOPackets = true;




  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true); 
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");
}

AREXPORT ArRobotSeekurJr::ArRobotSeekurJr(const char *dir)
{
  sprintf(mySubClass, "seekurjr");
  myRobotRadius = 600;
  myRobotDiagonal = 600; 
  myAbsoluteMaxRVelocity = 80;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0056;
  myRobotWidth = 830;
  myRobotLength = 1200;
  myRobotLengthFront = myRobotLength/2.0;
  myRobotLengthRear = myRobotLength/2.0;

  myHaveMoveCommand = 0;  
  myFrontBumpers = true;
  myNumFrontBumpers = 4;
  myRearBumpers = true;
  myNumRearBumpers = 4;
  myNumSonar = 0;

  if (getLaserData(1) != NULL)
  {
    sprintf(getLaserData(1)->myLaserType, "lms1XX");
    sprintf(getLaserData(1)->myLaserPortType, "tcp");
    sprintf(getLaserData(1)->myLaserPort, "192.168.0.1");
    getLaserData(1)->myLaserX = 527;
    getLaserData(1)->myLaserY = 0;
    getLaserData(1)->myLaserFlipped = false;
    getLaserData(1)->myLaserPowerControlled = false;
    sprintf(getLaserData(1)->myLaserStartDegrees, "-117");
    sprintf(getLaserData(1)->myLaserEndDegrees, "117");
    getLaserData(1)->myLaserAutoConnect = true;
  }

  if (getLaserData(2) != NULL)
  {
    sprintf(getLaserData(2)->myLaserType, "lms1XX");
    sprintf(getLaserData(2)->myLaserPortType, "tcp");
    sprintf(getLaserData(2)->myLaserPort, "192.168.0.2");
    getLaserData(2)->myLaserX = -527;
    getLaserData(2)->myLaserY = 0;
    getLaserData(2)->myLaserFlipped = false;
    getLaserData(2)->myLaserPowerControlled = false;
    sprintf(getLaserData(2)->myLaserStartDegrees, "-117");
    sprintf(getLaserData(2)->myLaserEndDegrees, "117");
  }

  mySettableAccsDecs = true;
  myTransVelMax = 0;
  myRotVelMax = 0;
  myTransAccel = 0;
  myTransDecel = 0;
  myRotAccel = 0;
  myRotDecel = 0;

  myGPSX = 0;
  myGPSY = 0;
  sprintf(myGPSPort, "COM2"); 
  sprintf(myGPSType, "trimble");
  myGPSBaud = 38400;

  myPTZParams[0].setType("rvision");
  myPTZParams[0].setSerialPort("COM4");
  myPTZParams[0].setConnect(true);
#ifdef WIN32
  myVideoParams[0].setType("sx11");
#else
  myVideoParams[0].setType("v4l");
#endif   
  myVideoParams[0].setConnect(true); 
}

AREXPORT ArRobotP3DXSH_lms1xx::ArRobotP3DXSH_lms1xx(const char *dir) :
  ArRobotP3DXSH(dir)
{
  sprintf(mySubClass, "p3dx-sh-lms1xx");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms1XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 55;
  }
}

AREXPORT ArRobotP3ATSH_lms1xx::ArRobotP3ATSH_lms1xx(const char *dir) :
  ArRobotP3ATSH(dir)
{
  sprintf(mySubClass, "p3at-sh-lms1xx");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms1XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 197;
  }
}

AREXPORT ArRobotPeopleBotSH_lms1xx::ArRobotPeopleBotSH_lms1xx(const char *dir) :
  ArRobotPeopleBotSH(dir)
{
  sprintf(mySubClass, "peoplebot-sh-lms1xx");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms1XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 55;
  }
}

AREXPORT ArRobotP3DXSH_lms500::ArRobotP3DXSH_lms500(const char *dir) :
  ArRobotP3DXSH(dir)
{
  sprintf(mySubClass, "p3dx-sh-lms500");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms5XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 21;
    ld->myLaserAutoConnect = true;
  }
}

AREXPORT ArRobotP3ATSH_lms500::ArRobotP3ATSH_lms500(const char *dir) :
  ArRobotP3ATSH(dir)
{
  sprintf(mySubClass, "p3at-sh-lms500");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms5XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 125;
  }
}

AREXPORT ArRobotPeopleBotSH_lms500::ArRobotPeopleBotSH_lms500(const char *dir) :
  ArRobotPeopleBotSH(dir)
{
  sprintf(mySubClass, "peoplebot-sh-lms500");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms5XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 21;
  }
}

AREXPORT ArRobotPowerBotSH_lms500::ArRobotPowerBotSH_lms500(const char *dir) :
  ArRobotPowerBotSH(dir)
{
  sprintf(mySubClass, "powerbot-sh-lms500");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms5XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 251;
  }
}

AREXPORT ArRobotResearchPB_lms500::ArRobotResearchPB_lms500(const char *dir) :
  ArRobotResearchPB(dir)
{
  sprintf(mySubClass, "researchPB-lms500");
  if(LaserData* ld = getLaserData(1))
  {
    sprintf(ld->myLaserType, "lms5XX");
    sprintf(ld->myLaserPortType, "tcp");
    sprintf(ld->myLaserPort, "192.168.0.1");
    ld->myLaserX = 37;
  }
}

AREXPORT ArRobotPioneerLX::ArRobotPioneerLX(const char *dir)
{
  sprintf(myClass, "MTX");
  sprintf(mySubClass, "pioneer-lx");
  myRobotRadius = 348;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 180;
  myAbsoluteMaxVelocity = 2500;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myVelConvFactor = 1.0;
  myRobotWidth = 500;
  myRobotLength = 696;
  myRobotLengthFront = 348;
  myRobotLengthRear = 348;

  myFrontBumpers = true;
  myNumFrontBumpers = 3;
  myRearBumpers = true;
  myNumRearBumpers = 3;

  mySettableVelMaxes = true;
  myTransVelMax = 1800;
  myRotVelMax = 180;
  mySettableAccsDecs = true;
  myTransAccel = 500;
  myTransDecel = 600;
  myRotAccel = 150;
  myRotDecel = 200;

  myHaveMoveCommand = false;

  LaserData *laser = NULL;
  if ( (laser = getLaserData(1)) != NULL)
  {
    sprintf(laser->myLaserType, "s3series");
    sprintf(laser->myLaserPortType, "serial422");
#ifdef WIN32
	sprintf(laser->myLaserPort, ArUtil::COM7);
#else
	sprintf(laser->myLaserPort, "/dev/ttyUSB4");
#endif
    sprintf(laser->myLaserStartingBaudChoice, "230400");
    laser->myLaserX = 267;
    laser->myLaserY = 0;
    laser->myLaserZ = 190;
    laser->myLaserFlipped = true;
    laser->myLaserPowerControlled = false;
    strcpy(laser->myLaserIgnore, 
	   "-135--120 120-135");
    laser->myLaserAutoConnect = true;
  }
 
  if(LCDMTXBoardData *lcd = getLCDMTXBoardData(1))
  {
    sprintf(lcd->myLCDMTXBoardType, "mtx");
    sprintf(lcd->myLCDMTXBoardPortType, "serial422");
#ifdef WIN32
	sprintf(lcd->myLCDMTXBoardPort, ArUtil::COM11);
#else
    sprintf(lcd->myLCDMTXBoardPort, "/dev/ttyUSB8");
#endif
    lcd->myLCDMTXBoardBaud = 115200;
    lcd->myLCDMTXBoardAutoConn = true;
  }
 
  if(SonarMTXBoardData *sonar = getSonarMTXBoardData(1))
  {
    sprintf(sonar->mySonarMTXBoardType, "mtx");
    sprintf(sonar->mySonarMTXBoardPortType, "serial422");
#ifdef WIN32
	  sprintf(sonar->mySonarMTXBoardPort, ArUtil::COM5);
#else
    sprintf(sonar->mySonarMTXBoardPort, "/dev/ttyUSB2");
#endif
    sonar->mySonarMTXBoardBaud = 115200;
    sonar->mySonarMTXBoardAutoConn = true;
    sonar->myNumSonarTransducers = 4;
    sonar->mySonarDelay = 2;
    sonar->mySonarGain = 10;
    sonar->mySonarDetectionThreshold = 25;
    sonar->mySonarMaxRange = 4335;
    sprintf(sonar->mySonarMTXBoardPowerOutput, "Sonar_1_Power");
  }
 
  /*
  if(SonarMTXBoardData *sonar = getSonarMTXBoardData(2))
  {
    sprintf(sonar->mySonarMTXBoardType, "mtx");
    sprintf(sonar->mySonarMTXBoardPortType, "serial422");
#ifdef WIN32
    sprintf(sonar->mySonarMTXBoardPort, ArUtil::COM6);
#else
    sprintf(sonar->mySonarMTXBoardPort, "/dev/ttyUSB3");
#endif
    sonar->mySonarMTXBoardBaud = 115200;
    sonar->mySonarMTXBoardAutoConn = false; 
    sonar->myNumSonarTransducers = 8;
  }
  */

  myNumSonar = 4;
  myNumSonarUnits = 4;
  //          sonar#     x    y    th board# unit# gain thresh   max
  // Front:
  internalSetSonar(0,  331,  61,   10,     1,    1,   0,   600,  400);
  internalSetSonar(1,  331, -61,  -10,     1,    2,   0,   600,  400);
  // back:
  internalSetSonar(2, -317,  90,  164,     1,    3,   0,   500,  500);
  internalSetSonar(3, -317, -90, -164,     1,    4,   0,   500,  500);

  if(BatteryMTXBoardData *bat = getBatteryMTXBoardData(1))
  {
    sprintf(bat->myBatteryMTXBoardType, "mtx");
    sprintf(bat->myBatteryMTXBoardPortType, "serial");
#ifdef WIN32
	sprintf(bat->myBatteryMTXBoardPort, ArUtil::COM4);
#else
    sprintf(bat->myBatteryMTXBoardPort, "/dev/ttyUSB1");
#endif
    bat->myBatteryMTXBoardBaud = 115200;
    bat->myBatteryMTXBoardAutoConn = true;
  }

  myRequestIOPackets = true;

  myPTZParams[0].setType("vapix");
  myPTZParams[0].setConnect(true);
  myVideoParams[0].setType("vapix");   
  myVideoParams[0].setConnect(true);
  myPTZParams[0].setAddress("192.168.0.90");
  myVideoParams[0].setAddress("192.168.0.90");


}


AREXPORT ArRobotPioneerLX_LD::ArRobotPioneerLX_LD(const char *dir) :
  ArRobotPioneerLX(dir)
{
  sprintf(mySubClass, "pioneer-lx-ld");
  if(LaserData *l = getLaserData(2))
  {
    l->myLaserAutoConnect = true;
    sprintf(l->myLaserType, "tim510");
    sprintf(l->myLaserPortType, "serial");
#ifdef WIN32
    sprintf(l->myLaserPort, ArUtil::COM10);
#else
    sprintf(l->myLaserPort, "/dev/ttyUSB7");
#endif
    l->myLaserX = 267;
    l->myLaserY = 0;
    l->myLaserZ = 58;
    l->myLaserFlipped = true;
    l->myLaserPowerControlled = false;
    sprintf(l->myLaserStartDegrees, "-61");
    sprintf(l->myLaserEndDegrees, "61");
  }

  internalSetSonarUseFlag(0, false);
  internalSetSonarUseFlag(1, false);
}
   

/** @endcond INCLUDE_INTERNAL_ROBOT_PARAM_CLASSES */
