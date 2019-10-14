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
/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.mobilerobots.ArNetworking;
import com.mobilerobots.Aria.*;
public class ArClientFileToClient {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArClientFileToClient(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArClientFileToClient obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArNetworkingJavaJNI.delete_ArClientFileToClient(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArClientFileToClient(ArClientBase client) {
    this(ArNetworkingJavaJNI.new_ArClientFileToClient(ArClientBase.getCPtr(client), client), true);
  }

  public boolean isAvailable() {
    return ArNetworkingJavaJNI.ArClientFileToClient_isAvailable(swigCPtr, this);
  }

  public boolean isAvailableSetTimestamp() {
    return ArNetworkingJavaJNI.ArClientFileToClient_isAvailableSetTimestamp(swigCPtr, this);
  }

  public boolean getFileFromDirectory(String directory, String fileName, String clientFileName, boolean isSetTimestamp) {
    return ArNetworkingJavaJNI.ArClientFileToClient_getFileFromDirectory__SWIG_0(swigCPtr, this, directory, fileName, clientFileName, isSetTimestamp);
  }

  public boolean getFileFromDirectory(String directory, String fileName, String clientFileName) {
    return ArNetworkingJavaJNI.ArClientFileToClient_getFileFromDirectory__SWIG_1(swigCPtr, this, directory, fileName, clientFileName);
  }

  public void cancelGet() {
    ArNetworkingJavaJNI.ArClientFileToClient_cancelGet(swigCPtr, this);
  }

  public boolean isWaitingForFile() {
    return ArNetworkingJavaJNI.ArClientFileToClient_isWaitingForFile(swigCPtr, this);
  }

  public String getDirectory() {
    return ArNetworkingJavaJNI.ArClientFileToClient_getDirectory(swigCPtr, this);
  }

  public String getFileName() {
    return ArNetworkingJavaJNI.ArClientFileToClient_getFileName(swigCPtr, this);
  }

  public String getClientFileName() {
    return ArNetworkingJavaJNI.ArClientFileToClient_getClientFileName(swigCPtr, this);
  }

  public void addFileReceivedCallback(ArFunctor1_Int functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArClientFileToClient_addFileReceivedCallback__SWIG_0(swigCPtr, this, ArFunctor1_Int.getCPtr(functor), functor, position.swigValue());
  }

  public void addFileReceivedCallback(ArFunctor1_Int functor) {
    ArNetworkingJavaJNI.ArClientFileToClient_addFileReceivedCallback__SWIG_1(swigCPtr, this, ArFunctor1_Int.getCPtr(functor), functor);
  }

  public void remFileReceivedCallback(ArFunctor1_Int functor) {
    ArNetworkingJavaJNI.ArClientFileToClient_remFileReceivedCallback(swigCPtr, this, ArFunctor1_Int.getCPtr(functor), functor);
  }

  public ArTime getLastReceived() {
    return new ArTime(ArNetworkingJavaJNI.ArClientFileToClient_getLastReceived(swigCPtr, this), true);
  }

  public ArTime getLastRequested() {
    return new ArTime(ArNetworkingJavaJNI.ArClientFileToClient_getLastRequested(swigCPtr, this), true);
  }

}
