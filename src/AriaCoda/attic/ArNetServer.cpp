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
#include <ctype.h>
#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArNetServer.h"
#include "ArRobot.h"
#include "ArLog.h"
#include "ariaUtil.h"
#include "ArSyncTask.h"
#include "ArArgumentBuilder.h"
#include "ariaInternal.h"


ArNetServer::ArNetServer(bool addAriaExitCB, bool doNotAddShutdownServer,
			 const char *name, ArNetServer *childServer) :
  myTaskCB(this, &ArNetServer::runOnce),
  myHelpCB(this, &ArNetServer::internalHelp),
  myEchoCB(this, &ArNetServer::internalEcho),
  myQuitCB(this, &ArNetServer::internalQuit),
  myShutdownServerCB(this, &ArNetServer::internalShutdownServer),
  myAriaExitCB(this, &ArNetServer::close)
{
  if (name != NULL)
    myName = name;
  else
    myName = "ArNetServer";
  myChildServer = childServer;
  myMutex.setLogName((myName + "::myMutex").c_str());
  myRobot = NULL;
  myPort = 0;
  myMultipleClients = false;
  myOpened = false;
  myWantToClose = false;
  myLoggingDataSent = false;
  myLoggingDataReceived = false;
  mySquelchNormal = false;
  addCommand("help", &myHelpCB, "gives the listing of available commands");
  addCommand("echo", &myEchoCB, "with no args gets echo, with args sets echo");
  addCommand("quit", &myQuitCB, "closes this connection to the server");
  // MPL 2013_06_10 letting folks take out shutdownServer since it
  // can do no good and much ill
  if (!doNotAddShutdownServer)
    addCommand("shutdownServer", &myShutdownServerCB, "shuts down the server");

  myMutex.setLogName((myName + "::myMutex").c_str());
  myNextCycleSendsMutex.setLogName((myName + "::myNextCycleSendsMutex").c_str());
  
  myAriaExitCB.setName((myName + "::ariaExit").c_str());
  if (addAriaExitCB)
    Aria::addExitCallback(&myAriaExitCB, 40);
}

ArNetServer::~ArNetServer()
{
  ArSyncTask *rootTask = NULL;
  ArSyncTask *proc = NULL;
  // get rid of us running on the robot task
  if (myRobot != NULL && (rootTask = myRobot->getSyncTaskRoot()) != NULL)
  {
    proc = rootTask->findNonRecursive(&myTaskCB);
    if (proc != NULL)
      delete proc;
  }
  close();
}

/**
   Open the server, if you supply a robot this will run in the robots
   attached, if you do not supply a robot then it will be open and
   you'll have to call runOnce yourself (this is only recommended for
   advanced users)

   @param robot the robot that this should be attached to and run in
   the sync task of or NULL not to run in any robot's task

   @param port the port to start up the service on

   @param password the password needed to use the service

   @param multipleClients if false only one client is allowed to connect, 
   if false multiple clients are allowed to connect or just one

   @param openOnIP If not NULL, restrict server port to the network interface
   with this IP address. If NULL, accept connections from any network interface.

   @return true if the server could be started, false otherwise
**/
AREXPORT bool ArNetServer::open(ArRobot *robot, unsigned int port, 
				const char *password, bool multipleClients,
				const char *openOnIP)
{
  ArSyncTask *rootTask = NULL;
  ArSyncTask *proc = NULL;
  std::string taskName;

  if (myOpened)
  {
    ArLog::log(ArLog::Terse, "%s already inited, cannot reinit",
	       myName.c_str());
    return false;
  }

  myRobot = robot;
  myPort = port;
  myPassword = password;
  myMultipleClients = multipleClients;
  
  if (myServerSocket.open(myPort, ArSocket::TCP, openOnIP))
  {
    // this can be taken out since the open does this now
    //myServerSocket.setLinger(0);
    myServerSocket.setNonBlock();
    if (openOnIP != NULL)
      ArLog::log(ArLog::Normal, "%s opened on port %d on ip %s.", 
		 myName.c_str(), myPort, openOnIP);
    else
      ArLog::log(ArLog::Normal, "%s opened on port %d.", 
		 myName.c_str(), myPort);
    myOpened = true;
  }
  else
  {
    ArLog::log(ArLog::Terse, "%s failed to open: %s", 
	       myName.c_str(), myServerSocket.getErrorStr().c_str());
    myOpened = false;
    return false;
  }

  // add ourselves to the robot if we aren't already there
  if (myRobot != NULL && (rootTask = myRobot->getSyncTaskRoot()) != NULL)
  {    
    proc = rootTask->findNonRecursive(&myTaskCB);
    if (proc == NULL)
    {
      // toss in a netserver (it used to say the port name, but did it wrong so put in gibberish)
      taskName = myName; 
      rootTask->addNewLeaf(taskName.c_str(), 60, &myTaskCB, NULL);
    }
  }
  return true;
  
}

/**
   This adds a command to the list, when the command is given the
   broken up argv and argc are given along with the socket it came
   from (so that acks can occur)
**/
AREXPORT bool ArNetServer::addCommand(const char *command, 
			      ArFunctor3<char **, int, ArSocket *> *functor,
				      const char *help)
{
  if (myChildServer != NULL)
    myChildServer->addCommand(command, functor, help); 

  std::map<std::string, ArFunctor3<char **, int, ArSocket *> *, ArStrCaseCmpOp>::iterator it;


  if ((it = myFunctorMap.find(command)) != myFunctorMap.end())
  {
    ArLog::log(ArLog::Normal, "%s::addCommand: Already a command for %s", myName.c_str(), command);
    return false;
  }

  myFunctorMap[command] = functor;
  myHelpMap[command] = help;


  return true;
}

/**
   @param command the command to remove
   @return true if the command was there to remove, false otherwise
**/
AREXPORT bool ArNetServer::remCommand(const char *command)
{
  if (myChildServer != NULL)
    myChildServer->remCommand(command);

  if (myFunctorMap.find(command) == myFunctorMap.end())
  {
    return false;
  }

  myFunctorMap.erase(command);
  myHelpMap.erase(command);
  return true;  
}


AREXPORT void ArNetServer::sendToAllClientsPlain(const char *str)
{
  if (myChildServer != NULL)


    myChildServer->sendToAllClientsPlain(str);

  std::list<ArSocket *>::iterator it;

  if (myLoggingDataSent)
    ArLog::log(ArLog::Terse, "%s::sendToAllClients: Sending %s", 
	       myName.c_str(), str);

  for (it = myConns.begin(); it != myConns.end(); ++it)
  {
    (*it)->setLogWriteStrings(false);
    (*it)->writeString(str);
    (*it)->setLogWriteStrings(myLoggingDataSent);
  }
}


/**
   This sends the given string to all the clients, this string cannot
   be more than 2048 number of bytes
**/
AREXPORT void ArNetServer::sendToAllClients(const char *str, ...)
{
  char buf[40000];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);

  sendToAllClientsPlain(buf);
  
  va_end(ptr);
}



AREXPORT void ArNetServer::sendToAllClientsNextCyclePlain(const char *str)
{
  if (myChildServer != NULL)
    myChildServer->sendToAllClientsNextCyclePlain(str);

  std::list<ArSocket *>::iterator it;

  if (myLoggingDataSent)
    ArLog::log(ArLog::Terse, "%s::sendToAllClientsNextCycle: Next cycle will send: %s", myName.c_str(), str);

  myNextCycleSendsMutex.lock();
  myNextCycleSends.push_back(str);
  myNextCycleSendsMutex.unlock();
}

AREXPORT bool ArNetServer::sendToAllClientsNextCyclePlainBool(const char *str)
{
  sendToAllClientsNextCyclePlain(str);
  return true;
}

/**
   This sends the given string to all the clients, this string cannot
   be more than 2048 number of bytes
**/
AREXPORT void ArNetServer::sendToAllClientsNextCycle(const char *str, ...)
{
  char buf[40000];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);

  sendToAllClientsNextCyclePlain(buf);
  
  va_end(ptr);
}


AREXPORT bool ArNetServer::isOpen(void)
{
  return myOpened;
}


/**
   @param loggingData if true data will be logged which means that all
   data sent out to the all the clients will be logged
**/
AREXPORT void ArNetServer::setLoggingDataSent(bool loggingData)
{
  if (myChildServer != NULL)
    myChildServer->setLoggingDataSent(loggingData);

  myLoggingDataSent = loggingData;
  std::list<ArSocket *>::iterator it;
  for (it = myConnectingConns.begin(); it != myConnectingConns.end(); ++it)
    (*it)->setLogWriteStrings(loggingData);
  for (it = myConns.begin(); it != myConns.end(); ++it)
    (*it)->setLogWriteStrings(loggingData);
}

/**
   @return if true data will be logged which means that all data sent
   out to the all the clients will be logged
**/
AREXPORT bool ArNetServer::getLoggingDataSent(void)
{
  return myLoggingDataSent;
}

/**
   @param loggingData if true data will be logged which means that all
   commands received from clients are logged
**/
AREXPORT void ArNetServer::setLoggingDataReceived(bool loggingData)
{
  if (myChildServer != NULL)
    myChildServer->setLoggingDataReceived(loggingData);

  myLoggingDataReceived = loggingData;
}

/**
   @return if true data will be logged which means that all commands
   received from clients are logged 
**/
AREXPORT bool ArNetServer::getLoggingDataReceived(void)
{
  return myLoggingDataReceived;
}


/**
   Set reverse line ending characters for compatibility with certain old
   clients.
   @param useWrongEndChars if true the wrong end chars will be sent ('\\n\\r' instead
   of '\\r\\n'); a nonstandard, deprecated line ending, but may be required for certain old
   clients.
**/
AREXPORT void ArNetServer::setUseWrongEndChars(bool useWrongEndChars)
{
  if (myChildServer != NULL)
    myChildServer->setUseWrongEndChars(useWrongEndChars);

  myUseWrongEndChars = useWrongEndChars;
  std::list<ArSocket *>::iterator it;
  for (it = myConnectingConns.begin(); it != myConnectingConns.end(); ++it)
    (*it)->setStringUseWrongEndChars(useWrongEndChars);
  for (it = myConns.begin(); it != myConns.end(); ++it)
    (*it)->setStringUseWrongEndChars(useWrongEndChars);
}

/**
   @return if true data will be logged which means that all data sent
   out to the all the clients will be logged
**/
AREXPORT bool ArNetServer::getUseWrongEndChars(void)
{
  return myUseWrongEndChars;
}

AREXPORT void ArNetServer::runOnce(void)
{

  //  ArSocket acceptingSocket;
  ArSocket *socket;
  char *str;
  std::list<ArSocket *> removeList;
  std::list<ArSocket *>::iterator it;
  ArArgumentBuilder *args = NULL;
  std::string command;

  if (!myOpened)
  {
    return;
  }

  // copy the strings we want to send next cycle
  myNextCycleSendsMutex.lock();

  std::list<std::string> nextCycleSends;
  std::list<std::string>::iterator ncsIt;;
  
  nextCycleSends = myNextCycleSends;
  myNextCycleSends.clear();

  myNextCycleSendsMutex.unlock();


  lock();
  // get any new sockets that want to connect
  while (myServerSocket.accept(&myAcceptingSocket) &&
	 myAcceptingSocket.getFD() >= 0)
  {
    //myAcceptingSocket.setNonBlock();
    // see if we want more sockets
    if (!myMultipleClients && (myConns.size() > 0 ||
			       myConnectingConns.size() > 0))
    {
      // we didn't want it, so politely tell it to go away
      myAcceptingSocket.writeString("Conn refused.");
      myAcceptingSocket.writeString(
	      "Only client allowed and it is already connected.");
      myAcceptingSocket.close();
      ArLog::log(ArLog::Terse, "%s not taking multiple clients and another client tried to connect from %s.", myName.c_str(), myAcceptingSocket.getIPString());
    }
    else 
    {
      // we want the client so we put it in our list of connecting
      // sockets, which means that it is waiting to give its password
      socket = new ArSocket;
      socket->setLogWriteStrings(myLoggingDataSent);
      socket->setStringUseWrongEndChars(myUseWrongEndChars);
      socket->transfer(&myAcceptingSocket);
      socket->setIPString((myName + "::" + socket->getIPString()).c_str());
      socket->setNonBlock();
      if (!myPassword.empty())
      {
	socket->writeString("Enter password:");
	myConnectingConns.push_front(socket);
	ArLog::log(ArLog::Normal, 
		   "%s: Client connecting from %s.",
		   myName.c_str(), socket->getIPString());
      }
      else
      {
	ArLog::log(ArLog::Normal, 
		   "%s: Client from %s connected (with no password required).",
		   myName.c_str(), socket->getIPString());
	myConns.push_front(socket);
	internalGreeting(socket);
      }
    }
  }

  // now we read in data from our connecting sockets and see if
  // they've given us the password
  for (it = myConnectingConns.begin(); it != myConnectingConns.end(); ++it)
  {
    socket = (*it);
    // read in what the client has to say
    if ((str = socket->readString()) != NULL)
    {
      if (str[0] == '\0')
	continue;
      // now see if the word matchs the password
      if (myPassword == str)
      {
	ArLog::log(ArLog::Normal, 
		   "%s: Client from %s gave password and connected.",
		   myName.c_str(), socket->getIPString());
	myConns.push_front(socket);
	removeList.push_front(socket);
	internalGreeting(socket);
      }
      else
      {
	socket->close();
	myDeleteList.push_front(socket);
	ArLog::log(ArLog::Terse, 
		   "%s: Client from %s gave wrong password and is being disconnected.", 
		   myName.c_str(), socket->getIPString());
      }
    }
    // if we couldn't read a string it means we lost a connection
    else
    {
      ArLog::log(ArLog::Normal, 
		 "%s: Connection to %s lost.", 
		 myName.c_str(), socket->getIPString());
      socket->close();
      myDeleteList.push_front(socket);
    }
  }
  // now we clear out the ones we want to remove from our connecting
  // clients list
  while ((it = removeList.begin()) != removeList.end())
  {
    socket = (*it);
    myConnectingConns.remove(socket);
    removeList.pop_front();
  }



  // first send it all the things to be sent the next cycle... this
  // could be done in the for loop below this one, but since we want
  // to only log once per broadcast it's done here
  for (ncsIt = nextCycleSends.begin(); 
       ncsIt != nextCycleSends.end(); 
       ncsIt++)
  {

    // now we read in data from our connected sockets 
    for (it = myConns.begin(); it != myConns.end() && myOpened; ++it)
    {
      socket = (*it);

      socket->setLogWriteStrings(false);

      if (myLoggingDataSent)
	ArLog::log(ArLog::Terse, "%s::sendToAllClientsNextCycle: Sending: %s", myName.c_str(), (*ncsIt).c_str());
      socket->writeString((*ncsIt).c_str());
      
      socket->setLogWriteStrings(myLoggingDataSent);
    }
  }

  // now we read in data from our connected sockets 
  for (it = myConns.begin(); it != myConns.end() && myOpened; ++it)
  {
    socket = (*it);

    // read in what the client has to say
    while ((str = socket->readString()) != NULL)
    {
      // if this is null then there wasn't anything said
      if (str[0] == '\0')
	break;
      // make sure we read something
      // set up the arguments and then call the function for the
      // argument
      args = new ArArgumentBuilder;
      args->addPlain(str);
      //args->log();
      parseCommandOnSocket(args, socket);
      delete args;
      args = NULL;
    }
    // if str was NULL we lost connection
    if (str == NULL)
    {
      ArLog::log(ArLog::Normal, 
		 "%s: Connection to %s lost.", myName.c_str(), 
		 socket->getIPString());
      socket->close();
      myDeleteList.push_front(socket);
    }
  }

  // now we delete the ones we want to delete (we could do this above
  // but then it wouldn't be symetrical with above)
  while ((it = myDeleteList.begin()) != myDeleteList.end())
  {
    socket = (*it);
    myConnectingConns.remove(socket);
    myConns.remove(socket);
    // remove this instead of the old pop since there could be two of
    // the same in the list if it lost connection exactly when it
    // parsed the quit
    myDeleteList.remove(socket);
    socket->close();
    delete socket;
  }

  if (myWantToClose)
  {
    close();
  }
  unlock();
}

AREXPORT void ArNetServer::close(void)
{
  std::list<ArSocket *>::iterator it;
  ArSocket *socket;
  
  if (!myOpened)
    return;
  myWantToClose = false;
  ArLog::log(ArLog::Normal, "%s shutting down server.", myName.c_str());
  sendToAllClients("Shutting down server");
  for (it = myConnectingConns.begin(); it != myConnectingConns.end(); ++it)
  {
    (*it)->writeString("Shutting down server");
  }
  myOpened = false;

  while ((it = myConnectingConns.begin())!= myConnectingConns.end())
  {
    socket = (*it);
    myConnectingConns.pop_front();
    socket->close();
    delete socket;
  }
  while ((it = myConns.begin()) != myConns.end())
  {
    socket = (*it);
    myConns.pop_front();
    socket->close();
    delete socket;
  }
  myServerSocket.close();
}

AREXPORT void ArNetServer::internalGreeting(ArSocket *socket)
{
  if (mySquelchNormal)
    return;
  socket->writeString("Welcome to the server.");
  socket->writeString("You can type 'help' at any time for the following help list.");
  internalHelp(socket);
}

AREXPORT void ArNetServer::internalHelp(ArSocket *socket)
{
 std::map<std::string, std::string, ArStrCaseCmpOp>::iterator it;
  
 socket->writeString("Commands:");
 for (it = myHelpMap.begin(); it != myHelpMap.end(); ++it)
   socket->writeString("%15s%10s%s", it->first.c_str(), "", 
		       it->second.c_str());
 socket->writeString("End of commands");
}

AREXPORT void ArNetServer::internalHelp(char **argv, int argc, 
					ArSocket *socket)
{
  internalHelp(socket);
}


AREXPORT void ArNetServer::internalEcho(char **argv, int argc, 
					    ArSocket *socket)
{
  // if they just typed it we tell them if its on or off
  if (argc == 1)
  {
    if (socket->getEcho())
      socket->writeString("Echo is on.");
    else
      socket->writeString("Echo is off.");
  }
  // if the have two words see if they have the right args
  else if (argc == 2 && strcasecmp(argv[1], "on") == 0)
  {
    socket->writeString("Echo turned on.");
    socket->setEcho(true);
  }
  else if (argc == 2 && strcasecmp(argv[1], "off") == 0)
  {
    socket->writeString("Echo turned off.");
    socket->setEcho(false);
  }
  else
  {
    socket->writeString("usage: echo <on/off>");
  }
}

AREXPORT void ArNetServer::internalQuit(char **argv, int argc, 
					 ArSocket *socket)
{
  socket->writeString("Closing connection");

  myDeleteList.push_front(socket);
  ArLog::log(ArLog::Normal, "%s: Client from %s quit.", 
	     myName.c_str(), socket->getIPString());
}

AREXPORT void ArNetServer::internalShutdownServer(char **argv, int argc, 
						  ArSocket *socket)
{
  sendToAllClients("Shutting down server");
  myWantToClose = true;
  if (myRobot != NULL)
    myRobot->stopRunning();
  
}

AREXPORT void ArNetServer::parseCommandOnSocket(ArArgumentBuilder *args, 
						ArSocket *socket, bool allowLog)
{

  std::map<std::string, ArFunctor3<char **, int, ArSocket *> *, ArStrCaseCmpOp>::iterator fIt;
  char **argv;
  int argc;

  if (myLoggingDataReceived && !mySquelchNormal && allowLog)
    ArLog::log(ArLog::Normal, "%s: Command received from %s: %s",
	       myName.c_str(), socket->getIPString(), args->getFullString());
  else if (myLoggingDataReceived && mySquelchNormal && allowLog)
    ArLog::log(ArLog::Normal, "%s: %s",
	       socket->getIPString(), args->getFullString());
  argv = args->getArgv();
  argc = args->getArgc();
  // if we have some command see if it has a functor
  if (argc >= 1 && 
      (fIt = myFunctorMap.find(argv[0])) != myFunctorMap.end())
  {
    fIt->second->invoke(argv, argc, socket);
  }
  // it didn't have a functor so we don't know it as a command
  else if (argc >= 1)
  {
    if (!mySquelchNormal)
      socket->writeString("Unknown command %s", argv[0]);
  }
}

AREXPORT void ArNetServer::internalAddSocketToList(ArSocket *socket)
{
  if (socket != NULL)
  {
    socket->setNonBlock();
    socket->setStringUseWrongEndChars(myUseWrongEndChars);
  }
  myConns.push_front(socket);
}


AREXPORT void ArNetServer::internalAddSocketToDeleteList(ArSocket *socket)
{
  myDeleteList.push_front(socket);
}

AREXPORT void ArNetServer::squelchNormal(void)
{
  mySquelchNormal = true;
  remCommand("help");
  remCommand("echo");
  remCommand("quit");
  remCommand("shutdownServer");

}

AREXPORT void ArNetServer::sendToClientPlain(
	ArSocket *socket, const char *ipString, const char *str)
{
  std::list<ArSocket *>::iterator it;

  for (it = myConns.begin(); it != myConns.end(); ++it)
  {
    if ((*it) == socket && strcmp((*it)->getIPString(), ipString) == 0)
    {
      if (myLoggingDataSent)
	ArLog::log(ArLog::Terse, 
		   "%s::sendToClient: Sending '%s' to %s", myName.c_str(), str,
		   ipString);
      (*it)->setLogWriteStrings(false);
      (*it)->writeString(str);
      (*it)->setLogWriteStrings(myLoggingDataSent);
    }
  }
}

/**
   This sends the given string to all the clients, this string cannot
   be more than 2048 number of bytes
**/
AREXPORT void ArNetServer::sendToClient(ArSocket *socket, const char *ipString,
			   const char *str, ...)
{
  char buf[2049];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);

  sendToClientPlain(socket, ipString, buf);
  
  va_end(ptr);
}

