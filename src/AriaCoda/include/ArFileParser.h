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
#ifndef ARFILEPARSER_H
#define ARFILEPARSER_H

#include "ariaTypedefs.h"
#include "ArArgumentParser.h"
#include "ArFunctor.h"
#include "ariaUtil.h"

/// Class for parsing files more easily
/**
   This class helps parse text files based on keywords followed by various
   values.
   To use it, add functors of different types
   of arguments with addHandler(), then call parseFile() to parse the file
   and invoke the various functors as items are read from the file.
   parseFile() returns true if there were no errors parsing and false if
   there were errors.

   One side feature is that you can have ONE handler for the keyword
   NULL, and if there is a line read that isn't entirely comments or
   whitespace that handler will be given the line.  There isn't an
   explicit set for them since then there'd be another set of 5 adds.

   There should be some whitespace after keywords in the file, and any
   semicolon (;) or hash mark (#) will act as a comment with the rest of the line
   ignored. (Alternative comment delimeters may be set using
   setCommentDelimeters()).   If no handler exists for the first word the line is
   passed to the handler above for NULL.  You can't have any lines
   longer than 10000 characters or keywords longer than 512 characters
   (though I don't know why you'd have either).  If you have more than
   2048 words on a line you'll have problems as well.

  @ingroup OptionalClasses

 * @note ArFileParser does not escape any special characters when writing or
 * loading to/from a file. Therefore in general keywords, values,
 * and comments must not contain characters which have special meaning
 * in a config file, such as '#', ';', tab or newline. 
 **/
class ArFileParser
{
public:

  /// Constructor
  AREXPORT ArFileParser(const char *baseDirectory = "./",
                        bool isPreCompressQuotes = false);



  /// Destructor
  AREXPORT ~ArFileParser(void);

  /// Adds a functor to handle a keyword that wants an easily parsable string
  AREXPORT bool addHandler(const char *keyword, 
			   ArRetFunctor1<bool, ArArgumentBuilder *> *functor);
  /// Adds a functor to handle a keyword that wants an easily parsable string and returns error messages
  AREXPORT bool addHandlerWithError(const char *keyword, 
			   ArRetFunctor3<bool, ArArgumentBuilder *, 
				    char *, size_t> *functor);
  /// Removes a handler for a keyword
  AREXPORT bool remHandler(const char *keyword, bool logIfCannotFind = true);
  /// Removes any handlers with this functor
  AREXPORT bool remHandler(ArRetFunctor1<bool, ArArgumentBuilder *> *functor);
  /// Removes any handlers with this functor
  AREXPORT bool remHandler(
	  ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor);
  /* this shouldn't be needed and would be inelegant with the new scheme, 
     if someone needs it let us know and I'll update it somehow
  /// Gets handler data for some keyword
  AREXPORT ArRetFunctor1<bool, ArArgumentBuilder *> *getHandler(const char *keyword);
  */
  
	AREXPORT void setPreParseFunctor(ArFunctor1<const char *> *functor);


  /// Opens, parses, and then closes the specified file.
  AREXPORT bool parseFile(const char *fileName, bool continueOnErrors = true,
			  bool noFileNotFoundMessage = false,
			  char *errorBuffer = NULL, size_t errorBufferLen = 0);

  /// Parses an open file; the file is not closed by this method.
  /**
   * @param file the open FILE* to be parsed; must not be NULL
   * @param buffer a non-NULL char array in which to read the file
   * @param bufferLength the number of chars in the buffer; must be greater than 0
   * @param continueOnErrors a bool set to true if parsing should continue
   * even after an error is detected
  **/
  AREXPORT bool parseFile(FILE *file, char *buffer, int bufferLength, 
			                    bool continueOnErrors = true, 
			                    char *errorBuffer = NULL, size_t errorBufferLen = 0);


  /// If parseFile is currently in progress, then terminates it as soon as possible.
  AREXPORT void cancelParsing();


  /// Gets the base directory
  AREXPORT const char *getBaseDirectory(void) const;
  /// Sets the base directory
  AREXPORT void setBaseDirectory(const char *baseDirectory);

  /// Sets the strings used to mark comments in the file to be parsed.
  AREXPORT void setCommentDelimiters(const std::list<std::string> &delimiters);

  /// Clears the strings used to mark comments in the file to be parsed.
  AREXPORT void clearCommentDelimiters();


  /// Function to parse a single line 
  AREXPORT bool parseLine(char *line, char *errorBuffer = NULL, 
			  size_t errorBufferLen = 0);
  /// Function to reset counters
  AREXPORT void resetCounters(void);
  /// Sets the maximum number of arguments in a line we can expect
  AREXPORT void setMaxNumArguments(size_t maxNumArguments = 512)
    { myMaxNumArguments = maxNumArguments; }
  /// Turn on this flag to reduce the number of verbose log messages.
  AREXPORT void setQuiet(bool isQuiet);

protected:

  /// Returns true if cancelParsing() has been called during parseFile()
  bool isInterrupted();

  class HandlerCBType
  {
    public:
    HandlerCBType(
	    ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor)
    {
      myCallbackWithError = functor;
      myCallback = NULL;
    }
    HandlerCBType(ArRetFunctor1<bool, ArArgumentBuilder *> *functor)
    {
      myCallbackWithError = NULL;
      myCallback = functor;
    }
    ~HandlerCBType() {}
    bool call(ArArgumentBuilder *arg, char *errorBuffer, 
	      size_t errorBufferLen) 
    { 
      if (myCallbackWithError != NULL) 
	return myCallbackWithError->invokeR(arg, errorBuffer, errorBufferLen);
      else if (myCallback != NULL) 
	return myCallback->invokeR(arg); 
      // if we get here there's a problem
      ArLog::log(ArLog::Terse, "ArFileParser: Horrible problem with process callbacks");
      return false;
    }
    bool haveFunctor(
	    ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor)
    { 
      if (myCallbackWithError == functor) 
	return true; 
      else 
	return false; 
    }
    bool haveFunctor(ArRetFunctor1<bool, ArArgumentBuilder *> *functor)
    { 
      if (myCallback == functor) 
	return true; 
      else 
	return false; 
    }
    const char *getName(void) 
    { 
      if (myCallbackWithError != NULL)
	return myCallbackWithError->getName();
      else if (myCallback != NULL)
	return myCallback->getName();
      // if we get here there's a problem
      ArLog::log(ArLog::Terse, "ArFileParser: Horrible problem with process callback names");
      return NULL;
    }
    protected:
    ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *myCallbackWithError;
    ArRetFunctor1<bool, ArArgumentBuilder *> *myCallback;
  };
  size_t myMaxNumArguments;
  int myLineNumber;
  std::string myBaseDir;
  std::list<std::string> myCommentDelimiterList;

  ArFunctor1<const char *> *myPreParseFunctor;

  std::map<std::string, HandlerCBType *, ArStrCaseCmpOp> myMap;
  // handles that NULL case
  HandlerCBType *myRemainderHandler;
  bool myIsQuiet;
  bool myIsPreCompressQuotes;
  bool myIsInterrupted;
  ArMutex myInterruptMutex;
};

#endif // ARFILEPARSER_H


