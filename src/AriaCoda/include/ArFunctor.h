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
#ifndef ARFUNCTOR_H
#define ARFUNCTOR_H

#include "ariaTypedefs.h"
#include "ariaOSDef.h"
#include <stdarg.h>
#include <stdio.h>

/// An object which allows storing a generalized reference to a method with an object instance to call later (used for callback functions)
/**
  Functors are meant to encapsulate the idea of a pointer to a function
  which is a member of a class. To use a pointer to a member function,
  you must have a C style function pointer, 'void(Class::*)()', and a
  pointer to an instance of the class in which the function is a member
  of. This is because all non-static member functions must have a 'this'
  pointer. If they dont and if the member function uses any member data
  or even other member functions it will not work right and most likely
  crash. This is because the 'this' pointer is not the correct value
  and is most likely a random uninitialized value. The virtue of static
  member functions is that they do not require a 'this' pointer to be run.
  But the compiler will never let you access any member data or functions
  from within a static member function.

  Because of the design of C++ never allowed for encapsulating these two
  pointers together into one language supported construct, this has to be
  done by hand. For conviences sake, there are functors (ArGlobalFunctor,
  ArGlobalRetFunctor) which take a pure C style function pointer
  (a non-member function). This is in case you want to use a functor that
  refers to a global C style function.

  @note When supplying the function pointer to the callback constructor,
  make sure to pass the function pointer using "&" (pointer reference
  syntax).  When supplying a method, make sure to qualify the method
  name with the class name as well.  For example:
  @code 
    ArFunctorC<int> example(&obj, &ExampleClass::exampleMethod);
  @endcode
  otherwise you will recieve compile errors such as "invalid use of
  member function", or that the function was not declared.

  Aria makes use of functors by using them as callback functions. Since
  Aria is programmed using the object oriented programming paradigm, all
  the callback functions need to be tied to an object and a particular
  instance. Thus the need for functors. Most of the use of callbacks simply
  take an ArFunctor, which is the base class for all the functors. This
  class only has the ability to invoke a functor. All the derivitave
  functors have the ability to invoke the correct function on the correct
  object.

  Because functions have different signatures because they take different
  types of parameters and have different number of parameters, templates
  were used to create the functors. These are the base classes for the
  functors. These classes encapsulate everything except for the class
  type that the member function is a member of. This allows someone to
  accept a functor of type ArFunctor1<int> which has one parameter of type
  'int'. But they never have to know that the function is a member function
  of class 'SomeUnknownType'. These classes are:

  ArFunctor, ArFunctor1, ArFunctor2, ArFunctor3
  ArRetFunctor, ArRetFunctor1, ArRetFunctor2, ArRetFunctor3

  These 8 functors are the only thing a piece of code that wants a functor
  will ever need. But these classes are abstract classes and can not be
  instantiated. On the other side, the piece of code that wants to be
  called back will need the functor classes that know about the class type.
  These functors are:

  ArFunctorC, ArFunctor1C, ArFunctor2C, ArFunctor3C
  ArRetFunctorC, ArRetFunctor1C, ArRetFunctor2C, ArRetFunctor3C

  These functors are meant to be instantiated and passed of to a piece of
  code that wants to use them. That piece of code should only know the
  functor as one of the functor classes without the 'C' in it.

  Note that you can create these FunctorC instances with default
  arguments that are then used when the invoke is called without those
  arguments...  These are quite useful since if you have a class that
  expects an ArFunctor you can make an ArFunctor1C with default
  arguments and pass it as an ArFunctor... and it will get called with
  that default argument, this is useful for having multiple functors
  use the same function with different arguments and results (just
  takes one functor each). 

  Functors now have a getName() method, this is useful as an aid to debugging,
  allowing you to display the name of some functor being used.

  @javanote You can subclass ArFunctor and override invoke().

  @see @ref functorExample.cpp

  @ingroup ImportantClasses
**/
class ArFunctor
{
public:

  /// Destructor
  virtual ~ArFunctor() {}

  /// Invokes the functor
  virtual void invoke(void) = 0;

  /// Gets the name of the functor
  virtual const char *getName(void) { return myName.c_str();  }

  /// Sets the name of the functor
  virtual void setName(const char *name) { myName = name; }

#ifndef SWIG
  /// Sets the name of the functor with formatting
  /** @swigomit use setName() */
  virtual void setNameVar(const char *name, ...) 
    { 
      char arg[2048];
      va_list ptr;
      va_start(ptr, name);
      vsnprintf(arg, sizeof(arg), name, ptr);
      arg[sizeof(arg) - 1] = '\0';
      va_end(ptr);
      return setName(arg);
    }
#endif

protected:
  std::string myName;
};

/// Base class for functors with 1 parameter
/**
   This is the base class for functors with 1 parameter. Code that has a
   reference to a functor that takes 1 parameter should use this class
   name. This allows the code to know how to invoke the functor without
   knowing which class the member function is in.

   For an overall description of functors, see ArFunctor.
*/
template<class P1>
class ArFunctor1 : public ArFunctor
{
public:

  /// Destructor
  virtual ~ArFunctor1() {}

  /// Invokes the functor
  virtual void invoke(void) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) = 0;
};

/// Base class for functors with 2 parameters
/**
   This is the base class for functors with 2 parameters. Code that has a
   reference to a functor that takes 2 parameters should use this class
   name. This allows the code to know how to invoke the functor without
   knowing which class the member function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2>
class ArFunctor2 : public ArFunctor1<P1>
{
public:

  /// Destructor
  virtual ~ArFunctor2() {}

  /// Invokes the functor
  virtual void invoke(void) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) = 0;
};

/// Base class for functors with 3 parameters
/**
   This is the base class for functors with 3 parameters. Code that has a
   reference to a functor that takes 3 parameters should use this class
   name. This allows the code to know how to invoke the functor without
   knowing which class the member function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2, class P3>
class ArFunctor3 : public ArFunctor2<P1, P2>
{
public:

  /// Destructor
  virtual ~ArFunctor3() {}

  /// Invokes the functor
  virtual void invoke(void) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) = 0;
};



/// Base class for functors with 4 parameters
/**
   This is the base class for functors with 4 parameters. Code that has a
   reference to a functor that takes 4 parameters should use this class
   name. This allows the code to know how to invoke the functor without
   knowing which class the member function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2, class P3, class P4>
class ArFunctor4 : public ArFunctor3<P1, P2, P3>
{
public:

  /// Destructor
  virtual ~ArFunctor4() {}

  /// Invokes the functor
  virtual void invoke(void) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
    virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) = 0;

};


/// Base class for functors with 4 parameters
/**
   This is the base class for functors with 4 parameters. Code that has a
   reference to a functor that takes 4 parameters should use this class
   name. This allows the code to know how to invoke the functor without
   knowing which class the member function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2, class P3, class P4, class P5>
class ArFunctor5 : public ArFunctor4<P1, P2, P3, P4>
{
public:

  /// Destructor
  virtual ~ArFunctor5() {}

  /// Invokes the functor
  virtual void invoke(void) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) = 0;

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) = 0;

};




/// Base class for functors with a return value
/**
   This is the base class for functors with a return value. Code that has a
   reference to a functor that returns a value should use this class
   name. This allows the code to know how to invoke the functor without
   knowing which class the member function is in.
   
   For an overall description of functors, see ArFunctor.

   @javanote To create the equivalent of ArRetFunctor<bool>, you can 
      subclass <code>ArRetFunctor_Bool</code> and override <code>invoke(bool)</code>
     
*/
template<class Ret>
class ArRetFunctor : public ArFunctor
{
public:

  /// Destructor
  virtual ~ArRetFunctor() {}

  /// Invokes the functor
  virtual void invoke(void) {invokeR();}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) = 0;
};

/// Base class for functors with a return value with 1 parameter
/**
   This is the base class for functors with a return value and take 1
   parameter. Code that has a reference to a functor that returns a value
   and takes 1 parameter should use this class name. This allows the code
   to know how to invoke the functor without knowing which class the member
   function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1>
class ArRetFunctor1 : public ArRetFunctor<Ret>
{
public:

  /// Destructor
  virtual ~ArRetFunctor1() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) = 0;
};

/// Base class for functors with a return value with 2 parameters
/**
   This is the base class for functors with a return value and take 2
   parameters. Code that has a reference to a functor that returns a value
   and takes 2 parameters should use this class name. This allows the code
   to know how to invoke the functor without knowing which class the member
   function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2>
class ArRetFunctor2 : public ArRetFunctor1<Ret, P1>
{
public:

  /// Destructor
  virtual ~ArRetFunctor2() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) = 0;
};

/// Base class for functors with a return value with 3 parameters
/**
   This is the base class for functors with a return value and take 3
   parameters. Code that has a reference to a functor that returns a value
   and takes 3 parameters should use this class name. This allows the code
   to know how to invoke the functor without knowing which class the member
   function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2, class P3>
class ArRetFunctor3 : public ArRetFunctor2<Ret, P1, P2>
{
public:

  /// Destructor
  virtual ~ArRetFunctor3() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) = 0;
};


/// Base class for functors with a return value with 4 parameters
/**
   This is the base class for functors with a return value and take 4
   parameters. Code that has a reference to a functor that returns a value
   and takes 4 parameters should use this class name. This allows the code
   to know how to invoke the functor without knowing which class the member
   function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2, class P3, class P4>
class ArRetFunctor4 : public ArRetFunctor3<Ret, P1, P2, P3>
{
public:

  /// Destructor
  virtual ~ArRetFunctor4() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) = 0;
};

/// Base class for functors with a return value with 5 parameters
/**
   This is the base class for functors with a return value and take 5
   parameters. Code that has a reference to a functor that returns a value
   and takes 5 parameters should use this class name. This allows the code
   to know how to invoke the functor without knowing which class the member
   function is in.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2, class P3, class P4, class P5>
class ArRetFunctor5 : public ArRetFunctor4<Ret, P1, P2, P3, P4>
{
public:

  /// Destructor
  virtual ~ArRetFunctor5() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) = 0;

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) = 0;
};





//////
//////
//////
//////
//////
//////
////// ArFunctors for global functions. C style function pointers.
//////
//////
//////
//////
//////
//////


#ifndef SWIG
/// Functor for a global function with no parameters
/**
   This is a class for global functions. This ties a C style function
   pointer into the functor class hierarchy as a convience. Code that
   has a reference to this class and treat it as an ArFunctor can use
   it like any other functor.
   
   For an overall description of functors, see ArFunctor.

   @swigomit
*/
class ArGlobalFunctor : public ArFunctor
{
public:

  /// Constructor
  ArGlobalFunctor() {}
  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalFunctor(void (*func)(void)) : myFunc(func) {}
  /// Destructor
  virtual ~ArGlobalFunctor() {}

  /// Invokes the functor
  virtual void invoke(void) {(*myFunc)();}
protected:

  void (*myFunc)(void);
};

/// Functor for a global function with 1 parameter
/**
   This is a class for global functions which take 1 parameter. This ties
   a C style function pointer into the functor class hierarchy as a
   convience. Code that has a reference to this class and treat it as
   an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1>
class ArGlobalFunctor1 : public ArFunctor1<P1>
{
public:

  /// Constructor
  ArGlobalFunctor1() {}
  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalFunctor1(void (*func)(P1)) :
    myFunc(func), myP1() {}
  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalFunctor1(void (*func)(P1), P1 p1) :
    myFunc(func), myP1(p1) {}

  /// Destructor
  virtual ~ArGlobalFunctor1() {}

  /// Invokes the functor
  virtual void invoke(void) {(*myFunc)(myP1);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(*myFunc)(p1);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

protected:

  void (*myFunc)(P1);
  P1 myP1;
};


/// Functor for a global function with 1 parameter which may be a const type
template<class P1>
class ArGlobalFunctor1Const : public ArFunctor1<P1>
{
public:
  ArGlobalFunctor1Const() {}
  ArGlobalFunctor1Const(void (*func)(P1)) :
    myFunc(func) {}
  virtual ~ArGlobalFunctor1Const() {}
  virtual void invoke(P1 p1) {(*myFunc)(p1);}
protected:
  void (*myFunc)(P1);
};


/// Functor for a global function with 2 parameters
/**
   This is a class for global functions which take 2 parameters. This ties
   a C style function pointer into the functor class hierarchy as a
   convience. Code that has a reference to this class and treat it as
   an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2>
class ArGlobalFunctor2 : public ArFunctor2<P1, P2>
{
public:

  /// Constructor
  ArGlobalFunctor2() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalFunctor2(void (*func)(P1, P2)) :
    myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalFunctor2(void (*func)(P1, P2), P1 p1) :
    myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalFunctor2(void (*func)(P1, P2), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2) {}

  /// Destructor
  virtual ~ArGlobalFunctor2() {}

  /// Invokes the functor
  virtual void invoke(void) {(*myFunc)(myP1, myP2);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(*myFunc)(p1, myP2);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(*myFunc)(p1, p2);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

protected:

  void (*myFunc)(P1, P2);
  P1 myP1;
  P2 myP2;
};

/// Functor for a global function with 3 parameters
/**
   This is a class for global functions which take 3 parameters. This ties
   a C style function pointer into the functor class hierarchy as a
   convience. Code that has a reference to this class and treat it as
   an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2, class P3>
class ArGlobalFunctor3 : public ArFunctor3<P1, P2, P3>
{
public:

  /// Constructor
  ArGlobalFunctor3() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalFunctor3(void (*func)(P1, P2, P3)) :
    myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalFunctor3(void (*func)(P1, P2, P3), P1 p1) :
    myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalFunctor3(void (*func)(P1, P2, P3), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArGlobalFunctor3(void (*func)(P1, P2, P3), P1 p1, P2 p2, P3 p3) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Destructor
  virtual ~ArGlobalFunctor3() {}

  /// Invokes the functor
  virtual void invoke(void) {(*myFunc)(myP1, myP2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(*myFunc)(p1, myP2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(*myFunc)(p1, p2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(*myFunc)(p1, p2, p3);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

protected:

  void (*myFunc)(P1, P2, P3);
  P1 myP1;
  P2 myP2;
  P3 myP3;
};



/// Functor for a global function with 4 parameters
/**
   This is a class for global functions which take 4 parameters. This ties
   a C style function pointer into the functor class hierarchy as a
   convience. Code that has a reference to this class and treat it as
   an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2, class P3, class P4>
class ArGlobalFunctor4 : public ArFunctor4<P1, P2, P3, P4>
{
public:

  /// Constructor
  ArGlobalFunctor4() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalFunctor4(void (*func)(P1, P2, P3, P4)) :
    myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalFunctor4(void (*func)(P1, P2, P3, P4), P1 p1) :
    myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalFunctor4(void (*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArGlobalFunctor4(void (*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

   /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
 ArGlobalFunctor4(void (*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

  /// Destructor
  virtual ~ArGlobalFunctor4() {}

  /// Invokes the functor
  virtual void invoke(void) {(*myFunc)(myP1, myP2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(*myFunc)(p1, myP2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(*myFunc)(p1, p2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(*myFunc)(p1, p2, p3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) {(*myFunc)(p1, p2, p3, p4);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

protected:

  void (*myFunc)(P1, P2, P3, P4);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
};


/// Functor for a global function with 5 parameters
/**
   This is a class for global functions which take 5 parameters. This ties
   a C style function pointer into the functor class hierarchy as a
   convience. Code that has a reference to this class and treat it as
   an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class P1, class P2, class P3, class P4, class P5>
class ArGlobalFunctor5 : public ArFunctor5<P1, P2, P3, P4, P5>
{
public:

  /// Constructor
  ArGlobalFunctor5() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalFunctor5(void (*func)(P1, P2, P3, P4, P5)) :
    myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalFunctor5(void (*func)(P1, P2, P3, P4, P5), P1 p1) :
    myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalFunctor5(void (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArGlobalFunctor5(void (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

   /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArGlobalFunctor5(void (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}

   /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
 */
  ArGlobalFunctor5(void (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

  /// Destructor
  virtual ~ArGlobalFunctor5() {}

  /// Invokes the functor
  virtual void invoke(void) {(*myFunc)(myP1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(*myFunc)(p1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(*myFunc)(p1, p2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(*myFunc)(p1, p2, p3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) {(*myFunc)(p1, p2, p3, p4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) {(*myFunc)(p1, p2, p3, p4, p5);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

  /// Set the default fifth parameter
  /**
     @param p5 default fifth parameter
  */
  virtual void setP5(P5 p5) {myP5=p5;}

protected:

  void (*myFunc)(P1, P2, P3, P4, P5);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
  P5 myP5;
};

#endif // Omitting ArGlobalFunctor from Swig




//////
//////
//////
//////
//////
//////
////// ArFunctors for global functions, C style function pointers with return
////// return values.
//////
//////
//////
//////
//////
//////

#ifndef SWIG

/// Functor for a global function with return value
/**
   This is a class for global functions which return a value. This ties
   a C style function pointer into the functor class hierarchy as a
   convience. Code that has a reference to this class and treat it as
   an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret>
class ArGlobalRetFunctor : public ArRetFunctor<Ret>
{
public:

  /// Constructor
  ArGlobalRetFunctor() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalRetFunctor(Ret (*func)(void)) : myFunc(func) {}

  /// Destructor
  virtual ~ArGlobalRetFunctor() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (*myFunc)();}

protected:

  Ret (*myFunc)(void);
};


/// Functor for a global function with 1 parameter and return value
/**
   This is a class for global functions which take 1 parameter and return
   a value. This ties a C style function pointer into the functor class
   hierarchy as a convience. Code that has a reference to this class
   and treat it as an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1>
class ArGlobalRetFunctor1 : public ArRetFunctor1<Ret, P1>
{
public:

  /// Constructor
  ArGlobalRetFunctor1() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalRetFunctor1(Ret (*func)(P1)) :
    myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalRetFunctor1(Ret (*func)(P1), P1 p1) :
    myFunc(func), myP1(p1) {}

  /// Destructor
  virtual ~ArGlobalRetFunctor1() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (*myFunc)(myP1);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (*myFunc)(p1);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

protected:

  Ret (*myFunc)(P1);
  P1 myP1;
};

/// Functor for a global function with 2 parameters and return value
/**
   This is a class for global functions which take 2 parameters and return
   a value. This ties a C style function pointer into the functor class
   hierarchy as a convience. Code that has a reference to this class
   and treat it as an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2>
class ArGlobalRetFunctor2 : public ArRetFunctor2<Ret, P1, P2>
{
public:

  /// Constructor
  ArGlobalRetFunctor2() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalRetFunctor2(Ret (*func)(P1, P2)) :
    myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalRetFunctor2(Ret (*func)(P1, P2), P1 p1) :
    myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalRetFunctor2(Ret (*func)(P1, P2), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2) {}

  /// Destructor
  virtual ~ArGlobalRetFunctor2() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (*myFunc)(myP1, myP2);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (*myFunc)(p1, myP2);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (*myFunc)(p1, p2);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

protected:

  Ret (*myFunc)(P1, P2);
  P1 myP1;
  P2 myP2;
};

/// Functor for a global function with 2 parameters and return value
/**
   This is a class for global functions which take 2 parameters and return
   a value. This ties a C style function pointer into the functor class
   hierarchy as a convience. Code that has a reference to this class
   and treat it as an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2, class P3>
class ArGlobalRetFunctor3 : public ArRetFunctor3<Ret, P1, P2, P3>
{
public:

  /// Constructor
  ArGlobalRetFunctor3() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalRetFunctor3(Ret (*func)(P1, P2, P3)) :
    myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalRetFunctor3(Ret (*func)(P1, P2, P3), P1 p1) :
    myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalRetFunctor3(Ret (*func)(P1, P2, P3), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArGlobalRetFunctor3(Ret (*func)(P1, P2, P3), P1 p1, P2 p2, P3 p3) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Destructor
  virtual ~ArGlobalRetFunctor3() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (*myFunc)(myP1, myP2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (*myFunc)(p1, myP2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (*myFunc)(p1, p2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) {return (*myFunc)(p1, p2, p3);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}
  
  

protected:

  Ret (*myFunc)(P1, P2, P3);
  P1 myP1;
  P2 myP2;
  P3 myP3;
};



/// Functor for a global function with 4 parameters and return value
/**
   This is a class for global functions which take 4 parameters and return
   a value. This ties a C style function pointer into the functor class
   hierarchy as a convience. Code that has a reference to this class
   and treat it as an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2, class P3, class P4>
class ArGlobalRetFunctor4 : public ArRetFunctor4<Ret, P1, P2, P3, P4>
{
public:

  /// Constructor
  ArGlobalRetFunctor4() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalRetFunctor4(Ret (*func)(P1, P2, P3, P4)) :
    myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalRetFunctor4(Ret (*func)(P1, P2, P3, P4), P1 p1) :
    myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalRetFunctor4(Ret (*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
 */
  ArGlobalRetFunctor4(Ret (*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

	/// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArGlobalRetFunctor4(Ret (*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

  /// Destructor
  virtual ~ArGlobalRetFunctor4() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (*myFunc)(myP1, myP2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (*myFunc)(p1, myP2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (*myFunc)(p1, p2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) {return (*myFunc)(p1, p2, p3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) {return (*myFunc)(p1, p2, p3, p4);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}
  
  
  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

protected:

  Ret (*myFunc)(P1, P2, P3, P4);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
};


/// Functor for a global function with 4 parameters and return value
/**
   This is a class for global functions which take 4 parameters and return
   a value. This ties a C style function pointer into the functor class
   hierarchy as a convience. Code that has a reference to this class
   and treat it as an ArFunctor can use it like any other functor.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class P1, class P2, class P3, class P4, class P5>
class ArGlobalRetFunctor5 : public ArRetFunctor5<Ret, P1, P2, P3, P4, P5>
{
public:

  /// Constructor
  ArGlobalRetFunctor5() {}

  /// Constructor - supply function pointer
  /**
     @param func global function pointer
  */
  ArGlobalRetFunctor5(Ret (*func)(P1, P2, P3, P4, P5)) :
    myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
  */
  ArGlobalRetFunctor5(Ret (*func)(P1, P2, P3, P4, P5), P1 p1) :
    myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArGlobalRetFunctor5(Ret (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
 */
  ArGlobalRetFunctor5(Ret (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

	/// Constructor - supply function pointer, default parameters
  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArGlobalRetFunctor5(Ret (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}

  /**
     @param func global function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
 */
  ArGlobalRetFunctor5(Ret (*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

  /// Destructor
  virtual ~ArGlobalRetFunctor5() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (*myFunc)(myP1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (*myFunc)(p1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (*myFunc)(p1, p2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) {return (*myFunc)(p1, p2, p3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) {return (*myFunc)(p1, p2, p3, p4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) {return (*myFunc)(p1, p2, p3, p4, p5);}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}
  
  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

  /// Set the default fifth parameter
  /**
     @param p5 default fifth parameter
  */
  virtual void setP5(P5 p5) {myP5=p5;}

protected:

  Ret (*myFunc)(P1, P2, P3, P4, P5);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
  P5 myP5;
};

#endif // omitting ArGlobalRetFunctor from SWIG




//////
//////
//////
//////
//////
//////
////// ArFunctors for member functions
//////
//////
//////
//////
//////
//////


/// Functor for a member function
/**
   This is a class for member functions. This class contains the knowledge
   on how to call a member function on a particular instance of a class.
   This class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T>
class ArFunctorC : public ArFunctor
{
public:

  /// Constructor
  ArFunctorC() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctorC(T &obj, void (T::*func)(void)) : myObj(&obj), myFunc(func) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctorC(T *obj, void (T::*func)(void)) : myObj(obj), myFunc(func) {}

  /// Destructor
  virtual ~ArFunctorC() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)();}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

protected:

  T *myObj;
  void (T::*myFunc)(void);
};


/// Functor for a member function with 1 parameter
/**
   This is a class for member functions which take 1 parameter. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T, class P1>
class ArFunctor1C : public ArFunctor1<P1>
{
public:

  /// Constructor
  ArFunctor1C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor1C(T &obj, void (T::*func)(P1)) :
    myObj(&obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor1C(T &obj, void (T::*func)(P1), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor1C(T *obj, void (T::*func)(P1)) :
    myObj(obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor1C(T *obj, void (T::*func)(P1), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1) {}

  /// Destructor
  virtual ~ArFunctor1C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

protected:

  T *myObj;
  void (T::*myFunc)(P1);
  P1 myP1;
};


/// Functor for a member function with 1 parameter, which may be const
/**
   This is a class for member functions which take 1 parameter, which may be a
   const type. This class is like ArFunctor1C, except a default value for P1 
   can't be set. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T, class P1>
class ArFunctor1CConst : public ArFunctor1<P1>
{
public:
  ArFunctor1CConst() {}
  ArFunctor1CConst(T &obj, void (T::*func)(P1)) :
    myObj(&obj), myFunc(func) {}
  ArFunctor1CConst(T *obj, void (T::*func)(P1)) :
    myObj(obj), myFunc(func) {}
  virtual ~ArFunctor1CConst() {}
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1);}
  virtual void setThis(T *obj) {myObj=obj;}
  virtual void setThis(T &obj) {myObj=&obj;}
private:
  virtual void invoke() {} // can't do anything without an argument
protected:
  T *myObj;
  void (T::*myFunc)(P1);
};


/// Functor for a member function with 2 parameters
/**
   This is a class for member functions which take 2 parameters. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T, class P1, class P2>
class ArFunctor2C : public ArFunctor2<P1, P2>
{
public:

  /// Constructor
  ArFunctor2C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor2C(T &obj, void (T::*func)(P1, P2)) :
    myObj(&obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor2C(T &obj, void (T::*func)(P1, P2), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor2C(T &obj, void (T::*func)(P1, P2), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor2C(T *obj, void (T::*func)(P1, P2)) :
    myObj(obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor2C(T *obj, void (T::*func)(P1, P2), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor2C(T *obj, void (T::*func)(P1, P2), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Destructor
  virtual ~ArFunctor2C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

protected:

  T *myObj;
  void (T::*myFunc)(P1, P2);
  P1 myP1;
  P2 myP2;
};

/// Functor for a member function with 3 parameters
/**
   This is a class for member functions which take 3 parameters. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T, class P1, class P2, class P3>
class ArFunctor3C : public ArFunctor3<P1, P2, P3>
{
public:

  /// Constructor
  ArFunctor3C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor3C(T &obj, void (T::*func)(P1, P2, P3)) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor3C(T &obj, void (T::*func)(P1, P2, P3), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor3C(T &obj, void (T::*func)(P1, P2, P3), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArFunctor3C(T &obj, void (T::*func)(P1, P2, P3), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor3C(T *obj, void (T::*func)(P1, P2, P3)) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor3C(T *obj, void (T::*func)(P1, P2, P3), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor3C(T *obj, void (T::*func)(P1, P2, P3), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3() {} 

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArFunctor3C(T *obj, void (T::*func)(P1, P2, P3), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Destructor
  virtual ~ArFunctor3C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(myObj->*myFunc)(p1, p2, p3);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

protected:

  T *myObj;
  void (T::*myFunc)(P1, P2, P3);
  P1 myP1;
  P2 myP2;
  P3 myP3;
};



/// Functor for a member function with 4 parameters
/**
   This is a class for member functions which take 4 parameters. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T, class P1, class P2, class P3, class P4>
class ArFunctor4C : public ArFunctor4<P1, P2, P3, P4>
{
public:

  /// Constructor
  ArFunctor4C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4)) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
      @param p4 default fourth parameter
 */
  ArFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4)) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

	
  /// Destructor
  virtual ~ArFunctor4C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(myObj->*myFunc)(p1, p2, p3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) {(myObj->*myFunc)(p1, p2, p3, p4);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}


protected:

  T *myObj;
  void (T::*myFunc)(P1, P2, P3, P4);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
};


/// Functor for a member function with 5 parameters
/**
   This is a class for member functions which take 5 parameters. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class T, class P1, class P2, class P3, class P4, class P5>
class ArFunctor5C : public ArFunctor5<P1, P2, P3, P4, P5>
{
public:

  /// Constructor
  ArFunctor5C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5)) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}
  
  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
 */
ArFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
  myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5)) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
  */
  ArFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

	
  /// Destructor
  virtual ~ArFunctor5C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(myObj->*myFunc)(p1, p2, p3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) {(myObj->*myFunc)(p1, p2, p3, p4, myP5);}


  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) {(myObj->*myFunc)(p1, p2, p3, p4, p5);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

  /// Set the default fifth parameter
  /**
     @param p5 default fifth parameter
  */
  virtual void setP5(P5 p5) {myP5=p5;}


protected:

  T *myObj;
  void (T::*myFunc)(P1, P2, P3, P4, P5);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
  P5 myP5;
};






//////
//////
//////
//////
//////
//////
////// ArFunctors for member functions with return values
//////
//////
//////
//////
//////
//////


/// Functor for a member function with return value
/**
   This is a class for member functions which return a value. This class
   contains the knowledge on how to call a member function on a particular
   instance of a class. This class should be instantiated by code that
   wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class T>
class ArRetFunctorC : public ArRetFunctor<Ret>
{
public:

  /// Constructor
  ArRetFunctorC() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctorC(T &obj, Ret (T::*func)(void)) : myObj(&obj), myFunc(func) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctorC(T *obj, Ret (T::*func)(void)) : myObj(obj), myFunc(func) {}

  /// Destructor - supply function pointer
  virtual ~ArRetFunctorC() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)();}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

protected:

  T *myObj;
  Ret (T::*myFunc)(void);
};

/// Functor for a member function with return value and 1 parameter
/**
   This is a class for member functions which take 1 parameter and return
   a value. This class contains the knowledge on how to call a member
   function on a particular instance of a class. This class should be
   instantiated by code that wishes to pass off a functor to another
   piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class T, class P1>
class ArRetFunctor1C : public ArRetFunctor1<Ret, P1>
{
public:

  /// Constructor
  ArRetFunctor1C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor1C(T &obj, Ret (T::*func)(P1)) :
    myObj(&obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor1C(T &obj, Ret (T::*func)(P1), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor1C(T *obj, Ret (T::*func)(P1)) :
    myObj(obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor1C(T *obj, Ret (T::*func)(P1), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1) {}

  /// Destructor
  virtual ~ArRetFunctor1C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1);
  P1 myP1;
};

/// Functor for a member function with return value and 2 parameters
/**
   This is a class for member functions which take 2 parameters and return
   a value. This class contains the knowledge on how to call a member
   function on a particular instance of a class. This class should be
   instantiated by code that wishes to pass off a functor to another
   piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class T, class P1, class P2>
class ArRetFunctor2C : public ArRetFunctor2<Ret, P1, P2>
{
public:

  /// Constructor
  ArRetFunctor2C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor2C(T &obj, Ret (T::*func)(P1, P2)) :
    myObj(&obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor2C(T &obj, Ret (T::*func)(P1, P2), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor2C(T &obj, Ret (T::*func)(P1, P2), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor2C(T *obj, Ret (T::*func)(P1, P2)) :
    myObj(obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor2C(T *obj, Ret (T::*func)(P1, P2), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor2C(T *obj, Ret (T::*func)(P1, P2), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Destructor
  virtual ~ArRetFunctor2C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2);
  P1 myP1;
  P2 myP2;
};

/// Functor for a member function with return value and 3 parameters
/**
   This is a class for member functions which take 3 parameters and return
   a value. This class contains the knowledge on how to call a member
   function on a particular instance of a class. This class should be
   instantiated by code that wishes to pass off a functor to another
   piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class T, class P1, class P2, class P3>
class ArRetFunctor3C : public ArRetFunctor3<Ret, P1, P2, P3>
{
public:

  /// Constructor
  ArRetFunctor3C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3)) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3)) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Destructor
  virtual ~ArRetFunctor3C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) 
    {return (myObj->*myFunc)(p1, p2, p3);}

  /// Set the 'this' pointer
  /**
     @param obj object to call function on
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj object to call function on
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2, P3);
  P1 myP1;
  P2 myP2;
  P3 myP3;
};




// Start 4


/// Functor for a member function with return value and 4 parameters
/**
   This is a class for member functions which take 4 parameters and return
   a value. This class contains the knowledge on how to call a member
   function on a particular instance of a class. This class should be
   instantiated by code that wishes to pass off a functor to another
   piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class T, class P1, class P2, class P3, class P4>
class ArRetFunctor4C : public ArRetFunctor4<Ret, P1, P2, P3, P4>
{
public:

  /// Constructor
  ArRetFunctor4C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4)) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}



  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4)) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

  /// Destructor
  virtual ~ArRetFunctor4C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) 
    {return (myObj->*myFunc)(p1, p2, p3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) 
    {return (myObj->*myFunc)(p1, p2, p3, p4);}


  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2, P3, P4);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
};



/// Functor for a member function with return value and 4 parameters
/**
   This is a class for member functions which take 4 parameters and return
   a value. This class contains the knowledge on how to call a member
   function on a particular instance of a class. This class should be
   instantiated by code that wishes to pass off a functor to another
   piece of code.
   
   For an overall description of functors, see ArFunctor.
*/
template<class Ret, class T, class P1, class P2, class P3, class P4, class P5>
class ArRetFunctor5C : public ArRetFunctor5<Ret, P1, P2, P3, P4, P5>
{
public:

  /// Constructor
  ArRetFunctor5C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5)) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}


  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
  */
  ArRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}



  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func member function pointer
  */
  ArRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5)) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
  */
  ArRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}


  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
 */
  ArRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

  /// Destructor
  virtual ~ArRetFunctor5C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) 
    {return (myObj->*myFunc)(p1, p2, p3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) 
    {return (myObj->*myFunc)(p1, p2, p3, p4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) 
    {return (myObj->*myFunc)(p1, p2, p3, p4, p5);}


  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

  /// Set the default fifth parameter
  /**
     @param p5 default fifth parameter
  */
  virtual void setP5(P5 p5) {myP5=p5;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2, P3, P4, P5);
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
  P4 myP5;
};


/// Swig doesn't like the const functors
#ifndef SWIG

//////
//////
//////
//////
//////
//////
////// ArFunctors for const member functions
//////
//////
//////
//////
//////
//////


/// Functor for a const member function
/**
   This is a class for const member functions. This class contains the
   knowledge on how to call a const member function on a particular
   instance of a class.  This class should be instantiated by code
   that wishes to pass off a functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  

   @swigomit
*/
template<class T>
class ArConstFunctorC : public ArFunctor
{
public:

  /// Constructor
  ArConstFunctorC() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctorC(T &obj, void (T::*func)(void) const) : myObj(&obj), myFunc(func) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctorC(T *obj, void (T::*func)(void) const) : myObj(obj), myFunc(func) {}

  /// Destructor
  virtual ~ArConstFunctorC() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)();}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

protected:

  T *myObj;
  void (T::*myFunc)(void) const;
};


/// Functor for a const member function with 1 parameter
/**
   This is a class for const member functions which take 1
   parameter. This class contains the knowledge on how to call a const
   member function on a particular instance of a class. This class
   should be instantiated by code that wishes to pass off a functor to
   another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class T, class P1>
class ArConstFunctor1C : public ArFunctor1<P1>
{
public:

  /// Constructor
  ArConstFunctor1C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor1C(T &obj, void (T::*func)(P1) const) :
    myObj(&obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor1C(T &obj, void (T::*func)(P1) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor1C(T *obj, void (T::*func)(P1) const) :
    myObj(obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor1C(T *obj, void (T::*func)(P1) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1) {}

  /// Destructor
  virtual ~ArConstFunctor1C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

protected:

  T *myObj;
  void (T::*myFunc)(P1) const;
  P1 myP1;
};


/// Functor for a const member function with 2 parameters
/**
   This is a class for const member functions which take 2
   parameters. This class contains the knowledge on how to call a
   const member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class T, class P1, class P2>
class ArConstFunctor2C : public ArFunctor2<P1, P2>
{
public:

  /// Constructor
  ArConstFunctor2C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor2C(T &obj, void (T::*func)(P1, P2) const) :
    myObj(&obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor2C(T &obj, void (T::*func)(P1, P2) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor2C(T &obj, void (T::*func)(P1, P2) const, P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor2C(T *obj, void (T::*func)(P1, P2) const) :
    myObj(obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor2C(T *obj, void (T::*func)(P1, P2) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor2C(T *obj, void (T::*func)(P1, P2) const, P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Destructor
  virtual ~ArConstFunctor2C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

protected:

  T *myObj;
  void (T::*myFunc)(P1, P2) const;
  P1 myP1;
  P2 myP2;
};

/// Functor for a const member function with 3 parameters
/**
   This is a class for const member functions which take 3
   parameters. This class contains the knowledge on how to call a
   const member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class T, class P1, class P2, class P3>
class ArConstFunctor3C : public ArFunctor3<P1, P2, P3>
{
public:

  /// Constructor
  ArConstFunctor3C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor3C(T &obj, void (T::*func)(P1, P2, P3) const) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor3C(T &obj, void (T::*func)(P1, P2, P3) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor3C(T &obj, void (T::*func)(P1, P2, P3) const, P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstFunctor3C(T &obj, void (T::*func)(P1, P2, P3) const, P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor3C(T *obj, void (T::*func)(P1, P2, P3) const) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor3C(T *obj, void (T::*func)(P1, P2, P3) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor3C(T *obj, void (T::*func)(P1, P2, P3) const, P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3() {} 

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstFunctor3C(T *obj, void (T::*func)(P1, P2, P3) const, P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Destructor
  virtual ~ArConstFunctor3C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2, myP3);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(myObj->*myFunc)(p1, p2, p3);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

protected:

  T *myObj;
  void (T::*myFunc)(P1, P2, P3) const;
  P1 myP1;
  P2 myP2;
  P3 myP3;
};

/// Functor for a const member function with 4 parameters
/**
   This is a class for const member functions which take 4
   parameters. This class contains the knowledge on how to call a
   const member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class T, class P1, class P2, class P3, class P4>
class ArConstFunctor4C : public ArFunctor4<P1, P2, P3, P4>
{
public:

  /// Constructor
  ArConstFunctor4C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4) const) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
      @param p4 default fourth parameter
 */
  ArConstFunctor4C(T &obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4) const) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArConstFunctor4C(T *obj, void (T::*func)(P1, P2, P3, P4), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

	
  /// Destructor
  virtual ~ArConstFunctor4C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2, myP3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(myObj->*myFunc)(p1, p2, p3, myP4);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) {(myObj->*myFunc)(p1, p2, p3, p4);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}


protected:

  T *myObj;
  void (T::*myFunc)(P1, P2, P3, P4) const;
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
};

/// Functor for a const member function with 4 parameters
/**
   This is a class for const member functions which take 4
   parameters. This class contains the knowledge on how to call a
   const member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class T, class P1, class P2, class P3, class P4, class P5>
class ArConstFunctor5C : public ArFunctor5<P1, P2, P3, P4, P5>
{
public:

  /// Constructor
  ArConstFunctor5C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5) const) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
      @param p4 default fourth parameter
 */
  ArConstFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}


  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
 */
  ArConstFunctor5C(T &obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5) const) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArConstFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}


  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
  */
  ArConstFunctor5C(T *obj, void (T::*func)(P1, P2, P3, P4, P5), P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

	
  /// Destructor
  virtual ~ArConstFunctor5C() {}

  /// Invokes the functor
  virtual void invoke(void) {(myObj->*myFunc)(myP1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
  */
  virtual void invoke(P1 p1) {(myObj->*myFunc)(p1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual void invoke(P1 p1, P2 p2) {(myObj->*myFunc)(p1, p2, myP3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual void invoke(P1 p1, P2 p2, P3 p3) {(myObj->*myFunc)(p1, p2, p3, myP4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4) {(myObj->*myFunc)(p1, p2, p3, p4, myP5);}

  /// Invokes the functor
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
 */
  virtual void invoke(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) {(myObj->*myFunc)(p1, p2, p3, p4, p5);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

  /// Set the default fifth parameter
  /**
     @param p5 default fifth parameter
  */
  virtual void setP5(P5 p5) {myP5=p5;}


protected:

  T *myObj;
  void (T::*myFunc)(P1, P2, P3, P4, P5) const;
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
  P4 myP5;
};






//////
//////
//////
//////
//////
//////
////// ArFunctors for const member functions with return values
//////
//////
//////
//////
//////
//////


/// Functor for a const member function with return value
/**
   Use this when the function you are targetting is a const method.
   For example, use <tt>ArConstRetFunctorC<double, ArRobot>(&robot,
ArRobot::getX)</tt> to create a functor targetting the ArRobot method
   declared with: <tt>double getX() const;</tt>.
   
   For an overall description of functors, see ArFunctor.  */
template<class Ret, class T>
class ArConstRetFunctorC : public ArRetFunctor<Ret>
{
public:

  /// Constructor
  ArConstRetFunctorC() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctorC(T &obj, Ret (T::*func)(void) const) : myObj(&obj), myFunc(func) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctorC(T *obj, Ret (T::*func)(void) const) : myObj(obj), myFunc(func) {}

  /// Destructor - supply function pointer
  virtual ~ArConstRetFunctorC() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)();}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

protected:

  T *myObj;
  Ret (T::*myFunc)(void) const;
};

/// Functor for a const member function with return value and 1 parameter
/**
   This is a class for const member functions which take 1 parameter
   and return a value. This class contains the knowledge on how to
   call a member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class Ret, class T, class P1>
class ArConstRetFunctor1C : public ArRetFunctor1<Ret, P1>
{
public:

  /// Constructor
  ArConstRetFunctor1C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor1C(T &obj, Ret (T::*func)(P1) const) :
    myObj(&obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor1C(T &obj, Ret (T::*func)(P1) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor1C(T *obj, Ret (T::*func)(P1) const) :
    myObj(obj), myFunc(func), myP1() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor1C(T *obj, Ret (T::*func)(P1) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1) {}

  /// Destructor
  virtual ~ArConstRetFunctor1C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1) const;
  P1 myP1;
};

/// Functor for a const member function with return value and 2 parameters
/**
   This is a class for const member functions which take 2 parameters
   and return a value. This class contains the knowledge on how to
   call a member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class Ret, class T, class P1, class P2>
class ArConstRetFunctor2C : public ArRetFunctor2<Ret, P1, P2>
{
public:

  /// Constructor
  ArConstRetFunctor2C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor2C(T &obj, Ret (T::*func)(P1, P2) const) :
    myObj(&obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor2C(T &obj, Ret (T::*func)(P1, P2) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor2C(T &obj, Ret (T::*func)(P1, P2) const, P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor2C(T *obj, Ret (T::*func)(P1, P2) const) :
    myObj(obj), myFunc(func), myP1(), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor2C(T *obj, Ret (T::*func)(P1, P2) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor2C(T *obj, Ret (T::*func)(P1, P2) const, P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2) {}

  /// Destructor
  virtual ~ArConstRetFunctor2C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2) const;
  P1 myP1;
  P2 myP2;
};

/// Functor for a const member function with return value and 3 parameters
/**
   This is a class for const member functions which take 3 parameters
   and return a value. This class contains the knowledge on how to
   call a member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class Ret, class T, class P1, class P2, class P3>
class ArConstRetFunctor3C : public ArRetFunctor3<Ret, P1, P2, P3>
{
public:

  /// Constructor
  ArConstRetFunctor3C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3) const) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3) const, P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstRetFunctor3C(T &obj, Ret (T::*func)(P1, P2, P3) const, P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3) const) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3) const, P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstRetFunctor3C(T *obj, Ret (T::*func)(P1, P2, P3) const, P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3) {}

  /// Destructor
  virtual ~ArConstRetFunctor3C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2, myP3);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) 
    {return (myObj->*myFunc)(p1, p2, p3);}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2, P3) const;
  P1 myP1;
  P2 myP2;
  P3 myP3;
};




// Start 4


/// Functor for a const member function with return value and 4 parameters
/**
   This is a class for const member functions which take 4 parameters
   and return a value. This class contains the knowledge on how to
   call a member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class Ret, class T, class P1, class P2, class P3, class P4>
class ArConstRetFunctor4C : public ArRetFunctor4<Ret, P1, P2, P3, P4>
{
public:

  /// Constructor
  ArConstRetFunctor4C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4) const) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArConstRetFunctor4C(T &obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}



  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4) const) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArConstRetFunctor4C(T *obj, Ret (T::*func)(P1, P2, P3, P4) const, P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4) {}

  /// Destructor
  virtual ~ArConstRetFunctor4C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2, myP3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) 
    {return (myObj->*myFunc)(p1, p2, p3, myP4);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) 
    {return (myObj->*myFunc)(p1, p2, p3, p4);}


  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2, P3, P4) const;
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
};




/// Functor for a const member function with return value and 5 parameters
/**
   This is a class for const member functions which take 5 parameters
   and return a value. This class contains the knowledge on how to
   call a member function on a particular instance of a class. This
   class should be instantiated by code that wishes to pass off a
   functor to another piece of code.
   
   For an overall description of functors, see ArFunctor.  */
template<class Ret, class T, class P1, class P2, class P3, class P4, class P5>
class ArConstRetFunctor5C : public ArRetFunctor5<Ret, P1, P2, P3, P4, P5>
{
public:

  /// Constructor
  ArConstRetFunctor5C() {}

  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5) const) :
    myObj(&obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2, P3 p3) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
  */
  ArConstRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
  */
  ArConstRetFunctor5C(T &obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(&obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}



  /// Constructor - supply function pointer
  /**
     @param obj object to call function on
     @param func const member function pointer
  */
  ArConstRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5) const) :
    myObj(obj), myFunc(func), myP1(), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
  */
  ArConstRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1) :
    myObj(obj), myFunc(func), myP1(p1), myP2(), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
  */
  ArConstRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
  */
  ArConstRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2, P3 p3) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(), myP5() {}

  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
 */
  ArConstRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2, P3 p3, P4 p4) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5() {}


  /// Constructor - supply function pointer, default parameters
  /**
     @param obj object to call function on
     @param func const member function pointer
     @param p1 default first parameter
     @param p2 default second parameter
     @param p3 default third parameter
     @param p4 default fourth parameter
     @param p5 default fifth parameter
 */
  ArConstRetFunctor5C(T *obj, Ret (T::*func)(P1, P2, P3, P4, P5) const, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) :
    myObj(obj), myFunc(func), myP1(p1), myP2(p2), myP3(p3), myP4(p4), myP5(p5) {}

  /// Destructor
  virtual ~ArConstRetFunctor5C() {}

  /// Invokes the functor with return value
  virtual Ret invokeR(void) {return (myObj->*myFunc)(myP1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
  */
  virtual Ret invokeR(P1 p1) {return (myObj->*myFunc)(p1, myP2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2) {return (myObj->*myFunc)(p1, p2, myP3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 second parameter
  */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3) 
    {return (myObj->*myFunc)(p1, p2, p3, myP4, myP5);}

  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4) 
    {return (myObj->*myFunc)(p1, p2, p3, p4, myP5);}


  /// Invokes the functor with return value
  /**
     @param p1 first parameter
     @param p2 second parameter
     @param p3 third parameter
     @param p4 fourth parameter
     @param p5 fifth parameter
 */
  virtual Ret invokeR(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5) 
    {return (myObj->*myFunc)(p1, p2, p3, p4, p5);}


  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T *obj) {myObj=obj;}

  /// Set the 'this' pointer
  /**
     @param obj the 'this' pointer
  */
  virtual void setThis(T &obj) {myObj=&obj;}

  /// Set the default parameter
  /**
     @param p1 default first parameter
  */
  virtual void setP1(P1 p1) {myP1=p1;}

  /// Set the default 2nd parameter
  /**
     @param p2 default second parameter
  */
  virtual void setP2(P2 p2) {myP2=p2;}

  /// Set the default third parameter
  /**
     @param p3 default third parameter
  */
  virtual void setP3(P3 p3) {myP3=p3;}

  /// Set the default fourth parameter
  /**
     @param p4 default fourth parameter
  */
  virtual void setP4(P4 p4) {myP4=p4;}

  /// Set the default fifth parameter
  /**
     @param p5 default fifth parameter
  */
  virtual void setP5(P5 p5) {myP5=p5;}

protected:

  T *myObj;
  Ret (T::*myFunc)(P1, P2, P3, P4, P5) const;
  P1 myP1;
  P2 myP2;
  P3 myP3;
  P4 myP4;
  P5 myP5;
};


#endif // omitting Const functors from SWIG



#endif // ARFUNCTOR_H


