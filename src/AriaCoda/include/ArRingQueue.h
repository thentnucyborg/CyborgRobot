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


#ifndef _AR_RING_QUEUE_H_
#define _AR_RING_QUEUE_H_

#include <iostream>
#include <list>
#include <ariaTypedefs.h>

/** @brief An expanding ring queue. 
 *
 *  It is used to keep a queue with a minimum of allocation and freeing of
 *  heap memory.
 *  The ring queue is implemented using std::list.  The queue starts with an 
 *  initial capacity, but those initial items are considered 'unused'.  Items
 *  are "pushed" into the queue at the "back", and "popped" from the queue at
 *  the "front". pop() and advance_front() will move the front of the queue to 
 *  the next item, creating a new 'unused slot' for future use; advance_back()
 *  changes the next item in the back to a 'used' slot. push() either uses the
 *  next 'unused' slot, or inserts a new item into the std::list.  
 *  When the capacity of the queue is filled, all operations will fail
 *  except push(). Use push() to insert new items in the queue and increase
 *  its capacity.  
 *
 *  @todo Ideally, this class would be fully threadsafe (with occasional or no mutex
 *  locking), but it is not currently.
 *  @todo Optionally allocate several future 'slots' instead of just one.
 *
 *  @ingroup UtilityClasses
 */
template<class T> 
class ArRingQueue {
public:
  /** @param capacity Initial capacity of the ring queue. 
   *  @param init_value Initial value for new, unused items in the queue. 
   *  */
  ArRingQueue(int capacity, T init_value)
    : ring(capacity, init_value), curSize(0), initval(init_value)
  {
    back_it = ring.begin(); 
    front_it = ring.end();// signals empty state
  }


  /** Get an iterator for the front item of the ring queue (the item that would
   * be returned by pop()). If the queue is currently empty, nil() will be
   * returned.
   * 
   * To remove an item from the queue without
   * making a copy with pop(), first check if the queue is empty(). Then  use this 
   * function to get the data. Then call advance_front(). 
   */
  typename std::list<T>::iterator front() {
    if(empty())
      return nil();
    return front_it;
  }

  /** Get an iterator for the back of the queue (the item that would be
   * replaced by push()). This is not the last item in the queue, rather it is the
   * next, unused, "slot".  If the queue is full, an iterator equivalent to that
   * returned by nil() is returned.
   *
   * To add an item to the queue without pushing
   * a copy with push(), first check if the queue is full (in which case you
   * must push() your item). Then use this function to write the data into the
   * next unused 'slot'. Then call advance_back() to advance the back of the queue
   * to your new item. 
   */
  typename std::list<T>::iterator back() {
    if(front_it == back_it)
    {
      //std::cerr << "ArRingQueue: back(): 0-capacity or full, returning nil.\n";
      return nil();
    }
    return back_it;
  }

  /** Advance (pop) the front of the queue. 'Used' size will be decremented.  */
  void advance_front() {
    if(front_it == ring.end())  // initial or  empty state.
      front_it = ring.begin();
    else if(++front_it == ring.end()) 
      front_it = ring.begin();
    if(front_it == back_it) { // it's now empty (not full)
      front_it = ring.end();
      back_it = ring.begin();
    }
    curSize--;
  }

  /** Same as advance_front() */
  void pop_front() { advance_front(); };

  /** Advance the back (an 'empty' push), if the queue is not full.  'Used' size will be incremented.  */
  void advance_back() {
    if(front_it == back_it) // full or 0-capacity
    {
      // debugging:
      /*
      if(empty()) {
        std::cerr << "ArRingQueue: advance_back(): queue is *empty*, can't advance back.\n";
        return;
      }
      std::cerr << "ArRingQueue: advance_back(): queue is full, can't advance back.\n";
      */
      return;
    }
    if(++back_it == ring.end())
      back_it = ring.begin();
    if(front_it == ring.end())
      front_it = ring.begin();  // no longer empty.
    curSize++;
  }

  /** Add an item to the back of the ring queue. If the queue is full, the
   * capacity of the queue will be expanded and the item
   * will be inserted. */
  void push(const T& item) {
    if(full()) {
      // expand
      back_it = ring.insert(back_it, item);
    } else {
      // back is unused, use it
      *back_it = item;
    }
    advance_back();
  }

  /** Same as push() */
  void push_back(const T& item) { push(item); }

  /** Push a new item, but preserve capacity: instead of expanding the queue if
   * full, then the oldest item is replaced and the front is advanced.
   */
  void push_without_expanding(const T& item) {
    if(full())
      advance_front();
    push(item);
  }

  /** Print the current contents of the queue. 
   *  @pynote use printQueue() instead of print() (which is a reserved word in Python)
  */
  void print() {
    for(typename std::list<T>::const_iterator i = ring.begin(); i != ring.end(); i++) {
      if(i == back_it)
        std::cerr << "]";
      if(i == front_it || (i == ring.begin() && front_it == ring.end()) )
        std::cerr << "[";
      std::cerr << (*i) << "," ;
    }
    std::cerr << std::endl;
  }

  /** Get the number of items currently 'used' in the queue. */
  size_t size() {
    return curSize;
  }

  /** Get the current capacity of the queue. */
  size_t capacity() {
    return ring.size();
  }

  /** Return true if the queue is empty (has no 'used' items), false otherwise.  */
  bool empty() {
    return (front_it == ring.end());
  }

  /** Logically clear the queue, resetting to initial empty state, but preserving current
   * capacity, and leaving all contents as they are; the contents are not
   * destroyed, but will be when replaced by new data later. */
  void reset() {
    front_it = ring.end();
    back_it = ring.begin();
    curSize = 0;
  }

  /** Return true if the queue is full, false otherwise. */
  bool full() {
    return (back_it == front_it);
  }

  /** Return an iterator representing an invalid item. Compare to the return
   * values of front(), back(), pop(), etc. */
  typename std::list<T>::iterator nil() {
    return ring.end();
  }

protected:
  std::list<T> ring;
  typename std::list<T>::iterator front_it, back_it;   
  // push to back, pop from front; front will point to first item, 
  // back to one past last. 

  size_t curSize;
  T initval;

};



#endif
