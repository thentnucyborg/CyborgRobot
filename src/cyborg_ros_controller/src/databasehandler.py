#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import sqlite3
import datetime
import collections
import random
import math
from collections import namedtuple

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.2"
__all__ = []


EventRecord = collections.namedtuple('EventRecord', 'event_id state event reward_pleasure reward_arousal reward_dominance event_cost event_value')


class DatabaseHandler(object):
    """DatabaseHandler for the Motivator.

    Usage:
        database_handler = DatabaseHandler(filename="path")
        
    Available:
        * add_event(state, event, reward_pleasure, reward_arousal, reward_dominance)
        * get_all_events(state)
    """

    def __init__(self, filename):
        self.dbfilename = filename


    def create(self):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            connection.execute("CREATE TABLE Event(event_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, state TEXT, event TEXT, reward_pleasure REAL, reward_arousal REAL, reward_dominance REAL, event_cost REAL, event_value)")
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Event table allready exist...")


    # Functions used for transfearing sqlite records to named touples
    def namedtuple_factory_event_record(self, cursor, row):
        return EventRecord(*row)


    # Add an event to the database of available events (actions) and returns the event_id
    def add_event(self, state, event, reward_pleasure, reward_arousal, reward_dominance, event_cost, event_value=0.00):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            cursor.execute("INSERT INTO Event (state, event, reward_pleasure, reward_arousal, reward_dominance, event_cost, event_value) VALUES (?,?,?,?,?,?,?)", (state, event, reward_pleasure, reward_arousal, reward_dominance, event_cost, event_value))
            cursor.execute("SELECT last_insert_rowid();")
            id = cursor.fetchall()
            connection.commit()
            cursor.close()
            return id[0][0]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to add events to the action database...")


    # Returns all events with maching state name
    def get_all_events(self, state):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_event_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Event WHERE state=?', (state, ))
            records = cursor.fetchall()
            cursor.close()
            return records
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to get all events to the action database (state name was " + state + " )...")

    # Set the event_value
    def update_event_value(self, event_id, event_value):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_event_record
            cursor = connection.cursor()
            cursor.execute('UPDATE Event SET event_value=? WHERE event_id=?', (event_value, event_id, ))
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to update event_value to the action database (event_id was " + event_id + " and event_value was " + event_value + " )...")


    # Add delta_event_value to all event_values
    def update_all_event_values(self, delta_event_value):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_event_record
            cursor = connection.cursor()
            cursor.execute('UPDATE Event SET event_value=event_value+?', (delta_event_value, ))
            cursor.execute('UPDATE Event SET event_value=? WHERE event_value<=?', (0,0 ))
            cursor.execute('UPDATE Event SET event_value=? WHERE event_value>=?', (1,1 ))
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to update event_value to the action database (delta_event_value was " + delta_event_values + " )...")


    # Reset event_value
    def reset_event_values(self, event_value=0.00):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_event_record
            cursor = connection.cursor()
            cursor.execute('UPDATE Event SET event_value=?', (event_value, ))
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to update event_value to the action database (event_value was " + event_values + " )...")


