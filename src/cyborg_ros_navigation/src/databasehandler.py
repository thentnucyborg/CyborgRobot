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

LocationRecord = collections.namedtuple('LocationRecord', 'location_name robot_map_name x y z p j r threshold crowded environment')
EventRecord = collections.namedtuple('EventRecord', ["event_id", 'event_name', "location_name", "start_date", "end_date", "ignore", "robot_map_name", 'x', "y", "z", "p", "j", "r", "threshold", "crowded", "environment"])
ResponseRecord = collections.namedtuple('ResponseRecord', ["response_id", 'message', "response_type", "emotion"])

class DatabaseHandler(object):
    """DatabaseHandler"""

    def __init__(self, filename):
        self.dbfilename = filename

    def create(self):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            connection.execute("CREATE TABLE Location(location_name TEXT PRIMARY KEY NOT NULL, robot_map_name TEXT, x REAL, y REAL, z REAL, p REAL, j REAL, r REAL, threshold REAL, crowded BOOLEAN, environment REAL)")
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Location table allready exist...")

        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            connection.execute("CREATE TABLE Event(event_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, event_name TEXT, location_name TEXT, start_date DATETIME, end_date DATETIME, ignore BOOLEAN, FOREIGN KEY(location_name) REFERENCES Location(location_name))")
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Event table allready exist...")

        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            connection.execute("CREATE TABLE Response(response_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, message TEXT, response_type TEXT, emotion TEXT)")
            connection.commit()
            cursor.close()
        except sqlite3.OperationalError:
            print("DatabaseHandler: Response table allready exist...")

    def namedtuple_factory_location_record(self, cursor, row):
        return LocationRecord(*row)

    def namedtuple_factory_event_record(self, cursor, row):
        return EventRecord(*row)

    def namedtuple_factory_response_record(self, cursor, row):
        return ResponseRecord(*row)


    def add_response(self, message, response_type, emotion):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            cursor.execute("INSERT INTO Response (message, response_type, emotion) VALUES (?,?,?)", (message, response_type, emotion))
            cursor.execute("SELECT last_insert_rowid();")
            id = cursor.fetchall()
            connection.commit()
            cursor.close()
            return id[0][0]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to add_response()...")


    def search_for_response(self, response_type, emotion):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_response_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Response WHERE response_type=? AND emotion=?', (response_type, emotion ))
            records = cursor.fetchall()
            cursor.close()
            if len(records) == 0: 
                return None
            elif len(records) == 1: 
                return records[0]
            else: 
                n = random.randrange(start=0, stop=(len(records)-1))
                return records[n]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to search_for_response()...")


    def search_for_location(self, location_name):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_location_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Location WHERE location_name=?', (location_name, ))
            records = cursor.fetchall()
            cursor.close()
            print(records)
            return records[0] if len(records) > 0 else None
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to search_for_location()...")


    def get_all_locations(self):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_location_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Location', ( ))
            records = cursor.fetchall()
            cursor.close()
            return records
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to get_all_locations()...")


    def add_location(self, location_name, robot_map_name="ntnu.map", x=0, y=0, z=0, p=0, j=0, r=0, threshold=0.00, crowded=False, environment=0.00):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            cursor.execute("INSERT INTO Location (location_name, robot_map_name, x, y, z, p, j, r, threshold, crowded, environment) VALUES (?,?,?,?,?,?,?,?,?,?,?)", (location_name, robot_map_name, x, y, z, p, j, r, threshold, crowded, environment))
            cursor.execute("SELECT last_insert_rowid();")
            id = cursor.fetchall()
            connection.commit()
            cursor.close()
            return id[0][0]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to add_location()...")

        
    def add_event(self, event_name, location_name, start_date=datetime.datetime.now(), end_date=datetime.datetime.now(), ignore=False):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            cursor.execute("INSERT INTO Event (event_name, location_name, start_date, end_date, ignore) VALUES (?,?,?,?,?)", (event_name, location_name, start_date, end_date, ignore))
            cursor.execute("SELECT last_insert_rowid();")
            id = cursor.fetchall()
            connection.commit()
            cursor.close()
            return id[0][0]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to add_event()...")


    def search_ongoing_events(self, robot_map_name, current_date=datetime.datetime.now()):
        try:
            print("DatabaseHandler: Searching for ongoing events...")

            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_event_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Event natural join Location WHERE Event.start_date<? and Event.end_date>? and Location.robot_map_name=? and Event.ignore=? ORDER BY Event.start_date DESC', (current_date, current_date, robot_map_name, False))
            records = cursor.fetchall()

            print("DatabaseHandler: found: ")
            print(len(records))

            cursor.close()
            if len(records) == 0:
                return None
            else:
                return records[0]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to search_ongoing_events()...")


    def search_for_crowded_locations(self, robot_map_name, crowded=True):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_location_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Location WHERE robot_map_name=? AND crowded=?', (robot_map_name, crowded))
            records = cursor.fetchall()
            cursor.close()
            if len(records) == 0: 
                return None
            elif len(records) == 1: 
                return records[0]
            else: 
                n = random.randrange(start=0, stop=(len(records)-1))
                return records[n]
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to search_for_crowded_locations()...")


    def location_is_crowded(self, robot_map_name, location_name):
        try:
            connection = sqlite3.connect(self.dbfilename)
            cursor = connection.cursor()
            cursor.execute('SELECT * from Location WHERE robot_map_name=? AND crowded=? AND location_name=?', (robot_map_name, True, location_name))
            records = cursor.fetchall()
            cursor.close()
            return True if len(records) > 0 else False
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to location_is_crowded()...")


    def find_location(self, robot_map_name, location_x, location_y):
        try:
            connection = sqlite3.connect(self.dbfilename)
            connection.row_factory = self.namedtuple_factory_location_record
            cursor = connection.cursor()
            cursor.execute('SELECT * from Location WHERE robot_map_name=?', (robot_map_name, ))
            records = cursor.fetchall()
            cursor.close()
            if len(records) == 0: 
                return None
            else:
                best = None
                for record in records:
                    distance = math.sqrt((location_x - record.x)**2 + (location_y - record.y)**2) 
                    if distance < record.threshold:
                        best = record
                return best
        except sqlite3.OperationalError:
            print("DatabaseHandler: Unable to find_location()...")

