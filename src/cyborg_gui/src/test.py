#!/usr/bin/env python
from pymongo import MongoClient 

# client = MongoClient("mongodb+srv://cyborg:hmPHK#4.iunGKD2@cyborg-gui-mg1nk.azure.mongodb.net/test?retryWrites=true&w=majority")
# # Access database 
# mydb = client['cyborg_data'] 

# # Access collection of the database 
# mycol = mydb['primarystates_goal'] 

# # dictionary to be added in the database 
# rec={ 
#     "title": 'MongoDB and Python',  
#     "description": 'TESTING2',  
#     "tags": ['mongodb', 'database', 'NoSQL'],  
#     "viewers": 104
# } 

# # inserting the data in the database
# x = mydb.primarystates_goal.insert(rec) 
# rec = {"TITLE":"HELLO"}
# x = mycol.insert(rec)

rec ={
  "header": {
    "stamp": {
      "secs": 1573139359,
      "nsecs": 599050998
    },
    "frame_id": "",
    "seq": 1
  },
  "status_list": []
}

print(rec)
client = MongoClient("mongodb+srv://cyborg:hmPHK#4.iunGKD2@cyborg-gui-mg1nk.azure.mongodb.net/test?retryWrites=true&w=majority")
# Access database 
mydb = client['cyborg_data'] 

# Access collection of the database 
db_collection = mydb['ControllerViewer_SmachContainerStructure'] 

# dictionary to be added in the database 

# inserting the data in the database
db_collection.insert(rec)
# rospy.sleep(interval)