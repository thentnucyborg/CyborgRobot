cmake_minimum_required(VERSION 2.8.3)
project(cyborg_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
    BehaviorTree.msg
    BehaviorTreeNodes.msg
    GoalWithHeading.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    AvailableGoals.srv
    ClosestGoal.srv
    DistanceToGoal.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()
