#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

def callback(point):
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(1)

    p = PoseStamped()
    p.header.frame_id = "robot"
    p.pose.position.x = 1
    p.pose.position.y = 1
    p.pose.position.z = 0
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1

    world = "../world.stl" 
    scene.add_mesh("world", p, world)

    collision_object = moveit_msgs.msg.CollisionObject()

    # Movegroups 
    leftArm = moveit_commander.MoveGroupCommander("left_arm")
    rightArm = moveit_commander.MoveGroupCommander("right_arm")

    leftArm.clear_pose_targets()
    rightArm.clear_pose_targets()

    # Current set of joints
    leftArm_variable_values = leftArm.get_current_joint_values()
    rightArm_variable_values = rightArm.get_current_joint_values()


    print "============ Move to tomato..."

    # Move to the tomato, but don't grab it yet
    leftArm_variable_values[0] = point.x
    leftArm_variable_values[1] = point.y  + 0.05 # first not grab it
    leftArm_variable_values[2] = point.z
    leftArm.set_joint_value_target(leftArm_variable_values)

    rightArm_variable_values[0] = point.x
    rightArm_variable_values[1] = point.y - 0.05 # first not grab it
    rightArm_variable_values[2] = point.z
    rightArm.set_joint_value_target(rightArm_variable_values)

    planLeft = leftArm.plan()
    planRight = rightArm.plan()

    leftArm.go(wait=True)
    rightArm.go(wait=True)

    print "============ Grab tomato..."

    # Grab the tomato
    leftArm_variable_values[1] = point.y + 0.02 # grab tomato
    leftArm.set_joint_value_target(leftArm_variable_values)

    rightArm_variable_values[1] = point.y - 0.02 # grab tomato
    rightArm.set_joint_value_target(rightArm_variable_values)

    planLeft = leftArm.plan()
    planRight = rightArm.plan()

    leftArm.go(wait=True)
    rightArm.go(wait=True)

    print "============ Lift arms..."
    # Lift arms a bit up and move to the robots chest to not hit the table 
    # and to be more stable while moving 
    leftArm_variable_values[0] = point.x - 0.1 # move to chest
    leftArm_variable_values[2] = point.z + 0.1 # lift arm
    leftArm.set_joint_value_target(leftArm_variable_values)

    rightArm_variable_values[0] = point.x - 0.1 # move to chest
    rightArm_variable_values[2] = point.z + 0.1 # lift arm
    rightArm.set_joint_value_target(rightArm_variable_values)

    planLeft = leftArm.plan()
    planRight = rightArm.plan()

    leftArm.go(wait=True)
    rightArm.go(wait=True)

    # rospy.sleep(5)

    rospy.loginfo("Picked up the tomato")

    print "============ STOPPING"

    moveit_commander.roscpp_shutdown()

def main():
    moveit_commander.roscpp_initialize(sys.argv)    
    rospy.init_node('moveit', anonymous=True)
    rospy.Subscriber("point", Point, callback)
    rospy.loginfo("Starting moveit")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
