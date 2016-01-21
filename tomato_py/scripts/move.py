#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Point

def callback(point):
    # moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    leftArm = moveit_commander.MoveGroupCommander("left_arm")

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the right
    ## arm.  This interface can be used to plan and execute motions on the right
    ## arm.
    rightArm = moveit_commander.MoveGroupCommander("right_arm")

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(10)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    # print "============ Reference frame: %s" % leftArm.get_planning_frame()

    # ## We can also print the name of the end-effector link for this group
    # print "============ Reference frame: %s" % leftArm.get_end_effector_link()

    # ## We can get a list of all the groups in the robot
    # print "============ Robot Groups:"
    # print robot.get_group_names()

    # ## Sometimes for debugging it is useful to print the entire state of the
    # ## robot.
    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print "============"

    # Planning to a joint-space goal 
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #
    # Let's set a joint space goal and move towards it. 
    # First, we will clear the pose target we had just set.

    leftArm.clear_pose_targets()
    rightArm.clear_pose_targets()

    ## Then, we will get the current set of joint values for the group
    leftArm_variable_values = leftArm.get_current_joint_values()
    print "============ Joint values left: ", leftArm_variable_values

    rightArm_variable_values = rightArm.get_current_joint_values()
    print "============ Joint values right: ", rightArm_variable_values

    ## Move to the tomato, but don't grab it yet
    leftArm_variable_values[0] = point.x
    leftArm_variable_values[1] = point.y # + 0.05 # first not grab it
    leftArm_variable_values[2] = point.z
    leftArm.set_joint_value_target(leftArm_variable_values)

    rightArm_variable_values[0] = point.x
    rightArm_variable_values[1] = point.y #- 0.05 # first not grab it
    rightArm_variable_values[2] = point.z
    rightArm.set_joint_value_target(rightArm_variable_values)

    planLeft = leftArm.plan()
    planRight = rightArm.plan()

    leftArm.go(wait=True)
    rightArm.go(wait=True)

    print "============ Waiting while RVIZ displays move to tomato..."
    rospy.sleep(5)

    ## Grab the tomato
    leftArm_variable_values[1] = point.y + 0.02 # grab tomato
    leftArm.set_joint_value_target(leftArm_variable_values)

    rightArm_variable_values[1] = point.y - 0.02 # grab tomato
    rightArm.set_joint_value_target(rightArm_variable_values)

    planLeft = leftArm.plan()
    planRight = rightArm.plan()

    leftArm.go(wait=True)
    rightArm.go(wait=True)

    print "============ Waiting while RVIZ displays grab tomato..."
    rospy.sleep(5)

    ## Lift arms a bit up and move to the robots chest to not hit the table and to be more stable while moving 
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

    print "============ Waiting while RVIZ displays lift arms..."
    rospy.sleep(5)


    ## Then, we will get the current set of joint values for the group
    leftArm_variable_values = leftArm.get_current_joint_values()
    print "============ Joint values left: ", leftArm_variable_values

    rightArm_variable_values = rightArm.get_current_joint_values()
    print "============ Joint values right: ", rightArm_variable_values


    ## Moving to a pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^
    ##
    ## Moving to a pose goal is similar to the step above
    ## except we now use the go() function. Note that
    ## the pose goal we had set earlier is still active 
    ## and so the robot will try to move to that goal. We will
    ## not use that function in this tutorial since it is 
    ## a blocking function and requires a controller to be active
    ## and report success on execution of a trajectory.

    # Uncomment below line when working with a real robot
    leftArm.go(wait=True)
    rightArm.go(wait=True)

    ## Adding/Removing Objects and Attaching/Detaching Objects
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will define the collision object message
    # collision_object = moveit_msgs.msg.CollisionObject()


    ## When finished shut down moveit_commander.
    # moveit_commander.roscpp_shutdown()

    rospy.loginfo("Picked up the tomato")

    print "============ STOPPING"

def main():
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