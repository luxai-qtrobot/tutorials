#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import math
import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from moveit_commander.conversions import pose_to_list

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_spiral', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")

    rospy.sleep(3)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # print "============ Printing robot state"
    # print(group.get_current_pose().pose)
    # print(group.get_pose_reference_frame())

    group.allow_replanning(True)
    group.set_pose_reference_frame("base_link")
    group.set_planning_time(5.0)
    group.clear_path_constraints()
    group.clear_pose_targets()

    # move to starting point
    group.set_start_state_to_current_state()

    # go to start pose
    group.set_position_target([0.23, -0.23, 0.36])
    plan = group.go(wait=True)
    print("Reached starting point")

    # generate waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    r = 0.002
    for j in range(1, 10):
        for i in range(0, 360):
            theta = math.radians(i)
            wpose.position.x =  0.21 + r*j * math.cos(math.radians(i))
            wpose.position.y = -0.21 + r*j * math.sin(math.radians(i))
            if i == 0 and j == 0:
                wpose.position.z = 0.36
            else:
                wpose.position.z = 0.33
            waypoints.append(copy.deepcopy(wpose))
            # print(wpose.position.x, wpose.position.y)

    # plan trajectory
    plan, fraction = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0,         # jump_threshold
                                   False)       # avoid_collisions
    # execute the plan
    group.execute(plan, True)
