#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_rectangle', anonymous=True)

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

    # draw on ZY plane
    ####################
    # # go to start pose
    # group.set_position_target([0.18, -0.25, 0.35])
    # plan = group.go(wait=True)
    # print("Reached starting point")
    #
    # # generate waypoints
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # waypoints.append(copy.deepcopy(wpose))
    # for i in range(5):
    #     wpose.position.z += -0.05
    #     waypoints.append(copy.deepcopy(wpose))
    #     wpose.position.y += 0.05
    #     waypoints.append(copy.deepcopy(wpose))
    #     wpose.position.z += 0.05
    #     waypoints.append(copy.deepcopy(wpose))
    #     wpose.position.y += -0.05
    #     waypoints.append(copy.deepcopy(wpose))

    # draw on XY plane
    ####################
    group.set_position_target([0.23, -0.23, 0.35])
    plan = group.go(wait=True)
    print("Reached starting point")

    # generate waypoints
    waypoints = []
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))

    for i in range(3):
        wpose.position.y += 0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += -0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += -0.03
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += 0.03
        waypoints.append(copy.deepcopy(wpose))

    # plan trajectory
    plan, fraction = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0,         # jump_threshold
                                   False)       # avoid_collisions

    # execute the plan
    group.execute(plan, True)
