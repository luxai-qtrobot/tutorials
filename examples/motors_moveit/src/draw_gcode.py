#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
import re
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

TABLE_HEIGH = 0.396
CX = 0.22   # x length 0.04
CY = -0.14  # y length 0.12

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

    # move to start waypoint
    wpose = group.get_current_pose().pose
    group.set_position_target([wpose.position.x, wpose.position.y, wpose.position.z + 0.03])
    plan = group.go(wait=True)
    group.set_position_target([CX, CY, TABLE_HEIGH+0.03])
    plan = group.go(wait=True)
    # wpose = group.get_current_pose().pose
    # group.set_position_target([wpose.position.x, wpose.position.y, wpose.position.z-0.03])
    # plan = group.go(wait=True)
    print("Reached starting point")

    # fill waypoints with data from gcode
    waypoints = []
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    gcode_file = rospy.get_param('gcode', './data/drawing.gcode')
    with open(gcode_file, 'r') as gcode:
        for line in gcode:
            line = line.strip()
            coord = re.findall(r'[XY].?\d+.\d+', line)
            if coord:
                # reverse x and y and convert mm to m
                wpose.position.y = CY + float(coord[0][1:]) / 1000.0
                wpose.position.x = CX + float(coord[1][1:]) / 1000.0
                wpose.position.z = TABLE_HEIGH + 0.03 if float(coord[0][1:]) == 0 and float(coord[1][1:]) == 0 else TABLE_HEIGH
                waypoints.append(copy.deepcopy(wpose))
                print(wpose.position.x, wpose.position.y, wpose.position.z)

    # plan trajectorys
    plan, fraction = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0,         # jump_threshold
                                   False)       # avoid_collisions
    # execute the plan
    group.execute(plan, True)

    # # draw borders
    # waypoints = []
    #
    # # right top
    # wpose = group.get_current_pose().pose
    # waypoints.append(copy.deepcopy(wpose))
    #
    # # left top
    # wpose.position.y += 0.12
    # waypoints.append(copy.deepcopy(wpose))
    #
    # # left buttom
    # wpose.position.x -= 0.04
    # waypoints.append(copy.deepcopy(wpose))
    #
    # # right buttom
    # wpose.position.y -= 0.12
    # waypoints.append(copy.deepcopy(wpose))
    #
    # # right top
    # wpose.position.x += 0.04
    # waypoints.append(copy.deepcopy(wpose))
    #
    # # plan trajectorys
    # plan, fraction = group.compute_cartesian_path(
    #                                waypoints,   # waypoints to follow
    #                                0.01,        # eef_step
    #                                0.0,         # jump_threshold
    #                                False)       # avoid_collisions
    # # execute the plan
    # group.execute(plan, True)
    #
