#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import copy
# import rospy
# import moveit_commander
import re

TABLE_HEIGH = 0.396
PARK_POSE = [0.20, -0.20, TABLE_HEIGH + 0.03]
CX = 0.22   # x length 0.04
CY = -0.14  # y length 0.12

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_rectangle', anonymous=True)

    # extract coordination from gcode file
    gcode_file = rospy.get_param('gcode', './data/drawing.gcode')
    coordinations = []
    x, y, z = PARK_POSE[0], PARK_POSE[1], PARK_POSE[2]
    with open(gcode_file, 'r') as gcode:
        for line in gcode:
            if "pen park" in line:
                z = TABLE_HEIGH + 0.03
                coordinations.append([x,y,z])
                print("up   %.3f, %.3f, %.3f" % (x,y, z))
                x, y, z = PARK_POSE[0], PARK_POSE[1], PARK_POSE[2]
                coordinations.append([x,y,z])
                print("park %.3f, %.3f, %.3f" % (x,y, z))
            line = line.strip()
            if "pen down" in line:
                z = TABLE_HEIGH
                coordinations.append([x,y,z])
                print("down %.3f, %.3f, %.3f" % (x,y, z))
            elif "pen up" in line:
                z = TABLE_HEIGH + 0.03
                coordinations.append([x,y,z])
                print("up   %.3f, %.3f, %.3f" % (x,y, z))
            elif "draw" in line or "move" in line:
                c = re.findall(r'[XY].?\d+.\d+', line)
                if c:
                    y = CY + float(c[0][1:]) / 1000.0
                    x = CX + float(c[1][1:]) / 1000.0
                    coordinations.append([x,y,z])
                    print("goto %.3f, %.3f, %.3f" % (x,y, z))


    # create robot commnader and moveGroup
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


    group.allow_replanning(True)
    group.set_pose_reference_frame("base_link")
    group.set_planning_time(5.0)
    group.clear_path_constraints()
    group.clear_pose_targets()

    # move to starting point
    group.set_start_state_to_current_state()

    # move to start waypoint
    wpose = group.get_current_pose().pose
    group.set_position_target(coordinations[0])
    plan = group.go(wait=True)
    print("Reached starting point")

    # fill waypoints with data from gcode
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    for cord in coordinations:
        wpose.position.x = cord[0]
        wpose.position.y = cord[1]
        wpose.position.z = cord[2]
        waypoints.append(copy.deepcopy(wpose))

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
