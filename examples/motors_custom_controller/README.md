# An advanced example of implementing ROS controller for QTrobot
This example shows how to write a new ROS controller for QTrobot. Before getting into that, make sure you know and understand what is [ROS Conrol](http://wiki.ros.org/ros_control).

## What it does?
this controller simply play a predefined trajectory directly by commanding the QTrobot motors at hardware level.  You can use it as a simple template to implement more sophisticate controllers.  

## Compilation and build
* copy `motors_custom_controller` to the `~/catckin_ws`on the RPI of QTrobot

* build the example code:
```
cd ~/catckin_ws
catkin_make --pkg motors_custom_controller
```
## Configuration and launching
* modify the qt_motor config file to define your new controller. to do that open `/opt/ros/kinetic/share/qt_motor/config/qtrobot-controller.yaml` file and add the following lines to the end of the file.
```
  custom:
    type: motors_custom_controller/QTCustomController
    myparam: bar

```
* modify qt_motor launch file to load and run your controller. to do that, open `/opt/ros/kinetic/share/qt_motor/launch/qt_motor.launch` and add `/qt_robot/custom` controller to `controller_manager` node. it should look like this:

```
<!-- load the controllers -->
 <node pkg="controller_manager" name="controller" type="spawner" launch-prefix="bash -c 'sleep 15; $0 $@' " respawn="false"
       output="screen" clear_params="true"  args="/qt_robot/head_position
                             /qt_robot/right_arm_position
                             /qt_robot/left_arm_position
                             /qt_robot/joints_state
                             /qt_robot/motors
                             /qt_robot/gesture
                             /qt_robot/custom"/>
```

## Re-launch qt_motor node
you can simply reboot the robot or launch the `qt_motor` node from terminal to see its output.
* stop the running instance:
```
ps -aux | grep qt_motor | grep SCREEN
...
kill xxx
```

* launch `qt_motor` node
```
roslaunch qt_motor qt_motor.launch
```

you should see your controller (`QTCustomController`) running by checking the terminal output:
```
...
[INFO] [1576498210.924577]: Controller Spawner: Loaded controllers: /qt_robot/head_position, /qt_robot/right_arm_position, /qt_robot/left_arm_position, /qt_robot/joints_state, /qt_robot/motors, /qt_robot/gesture, /qt_robot/custom
[ INFO] [1576498210.984396566]: QTMotorsController: starting
[ INFO] [1576498210.985034385]: QTGestureController: starting
[ INFO] [1576498210.985518609]: QTCustomController: starting
...
```

## Testing
your custom controller implement a service to Start/Stop it.

* to start the trajectory player, open another termianl and
```
rosservice call /qt_robot/custom/startstop "command: true"
```

* to stop the trajectory player:
```
rosservice call /qt_robot/custom/startstop "command: false"
```
