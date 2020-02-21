---
title: Simple kinematics for mobile robot
sidebar_label: 3. Simple kinematics for mobile robot
id: 3-simple-kinematics-for-mobile-robot
---

> You can run this tutorial on:
>
> - [ROSbot 2.0](https://store.husarion.com/products/rosbot)
> - [ROSbot 2.0 PRO](https://store.husarion.com/collections/dev-kits/products/rosbot-pro)
> - [ROSbot 2.0 simulation model (Gazebo)](https://github.com/husarion/rosbot_description)

## A little bit of theory

### Introduction

The purpose of forward kinematics in mobile robotics is to determine robot
position and orientation based on wheels rotation measurements. To achieve that we'll create robot kinematic model. ROSbot is four wheeled mobile robot with separate drive for each wheel, but in order to simplify kinematic calculation we will treat it as two wheeled. Two virtual wheels (marked as W<sub>L</sub> and W<sub>R</sub> on the scheme) will have axis going through robot geometric center. This way we can use simpler kinematic model of differential wheeled robot. The name "differential" comes from the fact that robot can change its direction by varying the relative rate of rotation of its wheels and does not require additional steering motion. Robot scheme is presented below:

<div><center><img src="/docs/assets/img/ros/robot_scheme.png" width="50%" height="50%"/></center></div>

Description:
* R<sub>c</sub> - robot geometric centre
* x<sub>c</sub> - robot geometric centre x position
* y<sub>c</sub> - robot geometric centre y position
* x<sub>r</sub> - robot local x axis that determines front of the robot
* α - robot angular position
* W<sub>FL</sub> - front left wheel
* W<sub>FR</sub> - front right wheel
* W<sub>RL</sub> - rear left wheel
* W<sub>RR</sub> - rear right wheel
* W<sub>L</sub> - virtual left wheel
* W<sub>R</sub> - virtual right wheel
* l<sub>1</sub> - distance between robot center and front/rear wheels
* l<sub>2</sub> - distance between robot left and right wheels

Our mobile robot has constraints. It can only move in `x-y` plane and it has 3 DOF (degrees of freedom). However not all of DOFs are controllable which means robot cannot move in every direction of its local axes (e.g. it cannot move sideways). Such drive system is called **non-holonomic**. When amount of controllable DOFs is equal to total DOFs then a robot can be called **holonomic**. To achieve that some mobile robots are built using Omni or Mecanum wheels and thanks to vectoring movement they can change position without changing their heading (orientation).

### Forward Kinematics task

The robot position is determined by a tuple (x<sub>c</sub>, y<sub>c</sub>, α). The forward kinematic task is to find new robot position (x<sub>c</sub>, y<sub>c</sub>, α)'
after time _δt_ for given control parameters:
* v<sub>R</sub> - linear speed of right virtual wheel
* v<sub>L</sub> - linear speed of left virtual wheel

In our case the angular speed ω and the angular position Φ of each virtual wheel will be an average of its real counterparts:

<div><center><img src="/docs/assets/img/ros/man_3_formula_1_1.png" title="\large \phi_{W_L}=\frac{\phi_{W_{FL}}+\phi_{W_{RL}}}{2}" /></center></div>
<div><center><img src="/docs/assets/img/ros/man_3_formula_1_2.png" title="\large \phi_{W_R}=\frac{\phi_{W_{FR}}+\phi_{W_{RR}}}{2}" /></center></div>
<div><center><img src="/docs/assets/img/ros/man_3_formula_1_3.png" title="\large \omega_{W_L}=\frac{\omega_{W_{FL}}+\omega_{W_{RL}}}{2}" /></center></div>
<div><center><img src="/docs/assets/img/ros/man_3_formula_1_4.png" title="\large \omega_{W_R}=\frac{\omega_{W_{FR}}+\omega_{W_{RR}}}{2}" /></center></div>

Linear speed of each virtual wheel:

<div><center><img src="/docs/assets/img/ros/man_3_formula_2_1.png" title="\huge \huge v_{R} = \omega_{W_R} \times r" /></div></center>
<div><center><img src="/docs/assets/img/ros/man_3_formula_2_2.png" title="\huge \huge v_{L} = \omega_{W_L} \times r" /></div></center>

where _r_ - the wheel radius.

We can determine robot angular position and speed with:

<div><center><img src="/docs/assets/img/ros/man_3_formula_3_1.png" title="\huge \alpha = (\phi_{W_R} - \phi_{W_L})\frac{r}{l_2}" /></div></center>
<div><center><img src="/docs/assets/img/ros/man_3_formula_3_2.png" title="\huge \dot{\alpha}=\frac{d\alpha}{dt}" /></center></div>

Then robot speed x and y component:

<div><center><img src="/docs/assets/img/ros/man_3_formula_4_1.png" title="\huge \dot{x_c}=(v_L + \dot{\alpha}\tfrac{l_2}{2})cos(\alpha)" /></center></div>
<div><center><img src="/docs/assets/img/ros/man_3_formula_4_2.png" title="\huge \dot{y_c}=(v_L + \dot{\alpha}\tfrac{l_2}{2})sin(\alpha)" /></center></div>

To get position:

<div><center><img src="/docs/assets/img/ros/man_3_formula_5_1.png" title="\huge x_c = \int_{0}^{t}\dot{x_c}\ dt" /></center></div>
<div><center><img src="/docs/assets/img/ros/man_3_formula_5_2.png" title="\huge y_c = \int_{0}^{t}\dot{y_c}\ dt" /></center></div>

We assume starting position as (0,0).

In order for code to work correctly wheels should be connected to ports in following manner:
* front left wheel (W<sub>FL</sub>) - hMot4
* front right wheel (W<sub>FR</sub>) - hMot1
* rear left wheel (W<sub>RL</sub>) - hMot3
* rear right wheel (W<sub>RR</sub>) - hMot2

The implementation of the equations above in hFramework can be found [here](https://github.com/husarion/hFramework/blob/master/src/rosbot/ROSbot.cpp#L126).

## Controlling the motor

Most common way to send movement commands to the robot is with use of
`geometry_msgs/Twist` message type. Then motor driver node should use
data stored in them to control the motor.

The `geometry_msgs/Twist` message express velocity in free space and consists of two fields:

- `Vector3 linear` - represents linear part of velocity [m/s]
- `Vector3 angular` - represents angular part of velocity [rad/s]

You will control ROSbot in the `x-y` plane by manipulating the `x` component of linear speed vector and the `z` component of angular speed vector.

### Publishing the motion command for robot

You will use keyboard to control the movement of your robot. For getting the
key events and converting them to `geometry_msgs/Twist` messages you can
use `teleop_twist_keyboard.py` node from package
[`teleop_twist_keyboard`](http://wiki.ros.org/teleop_twist_keyboard).

Alternatively you can use joystick to control your robot, then you will
need `joy_node` node from [`joy`](http://wiki.ros.org/joy) package and
`teleop_node` node from [`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) package.

### Converting motion command to motor drive signal

In this section you will see part of default firmware for interfacing motors. Your node will subscribe to topic with `geometry_msgs/Twist` messages, drive the
motors, read encoders and publish their state to appropriate topic. Below you can see a fragment of the code that calculates the kinematics of the robot:

```cpp

void updateRosbotOdometry(RosbotDrive *drive, RosbotOdometry_t *odom, float dtime)
{
    double curr_wheel_R_ang_pos;
    double curr_wheel_L_ang_pos;
    odom->wheel_FR_ang_pos = drive->getAngularPos(MOTOR_FR);
    odom->wheel_FL_ang_pos = drive->getAngularPos(MOTOR_FL);
    odom->wheel_RR_ang_pos = drive->getAngularPos(MOTOR_RR);
    odom->wheel_RL_ang_pos = drive->getAngularPos(MOTOR_RL);
    if (drive->getRosbotDriveType() == 4)
    {
        curr_wheel_R_ang_pos = (odom->wheel_FR_ang_pos + odom->wheel_RR_ang_pos) / (2 * TYRE_DEFLATION);
        curr_wheel_L_ang_pos = (odom->wheel_FL_ang_pos + odom->wheel_RL_ang_pos) / (2 * TYRE_DEFLATION);
    }
    else
    {
        curr_wheel_R_ang_pos = odom->wheel_FR_ang_pos;
        curr_wheel_L_ang_pos = odom->wheel_FL_ang_pos;
    }
    odom->wheel_L_ang_vel = (curr_wheel_L_ang_pos - odom->wheel_L_ang_pos) / (dtime);
    odom->wheel_R_ang_vel = (curr_wheel_R_ang_pos - odom->wheel_R_ang_pos) / (dtime);
    odom->wheel_L_ang_pos = curr_wheel_L_ang_pos;
    odom->wheel_R_ang_pos = curr_wheel_R_ang_pos;
    odom->robot_angular_vel = (((odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR)) - odom->robot_angular_pos) / dtime;
    odom->robot_angular_pos = (odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR);
    odom->robot_x_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(odom->robot_angular_pos);
    odom->robot_y_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(odom->robot_angular_pos);
    odom->robot_x_pos = odom->robot_x_pos + odom->robot_x_vel * dtime;
    odom->robot_y_pos = odom->robot_y_pos + odom->robot_y_vel * dtime;
}
```

As we can see function updateRosbotOdometry does exactly the same calculation that we described in the theoretical introduction.

### Running motor controller step by step ###

In this section you will learn how to control your robot movement with
keyboard. You will need `teleop_twist_keyboard` node from
`teleop_twist_keyboard` package.

Log in to your CORE2 device through remote desktop and run terminal. In
first terminal window run `$ roscore`, in second run:

For ROSbot 2.0:
```
roslaunch rosbot_ekf all.launch
```

For PRO version add parameter:

```
roslaunch rosbot_ekf all.launch rosbot_pro:=true
```

This program is responsible for bridging your CORE2 to ROS network. When you are working with simulator, then above bridge is not necessary. Gazebo will subscribe appropriate topics automatically.
In third terminal window run:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Now you can control your robot with keyboard with following functions
for buttons:

- `’i’` - move forward

- `’,’` - move backward

- `’j’` - turn left

- `’l’` - turn right

- `’k’` - stop

- `’q’` - increase speed

- `’z’` - decrease speed

You should get similar view in `rqt_graph`:

![image](/docs/assets/img/ros/man_3_1.png)

### Running motor controller with `roslaunch`

To enable control of ROSbot with single launch file we need to prepare it or use launch from tutorial_pkg. 

Inside the `~/ros_workspace/src/tutorial_pkg/launch` create `tutorial_3.launch` with below content:

```xml
<launch>

    <arg name="use_rosbot" default="true"/>
    <arg name="use_gazebo" default="false"/>

    <include if="$(arg use_gazebo)" file="$(find rosbot_gazebo)/launch/rosbot.launch"/>    
        
        <!-- ROSbot 2.0 -->
    <include if="$(arg use_rosbot)" file="$(find rosbot_ekf)/launch/all.launch"/>

        <!-- ROSbot 2.0 PRO -->
    <!-- <include file="$(find rosbot_ekf)/launch/all.launch" >
      <arg name="rosbot_pro" value="true" />
    </include> -->

    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>

</launch>
```

We can use this launch files each time we want to start communication between SBC and CORE2.

### Running motor controller with forward kinematics task

In this section you will control your robot with keyboard and observe as it publishes its own position.

If you are working with ROSbot:
Log in to your CORE2 device through remote desktop, run terminal and start your robot as previously. In another terminal window run:

```bash
rostopic echo /pose
```

If you are working with Gazebo:
Start Gazebo as previously. In another terminal window run:

```bash
rostopic echo /odom
```

Above difference comes from the fact, that Gazebo and ROSbot are publishing its position in different ways.

Remember, that you need to have active window with `teleop_twist_keyboard` to control robot movement.

You should get something like this on your screen:

![image](/docs/assets/img/ros/man_3_2.png)

### Data visualization with PlotJuggler

In this section you will learn how to visualize data from ros topics using PlotJuggler. It is a simple tool that allows you to plot logged data, in particular timeseries. You can learn more about the tool on its [official webpage](https://github.com/facontidavide/PlotJuggler).

#### How to use

Start PlotJuggler:

```bash
rosrun plotjuggler PlotJuggler
```

In case your image lacks this tool you can install it by typing:

```bash
sudo apt-get update
sudo apt-get install ros-kinetic-plotjuggler
```

From menu bar select **Streaming > Start: ROS_Topic_Streamer**. In pop-up menu that will appear choose **/pose** from available topic names and press ok.

![image](/docs/assets/img/ros/man_3_6.png)

Pressing **CTRL** and **SHIFT** select positions:

- _/pose/pose/position/x_
- _/pose/pose/position/y_
- _/pose/pose/position/z_

and then drag and drop them to the window area. This way you can comfortably observe changes in the odometry data during robot motion:

![image](/docs/assets/img/ros/man_3_7.png)

## Robot visualization with Rviz

Rviz is tool which allows visualization of robot position, travelled path,
planned trajectory, sensor state or obstacles surrounding robot.

To run it type in terminal:

```bash
rviz
```

You can also start all nodes with single `.launch` file:

```xml
<launch>

    <arg name="use_rosbot" default="true"/>
    <arg name="use_gazebo" default="false"/>

    <include if="$(arg use_gazebo)" file="$(find rosbot_description)/launch/rosbot.launch"/>
            <!-- ROSbot 2.0 -->
    <include if="$(arg use_rosbot)" file="$(find rosbot_ekf)/launch/all.launch"/>

        <!-- ROSbot 2.0 PRO -->
    <!-- <include file="$(find rosbot_ekf)/launch/all.launch" >
      <arg name="rosbot_pro" value="true" />
    </include> -->

    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find tutorial_pkg)/rviz/tutorial_3.rviz"/>

    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>

</launch>
```

Rviz can be launched with argument pointing to file `.rviz`,  it will contain window configuration. When you are satisfied with visualization parameters, choose **File** -> **Save config**. At next `rviz` launch config will be restored.

New window will appear:

![image](/docs/assets/img/ros/man_3_3.png)

Application main view consists of:

1.  Toolbar

2.  Visualized items list

3.  Visualization window

4.  Object manipulation buttons

By default you will see only base frame, to add any other object push
**Add** from object manipulation buttons. In new window, there are two
tabs **By display type** and **By topic**. First one is for manual
selection from all possible objects, second one contains only currently
published topics.

After you choose object to display, click **OK** button and it will
appear in visualization window.

Now we will visualize position published by your robot, run `rviz`,
click **Add** and choose tab **By topic**.

<div><center><img src="/docs/assets/img/ros/man_3_4.png" /></center></div>

If you are working with ROSbot:
Find topic `/pose` and choose `Pose` and click **OK**.

If you are working with Gazebo:
Find topic `/odom` and choose `Odometry` and click **OK**.

Again click **Add**, choose tab **By display type**, **TF** and click **OK**.

Then in visualized items list find position `Fixed Frame` and from dropdown list choose `odom`.

After this is done, you should see an arrow representing position and orientation
of your robot. You will see also representation of coordinate frames bounded with robot starting position and robot base. Move your robot and observe as arrow changes its position.

![image](/docs/assets/img/ros/man_3_5.png)

Visualization of any other item is performed in the same way. In further
lessons, as you will produce more objects to visualize, you will add them
to the same view.

## Summary

After completing this tutorial you should be able to control motor
attached to your CORE2 device, set desired velocity for robot with
`geometry_msgs/Twist` message, determine position of your robot using
odometry, publish it into `tf` frames or as a `PoseStamped` message and visualize position of your robot using `rviz`.

---

_by Łukasz Mitka, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
