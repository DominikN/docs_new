---
sidebar_label: 2. Control virtual robot from your PC
id: control-virtual-robot-from-your-PC
title: ROSDS Virtual robot controlled from your local PC
---

### NOTE: You can run the following project only in a virtual environment do it, by clicking on the ROSDS button

<div><center>
<a href="http://www.rosject.io/l/c1db44b/">
<img alt="run-on-ROSDS" src="/img/ros/Run-on-ROSDS-button.png" width="250px"/></a>
</center></div>

## About

In this ROSject we will show you how to control a virtual robot operating in the ROS Development Studio environmet from a level of your laptop running ROS.

![theme](https://user-images.githubusercontent.com/29305346/64530720-224d3980-d30e-11e9-9901-ee5ecbd58776.png)

To control a robot the [Route Admin Planner](/software/route-admin-panel/) provided by Husarion is used.
The Route Admin Panel is a web user interface for managing routes of ROS based mobile robots.

It allows to:

- Define destination points
- Save robot position as destination point
- Send destination point to move_base
- Upload custom map
- Set a sequence of destination points

The Route Admin Panel is built as a Node.js application. On one side it is interfacing with ROS topics, while on another side it presents a frontend for managing robot destinations.

In this project you will find a simulation of ROSbot 2.0 controlled by Route Admin Panel. With it you will be able to controll virtual robot (not only ROSbot). If you can make it work in this ROSject, you will definetly be able to do it in the real robot.

## Establish the connection from your PC side

You need to have connection from ROS Development Studio to your computer for doing it follow instructions

On your computer open new terminal and paste following commands:

```
git clone https://bitbucket.org/theconstructcore/rosds_real_robot_connection.git
cd rosds_real_robot_connection
sudo ./realrobot_setup.sh
sudo reboot
```



Once your computer finished rebooting open new terminal and type:

`whoami –> It will give you the user_name_in_device.`  <-- remember this result we will use it later

Open web browser tab for seting up connection.

Type in the URL: `localhost:3000`

Now you have to click on Turn ON. This will generate the Robot URL that you need to make the connection in ROSDS.

![rosds_rap_connection_web1](https://user-images.githubusercontent.com/29305346/64002117-96ffc700-cb09-11e9-82af-426e625468b0.png)

## Establish the connection from ROSDS side

Open ROSject if you don't have, do it using button on the top of the site.

You have to click the RealRobot tab -> Connect to Robot -> **ON**, and after a few minutes, you will be greeted with the configuration window

Once window pop up place in the corresponding form the PC URL (generated in previous step) and the Robot Name (computer name).

![rosds_rap_connection_name](https://user-images.githubusercontent.com/29305346/64002125-99622100-cb09-11e9-834f-6abd31f13deb.png)

## Test that everything is working

On your computer open terminal and test connection using _ping6_ command:

```
ping6 rosdscomputer 
```

If it's working you can launch simulation.

## Launch simulation

In the RealRobot tab assign rosmaster to rosdscomputer.

![rosds_rap_connection_rosmaster](https://user-images.githubusercontent.com/29305346/64002120-9830f400-cb09-11e9-8764-a112fd7a2b68.png)

To start simulation, go to "Simulations" menu, then choose Choose launch file... -> rosbot_launch -> `route_admin_planner.launch` . 

![rosds_rap_launch](https://user-images.githubusercontent.com/29305346/64002109-95360380-cb09-11e9-8dd5-f1fb221df5fa.png)

After simulation starts properly open new tap in web browser and paste following url:

```
http://rosdscomputer:8000/
```

You should see webui to add new goal click _Add target_ chose point on map, name point and click _>>_. To make robot follow route add more points and click _set route_  

![rosds_rap_webui](https://user-images.githubusercontent.com/29305346/64002105-936c4000-cb09-11e9-8fb7-128943545f1a.png)

NOTE: If you can't see any map add target close to robot to move it a litte.

## Other devices

You can run this to controll real robot, to do, so follow this tutorial:

[Route Admin Planner](/software/route-admin-panel/)

---
_by Adam Krawczyk, Husarion_

Do you need any support with completing this project or have any difficulties? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com