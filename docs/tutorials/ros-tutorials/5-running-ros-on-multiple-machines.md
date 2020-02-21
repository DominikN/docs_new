---
title: Running ROS on multiple machines
sidebar_label: 5. Running ROS on multiple machines
id: 5-running-ros-on-multiple-machines
---

> You can run this tutorial on:
>
> - [ROSbot 2.0](https://store.husarion.com/products/rosbot)
> - [ROSbot 2.0 PRO](https://store.husarion.com/collections/dev-kits/products/rosbot-pro)
> - [ROSbot 2.0 simulation model (Gazebo)](https://github.com/husarion/rosbot_description)

## Introduction

In this manual you will learn how to configure ROS to work on multiple
computers.You will use this configuration to set up system consisting of
robot and computer or two computers (gazebo version), which perform task of searching an object.

In this manual you will need computer and robot with the same equipment as in the previous manual.

In case you are working on **Gazebo** simulator, it is possible to setup system to work on multiple computers, to do it you can have two computers with ROS or virtual machine running with ROS.

## Network setup

To run ROS on multiple machines, all of them must be in the same local
network. If you want to use separated networks use [Husarnet](https://husarnet.com/)

While working on multiple machines, you need only one `roscore` running.
Choose one device for it- we will call it `master`.
Options for configuration are:

1. Husarnet :

   ROSbots are coming preconfigured with [Husarnet](https://husarnet.com/) support, which provide low-latency, secure connection between robots. Husarnet is a P2P Virtual LAN network, so from your ROSbots point of view, they are in the same LAN network even if they are not physically in the same network. Husarnet setup is covered in [Connecting through Husarnet](#connecting-through-husarnet) section.

2. .Bashrc

    On the master device open the `.bashrc` file:

   ```bash
   nano ~/.bashrc
   ```

   To disable husarnet entries, find lines:

   ```bash
   export ROS_MASTER_URI=http://master:11311
   export ROS_IPV6=on
   ```
   and comment them with `#`:

   ```bash
   #export ROS_MASTER_URI=http://master:11311
   #export ROS_IPV6=on
   ```

   Then add two lines at file ending, replacing `X.X.X.X` and `Y.Y.Y.Y` with IP address of master device.

   ```bash
   export ROS_MASTER_URI=http://X.X.X.X:11311
   export ROS_IP=Y.Y.Y.Y
   ```

   On second robot also open the `.bashrc` file, comment Husarnet entries and add two lines at file ending. This time replace `X.X.X.X` with IP address of master device and `Y.Y.Y.Y` with IP address of second robot.

3. Hostname + .bashrc

    Last option is to use hostname, which is more convenient than using physical addresses. To do it instead of using physical address in `.bashrc` use:

   ```bash
   export ROS_MASTER_URI=http://computer-master:11311
   export ROS_IP=robot
   ```

   Now we have to tell what this hostname means. To do it edit file `/etc/hostnames` with sudo privileges:

   ```bash
   127.0.0.1	localhost
   X.X.X.X     computer-master 
   X.X.X.X     robot
   ```
One benefit of using husarnet is that it manages hostnames so you don't have to worry about it.


TIP! If you do changes in .bashrc file always source it with `. ~/.bashrc` or reopen terminal

TIP 2! Remember that `roscore` must be running on the device indicated as ROS master!!!

**Task 1**

As a remember:
-  astra.launch - `roslaunch astra_launch astra.launch`
-  rqt_graph - `rosrun rqt_graph rqt_graph`
-  image_view - `rosrun image_view image_view image:=/camera/rgb/image_raw`

Set configuration for working on multiple machines on two devices.
On the device named **master** launch `roscore`. First check if connection works simple with `rostopic list` command on second device - if you are unable to see topics then reconfigure your network communication.
On the second device run `astra.launch` and
`image_view` nodes. Now run `rqt_graph` on both of them, you should see
the same graph.

Next run `image_view` node on machine with `roscore`. You should see
the image from camera mounted on the device with `astra.launch` running.
Again use `rqt_graph` to examine what changed in the system.

In case of using gazebo:

Configure connection the same way ether with husarnet or manually by yourself. Next launch `roscore` on the device named **master**,
launch gazebo simulation `roslaunch tutorial_pkg tutorial_4.launch use_rosbot:=false use_gazebo:=true teach:=true recognize:=false` and `image_view` nodes. Examine connection with `rqt_graph`. 


Now let's use second machine. First check if connection works simple with `rostopic list` command on second device - if you are unable to see topics then reconfigure your network communication. Next run `image_view` node on the second machine. You should see
the image from camera running inside gazebo running.
Again use `rqt_graph` to examine what changed in the system.


## Performing a task with multiple machines

**Task 2**

In this section we will program robot and computer to outsource computer power.
To do that we will need one node for image processing - it will run on computer and we will need a node that
publishes camera stream - it will run on ROSbot. For recognizing
objects we will use `find_object_2d` node. 

### Running task

We will follow the pattern form previous tutorial but this time we do it using two machines. As a remember anything could be an object to recognize, 
but remember, that the more edges and contrast colours it has, the easier it will be recognized. A piece
of paper with something drawn on it would be enough for this tutorial.

1. Start `roscore`, open terminal on your computer (master) and type `roscore` 

2. On ROSbot you should run `astra.launch` `image transport` and `bridge to CORE2`.

   For connection to CORE2 we will use package `rosbot_ekf` if you haven't done `ROSbot - quick start` follow this instruction:

   Go to your workspace source directory
   ```
   cd ~/ros_workspace/src
   ```
   Clone rosbot_ekf repository:
   ```
   git clone https://github.com/husarion/rosbot_ekf.git
   ```
   Install dependencies required by rosbot_ekf package:
   ```
   sudo apt-get install ros-kinetic-robot-localization
   ```
   Change directory and build code using catkin_make:
   ```
   cd ~/ros_workspace
   catkin_make
   ```

   To launch rosserial communication and Kalman filter for mbed firmware run:

   ```
   roslaunch rosbot_ekf all.launch
   ```

   For PRO version add parameter:

   ```
   roslaunch rosbot_ekf all.launch rosbot_pro:=true
   ```

3. On computer (master) we will launch also `image transport` and `find_object_2d` node. 

Image transport node provides compressed image delivery from one device to another, on sending machine you have to run this node for compression and on reciving machine you have to decompress image. Before we used `image view` without any compression which is enough just for see if it's working, but if only you want to implement some real time image processing it's necessary to compress image. 

## Launch 

Create launch file under `tutorial_pkg/launch` and name it as `tutorial_5_rosbot.launch`:

```xml
<launch>

    <arg name="teach" default="true"/>
    <arg name="recognize" default="false"/>

    <include file="$(find astra_launch)/launch/astra.launch"/>
    
        <!-- ROSbot 2.0 -->
    <include file="$(find rosbot_ekf)/launch/all.launch"/>

        <!-- ROSbot 2.0 PRO -->
    <!-- <include file="$(find rosbot_ekf)/launch/all.launch" >
      <arg name="rosbot_pro" value="true" />
    </include> -->

    <node pkg="image_transport" type="republish" name="rgb_compress" args=" raw in:=/camera/rgb/image_raw compressed out:=/rgb_republish"/>

</launch>
```

And `launch` file for PC computer, name it `tutorial_5_pc.launch: 

```xml
<launch>

    <arg name="teach" default="true"/>
    <arg name="recognize" default="false"/>

    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>

    <node pkg="image_transport" type="republish" name="rgb_decompress" args=" compressed in:=/rgb_republish raw out:=/rgb_raw" >
        <param name="compressed/mode" value="color"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/rgb_raw"/>
        <param name="gui" value="$(arg teach)"/>
        <param if="$(arg recognize)" name="objects_path" value="$(find tutorial_pkg)/image_rec/"/>
    </node>

</launch>
```
Use `rqt_graph` to see how the system is working now.

That's all you don't have to worry about nothing more than properly configuration of .bashrc rest stays the same. If you have more than 2 devices the pattern is the same. ROS takes care of all data exchange. That is really simple, isn't it?

### Running the nodes in Gazebo

Gazebo node will be running on one machine. Second machine will be responsible for image processing.

At one machine use following launch file and save it as `tutorial_5_gazebo_1.launch`:

```xml
<launch>

    <arg name="teach" default="false"/>
    <arg name="recognize" default="true"/>

    <arg if="$(arg teach)" name="chosen_world" value="rosbot_world_teaching"/>
    <arg if="$(arg recognize)" name="chosen_world" value="rosbot_world_recognition"/>

    <include  file="$(find rosbot_gazebo)/launch/$(arg chosen_world).launch"/>
 
    <node name="rosbot_spawn" pkg="gazebo_ros" type="spawn_model" output="log" args="-urdf -param robot_description -model rosbot_s -y 0" />

    <node pkg="image_transport" type="republish" name="rgb_compress" args=" raw in:=/camera/rgb/image_raw compressed out:=/rgb_republish"/>

</launch>

```

At the other machine use this launch file and save it as `tutorial_5_gazebo_2.launch`:

```xml

<launch>
    <arg name="teach" default="true"/>
    <arg name="recognize" default="false"/>

    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>

    <node pkg="image_transport" type="republish" name="rgb_decompress" args=" compressed in:=/rgb_republish raw out:=/rgb_raw" >
        <param name="compressed/mode" value="color"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/rgb_raw"/>
        <param name="gui" value="$(arg teach)"/>
        <param if="$(arg recognize)" name="objects_path" value="$(find tutorial_pkg)/image_rec/"/>
    </node>

</launch>
```

Use `rqt_graph` to see how the system is working now.

## Connecting through Husarnet

In the case when more sophisticated network setup is required, e.g. robots will be connected to internet using LTE modem or different WiFi networks. For easy connecting robots in advanced network configurations, you can use [Husarnet](https://husarnet.com/) - the global LAN network for secure P2P connection between robots and IoT devices.

### Connection setup

Log in to [Husarnet Dashboard](https://app.husarnet.com/) or create an account if you don't have it yet.

You should see Husarnet Dashboard with no networks nor elements:

![husarnet-dashboard-empty](/docs/assets/img/ros/husarnet_empty_dashboard.png)

Push button "Create network" and in dialog type desired network name into field `Network name`:

![husarnet-add-network-dialog](/docs/assets/img/ros/husarnet_add_network_dialog.png)

After pushing button "Create", you will be redirected to network view:

![husarnet-empty-network](/docs/assets/img/ros/husarnet_empty_network.png)

You can use button "Add element" to add to your network cloud elements or mobile app, but now we will use terminal method.

#### Adding ROS device to network

Before you add device to network, it is required to setup the environment.

Open `.bashrc` file and find lines that you added at the beginning of this tutorial:

```bash
export ROS_MASTER_URI=http://X.X.X.X:11311
export ROS_IP=Y.Y.Y.Y
```

and replace them with:

```bash
export ROS_MASTER_URI=http://master:11311
export ROS_IPV6=on
```

ROS_IPV6 makes ROS enable IPv6 mode - Husarnet is a IPv6 network.
Setting ROS_MASTER_URI to http://master:11311 ensures ROS will always connect to host called master - which exactly machine it is depends on the setting on the Husarnet Dashboard.

Execute command:

```bash
sudo husarnet websetup
```

You will get response similar to:

```
Go to https://app.husarnet.com/husarnet/fc94cd22622bf708b9bb22d5589275fa8832943ffdb0175bff7e16ce to manage your network from web browser.
```

Open the provide link in web browser, you will see device configuration dialog:

![husarnet-add-device](/docs/assets/img/ros/husarnet_add_device_dialog.png)

Type desired name of the device into field `Name for this device`, you will use this name to distinguish your devices in dashboard.
From `Add to network` dropdown menu choose name of network that you created in previous step.

Repeat procedure of adding device with second robot.

#### Setting the master

After adding both robots, your network should look like below:

![husarnet-two-elements](/docs/assets/img/ros/husarnet_two_elements.png)

You can set device to be master in its settings. Choose device you want to be master and open configuration dialog by clicking its name, status or address:

![husarnet-set-master](/docs/assets/img/ros/husarnet_set_master.png)

Check `ROS master` checkbox and push button "Update".

When you will start `roscore` on master, message `ROS master (roscore) is not running on robot-2.` will be gone.

Your first Husarnet network is configured and ready:

![husarnet-network-ready](/docs/assets/img/ros/husarnet_network_ready.png)

### Running the nodes with Husarnet

Husarnet provide encrypted and direct virtual network for your devices, but it does not modify the ROS workflow. Just go back to section "Running the nodes on ROSbots", start required nodes and observe as your robots perform the task.

## Summary

After completing this tutorial you should be able to configure your
CORE2 devices to work together and exchange data with each other. You
should also know how to program robots for performing tasks in
cooperation.

---

_by Łukasz Mitka and Adam Krawczyk, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
