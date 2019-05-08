---
title: ROSbot + ROSDS Quick Start
sidebar_label: 7. ROSbot + ROSDS Quick Start
id: rosbot-rosds-quick-start
---

This tutorial will walk you through connecting ROSbot to [ROS Development Studio (RDS)](https://rds.theconstructsim.com/) cloud service.
We will show you, how to setup ROSbot to be controlled remotely with application hosted in cloud.

## Step 1: ROSbot configuration

ROSbot configuration is required only once, to proceed with setup, you will need to connect with ROSbot through SSH or remote desktop and issue all commands in terminal.

Clone RDS repository for real robot connection and install dependencies:

```bash
mkdir RDS
cd RDS
git clone https://bitbucket.org/theconstructcore/rosds_real_robot_connection.git
cd rosds_real_robot_connection/
./realrobot_setup.sh rosbot
sudo -H python -m pip install npyscreen
```

We will use Husarion `tutorial_pkg` as an example, but you can configure in the same way any other ROS application. 

Wa assume that you are working on fresh ROSbot image. If you were following any other Husarion tutorials, then you may have aready installed `husarion_pkg` repository, in that case, you can skip this workspace setup.

Prepare workspace and clone repository:

```bash
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace
git clone https://github.com/husarion/tutorial_pkg.git
cd ~/ros_workspace
catkin_make
echo '. ~/ros_workspace/devel/setup.sh' >> ~/.bashrc
. ~/.bashrc
```

## Step 2: Conecting ROSbot to RDS cloud

Below setup is required each time ROSbot is connecting to RDS:

Connect with ROSbot through SSH or remote desktop and issue below commands:

```bash
cd ~RDS/rosds_real_robot_connection/
sudo python real_robot_cli_ui.py
```

- In **Device Name** field provide `rosbot`.
- From ***Pick One** list choose **`( ) ON`**.
- Field **Robot URL** will get filled with link, copy it for further use
- Choose **OK** to Exit

![ROSbot config](/docs/assets/img/rosds-tutorials/rosbot_config.png)

ROSbot is configured to wait for connection form RDS.

It is required to start example application, we will use SLAM and path planning:

```bash
roslaunch tutorial_pkg tutorial_7_core.launch
```

In another terminal execute: 
```bash
/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2 
```

## Step 3: Working on RDS

Open [ROSject](http://www.rosject.io/l/97f593a/) prepared for this turorial.

Open menu **Real Robot** and choose button **Connect to Robot**

![Real Robot 1](/docs/assets/img/rosds-tutorials/rds_connect_to_real_robot.png)

In dialog menu provide `rosbot` in **Robot name** field and in **Robot URL* paste link that you have copied earlier.

![Real Robot 2](/docs/assets/img/rosds-tutorials/rds_connect_to_real_robot_2.png)

When process is finished, you should see menu allowing you to choose master device, select **rosbot**.

![Real Robot 3](/docs/assets/img/rosds-tutorials/rds_connect_to_real_robot_3.png)

Select **Tools** -> **Shell** to open terminal view and launch visualization tools:

```bash
roslaunch rosbot_description model_preview.launch
```

Select **Tools** -> **Graphical Tools**

![Rviz](/docs/assets/img/rosds-tutorials/rds_rviz_2.png)

You will see visualization of ROSbot, laser scans and generated map.

Use **2D Nav Goal** tool to set robot destination.

<iframe width="560" height="315" src="https://www.youtube.com/embed/41f40i_Au2c" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

You can observe as real ROSbot drives to point selected on map. You may also observe as Lidar scan are changing accordingly to ROSbot position and reflect real obstacles around device.