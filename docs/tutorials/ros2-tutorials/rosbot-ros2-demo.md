---
title: ROS 2 Demo for ROSbot 2.0
sidebar_label: 0. ROSbot Demo
id: rosbot-ros2-demo
---
<!-- 
TODO: add to sidebar.json below "ROS projects section"
      {
        "type": "subcategory",
        "label": "ROS 2 tutorials",
        "ids": [
          "tutorials/ros2-tutorials/rosbot-ros2-demo"
        ]
      }, -->

## Install ROS 2 image on ROSbot

### Get a system image

Working on your laptop, visit https://husarion.com/downloads/, find **Ubuntu 20.04 + ROS2 Foxy + Docker + Husarnet client** and download:
- **Tinker Board** version for ROSbot 2.0 
- **UpBoard** version for ROSbot 2.0 PRO

### Flash the image

To flash the image on ROSbot 2.0 or ROSbot 2.0 PRO, visit [system reinstallation guide](https://husarion.com/manuals/rosbot/#system-reinstallation).

## First ROSbot Configuration

### Connecting to Wi-Fi

In the ROSbot 2.0 set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter
4. To connect with ROSbot via ssh, type in terminal application on the computer: 
    ```bash
    ssh husarion@192.168.0.1
    ```
    a password is also`husarion`

5. To connect to a Wi-Fi network in the terminal type 
    ```bash
    nmcli d wifi connect <WiFi-SSID> password <WiFi-PASSWORD>
    ``` 
6. type `ip a` to find your IP address. Save it for later.

### Remote Desktop (optional)

Remote Desktop is a tool enabling user to connect to a computer in another location and see that computer screen and interact using graphical interface.

By default a VNC server is not running on ROSbot. To start VNC first connect via ssh to robot and then execute following command:

```bash
sudo systemctl start vncserver.service
```

To make use of it you have to ether install [tigervnc](https://tigervnc.org/) or use docker containers [link]](https://github.com/husarion/rosbot-remote) that we have prepared. Clone project to your computer and build. 

```bash
git clone https://github.com/husarion/rosbot-remote.git
cd rosbot-remote/tiger_vnc
./build_image.sh
./run_image.sh <ip-address-of-rosbot>
```

For more information please take a look at [readme](https://github.com/husarion/rosbot-remote/blob/main/README.md). 

If you are unfamiliar with docker or don't know how to install please use [nvidia guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (to run images it is compulsory to have nvidia-docker2 package installed)

## Launching a demo project

**NOTE**: If this is your first launch of ros2 make sure to upgrade firmware to at least [0.14.3 version](https://files.husarion.com/images/rosbot-2.0-fw-v0.14.3.bin) (pay attention because version for Noetic is different). Information about flashing you can find [here](/manuals/rosbot/#low-level-firmware-installation)

Once you are connected to ROSbot ether with ssh or remote desktop launch terminal and type following command:

```bash
ros2 launch rosbot_description navigation_demo.launch.py
or
ros2 launch rosbot_description navigation_demo_pro.launch.py #ROSbot pro
```

## Using a demo

Once ROSbot launched lidar should begin to rotate and logs starts to pop out on terminal. At this point to make your ROSbot move or see how map is being creating you need to launch rviz2. We also prepared ready to run docker container with rviz2 and configuration. Instruction is very similar to this with tigervnc client and if you already cloned the repository there is no need to do this again. 

```bash
git clone https://github.com/husarion/rosbot-remote.git
cd rosbot-remote/rviz
./build_image.sh
./run_image.sh
```

If you have ros2 Foxy installed on your system just type `rviz2` to launch rviz.

Add displays you want but for navigation add at least map and select global frame to map.

![image](/docs/assets/img/ros2-tutorials/rviz2.png)

To add destination use green arrow in top bar. 

![image](/docs/assets/img/ros2-tutorials/rviz2-nav2.png)

**NOTE** Currently dds implementations are heavy on resources so please limit shown displays. 

### Using many ROS2 devices it the same network (Optional)

ROS2 (DDS) provide a concept of multicast for machines discovery. As a result every machine running ROS2 can see every nodes/topics on all running machines in the same network. 

If you want to limit this behavior export `ROS_DOMAIN_ID` this might be number 0-255 (default is 0). If you want to communicate only two machines add to ~/.bashrc following line. 

```bash
echo "export ROS_DOMAIN_ID=14" >> ~/.bashrc
```

Other option is to follow our guide for set up [Husarnet with ROS2](https://husarnet.com/docs/tutorial-ros2/). This will enable you to communicate using p2p VPN from any network.  

---

_by Adam Krawczyk, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
