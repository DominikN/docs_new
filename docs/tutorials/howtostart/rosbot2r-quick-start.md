---
title: ROSbot 2R - quick start
sidebar_label: 3. ROSbot 2R
id: rosbot2r-quick-start
---

ROSbot 2R is an autonomous, open source robot platform for research and quick prototyping use cases. It can be used as a learning platform for Robot Operating System (ROS) as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can get it [here](https://store.husarion.com/).

## Unboxing

What's in the box:

- carrying case
- ROSbot 2R (with optional 3D camera and LiDAR already assembled)
- Wi-Fi 2.4GHz / 5GHz antenna
- 3x 18650 Li-Ion rechargeable batteries
- universal charger with power adapter
- charging cable
- microSD card with the software for ROSbot
- USB to Ethernet adapter

![](/img/howToStart/ROSbot_unboxing.jpg)

## Assembly

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna.

To mount batteries:

- turn ROSbot upside down
- unscrew battery cover mounted with two screws
- remove the battery cover
- place batteries accordingly to the symbols, keeping the black strip under the batteries
- place battery cover and mount it with screws

To charge the batteries, follow this [guide](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf).

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.


## Installing ROSbot's system image

Download the lates [ROSbot 2R ubuntu 20.04 system image](https://testing-github-action.s3.eu-west-1.amazonaws.com/rosbot-rpi-ubuntu-20.04-2022-07-28.img.xz) and burn it on the SD card with [Etcher](https://www.balena.io/etcher/).


## Accessing ROSbot's Linux terminal

To perform the initial network configuration, you need to access ROSbot's Linux terminal first. There are two options:

### Option 1: Using display, mouse and keyboard

ROSbot is basically a computer running Ubuntu, so let's open it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Open a `terminal` app.

### Option 2: Using an Ethernet adapter

In the ROSbot 2R set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in the Ethernet adapter (included in a set) to a USB port in the rear panel.
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter.
4. Set a static IP address **on your computer for its Ethernet card** in a `192.168.77.0/24` subnet, eg:

    - IPv4: `192.168.77.27`
    - mask: `255.255.255.0`

4. To connect with ROSbot via ssh, type in your terminal application: 

    ```bash title="user@mylaptop:~$ ..."
    ssh husarion@192.168.77.2
    ```

    The default password for user `husarion` is also `husarion`.

## Connecting ROSbot to your Wi-Fi network

ROSbot 2R is using [netplan](https://netplan.io/) instead of GUI Wi-Fi manager. It allows you to have all physical network interfaces configured from a single text file. 

To connect your ROSbot to a Wi-Fi network edit `/etc/netplan/01-network-manager-all.yaml` file, eg. with `nano`:

```bash title="husarion@rosbot2r:~$ ... "
sudo nano /etc/netplan/01-network-manager-all.yaml
```

And modify lines 31-32 by replacing `PLACE_YOUR_WIFI_SSID_HERE` with your SSID (Wi-Fi network name) and `PLACE_YOUR_WIFI_PASSWORD_HERE` with your Wi-Fi password:

```yaml {31-32} showLineNumbers title="/etc/netplan/01-network-manager-all.yaml"
network:
  version: 2
  renderer: networkd

  ethernets:
    
    # built-in Ethernet port
    eth0:
      dhcp4: no
      dhcp6: no
      addresses:           
       - 192.168.77.2/24

    # USB Ethernet card
    eth1:
      dhcp4: no
      dhcp6: no
      addresses:           
       - 192.168.77.2/24

  wifis:

    # wlan0:  # internal Raspberry Pi Wi-Fi (do not use it when the top cover is used)
    #   optional: true

    wlan1:  # external USB Wi-Fi card (with antenna)
      dhcp4: true
      dhcp6: true
      optional: true
      access-points:
        "PLACE_YOUR_WIFI_SSID_HERE":
          password: "PLACE_YOUR_WIFI_PASSWORD_HERE"
```

**save the file** then, apply the new network setup:

```bash title="husarion@rosbot2r:~$ ... "
sudo netplan apply
```

You can check to which Wi-Fi network your ROSbot us connected by using this command:

```bash title="husarion@rosbot2r:~$ ... "
iwgetid
```

If your Wi-Fi network setup is more complex (eg. if you want to connect to [Eduroam](https://en.wikipedia.org/wiki/Eduroam) based Wi-Fi that is popular in many universities), visit [netplan configuration examples](https://netplan.io/examples).

Open Linux terminal and type `ifconfig` to find your IP address (for `wlan1` network interface). Save it for later.

## Remote access in LAN

While ROSbot is connected to a Wi-Fi network, you can access it by using its IPv4 address by SSH:

```bash title="user@mylaptop:~$ ..."
ssh husarion@ROSBOT_IP
```

## Remote access over the Internet (VPN)

Instead of using a local IPv4 address you can access the robot by using it's hostname - both in LAN and over the Internet. You just need to setup a VPN connection (Husarnet VPN client is pre-installed)

### Get the Join Code for your Husarnet network:

You will find your Join Code at **https://app.husarnet.com**  

 **-> Click on the desired network  
 -> `Add element` button  
 -> `Join Code` tab**

![](/img/howToStart/husarnet.png)

### Connect your laptop

Install Husarnet VPN client on your laptop:

```bash title="user@mylaptop:~$ ..."
curl https://install.husarnet.com/install.sh | sudo bash
```

```bash title="user@mylaptop:~$ ... "
sudo systemctl restart husarnet
```

```bash title="user@mylaptop:~$ ... "
sudo husarnet join <your_join_code_from_step_1> mylaptop
```

### Connect your ROSbot 2R

```bash title="husarion@rosbot2r:~$ ... "
sudo systemctl enable husarnet
```

```bash title="husarion@rosbot2r:~$ ... "
sudo systemctl start husarnet
```

```bash title="husarion@rosbot2r:~$ ... "
sudo husarnet join <your_join_code_from_step_1> rosbot2r
```

### Test the connection

That's all - now you can use your device hostname instead of IPv4 addr, eg.:

```bash title="user@mylaptop:~$ ... "
ssh husarion@rosbot2r
```

After that you should see:
```
///////////////////////////////////////////  ____   ___  ____  _           _   
///////////////////////o/////////////////// |  _ \ / _ \/ ___|| |__   ___ | |_ 
//////////////////////yMs////////////////// | |_) | | | \___ \| '_ \ / _ \| __|
///////////////sNh+///NMm////////////////// |  _ <| |_| |___) | |_) | (_) | |_ 
////////////////NMNs/oMMM////////////////// |_| \_\\___/|____/|_.__/ \___/ \__|
//////////++////oNMMy/mNd//////////////////                                    
//////////omNmdyoodms//+/////////////////// 
////////////ymMMNh///////////////////////// 
/////////////+oyho/shh///////////////////// 
////////ohmmNNmmy//dMM///////////////////// 
/////////+oyhdmmy//dMM///////////////////// 
//////////+++ooo///dMM///////////////////// 
////////ohmNMMMMd//dMM/////ymd/////////////  husarion@rosbot2r
/////////+oosyys+//dMM/////mMN/////////////  =================
/////////+syhhhho//dMM+++++mMN/////////////  Website: https://husarion.com/
////////ohmNNNNNd//dMMNNNNNMMN/////////////  OS: 20.04.4 LTS (Focal Fossa)
/////////////+++///dMMsssssNMN/////////////  Kernel: 5.4.0-1052-raspi
///////////////////dMM/////mMN/////////////  Board: None
///////////////////dMN/////dMN/////////////  Uptime: 3m.
///////////////////////////////////////////  Memory: 17070MB / 29677MB (58%)
///////////////////////////////////////////

husarion@rosbot2r:~$ 
```

## ROSbot ROS packages

At this stage you should have your ROSbot up and running, with remote (in LAN or VPN) connection from your laptop.

You can run ROSbot's ROS packages natively on your ROSbot's host OS, by just clonning and building [rosbot_ros](https://github.com/husarion/rosbot_ros) repo, however the more convenient way is using Docker images for that.

### Pulling the ROSbot docker image

Access your ROSbot's terminal and run:

```bash title="husarion@rosbot2r:~$ ... "
docker pull husarion/rosbot:noetic
```

:::note

There is also a version for ROS Melodic: to pull it, just replace `noetic` with `melodic` tag

:::

### Flashing the firmware

Both `husarion/rosbot:noetic` and `husarion/rosbot:melodic` docker images include corresponding firmware for STM32F4 (a low level microcontroller that controlls motors, GPIO ports and TOF distance sensors).

To flash the right firmware, open ROSbot's terminal and execute one of the following command (if you use ROS noetic tag):
   
- for differential drive (regular wheels):
   
  ```bash title="husarion@rosbot2r:~$ ... "
  docker run --rm -it --privileged \
  husarion/rosbot:noetic \
  /flash-firmware.py /root/firmware_diff.bin
  ```
- for omnidirectional wheeled ROSbot (mecanum wheels):

  ```bash title="husarion@rosbot2r:~$ ... "
  docker run --rm -it --privileged \
  husarion/rosbot:noetic \
  /flash-firmware.py /root/firmware_mecanum.bin
  ```

### Running the ROSbot Docker image

Paste the following command in the ROSbot's terminal:

```bash title="husarion@rosbot2r:~$ ... "
docker run --rm -it \
--device /dev/ttyAMA0 \
--network host \
-e SERIAL_PORT=/dev/ttyAMA0 \
-e ROS_IP=127.0.0.1 \
-e ROS_MASTER_URI=http://127.0.0.1:11311 \
husarion/rosbot:noetic \
roslaunch rosbot_bringup rosbot_docker.launch
````

Now open the second terminal and check avaialable ROS topics:

```bash
husarion@rosbot2r:~$ rostopic list
/battery
/buttons
/cmd_ser
/cmd_vel
/diagnostics
/imu
/joint_states
/odom
/odom/wheel
/pose
/range/fl
/range/fr
/range/rl
/range/rr
/rosout
/rosout_agg
/set_pose
/tf
/tf_static
/velocity
```

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

```bash
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
```

## Next steps

Now you know how to access ROSbot's ROS node from the Linux terminal.

Running ROS natively is fine for a relatively small projects. For more complex ones, the full Dockerized setup is the better approach.

We have created a refference project to show you how to run autonomous mapping and navigation on ROSbot with using Navigation2 and SLAM Toolbox. Everything is Docker based, sor running it is straightforward.

![ROSbot with Navigation2 and Slam Toolbox](/img/howToStart/rviz_mapping.png)

Find the full source code and instructions [here](https://github.com/husarion/rosbot-docker/tree/ros1/demo)